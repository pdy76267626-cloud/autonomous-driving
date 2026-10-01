# -*- coding: utf-8 -*-

# ------------------------------
# TIKI-mini + RPLIDAR C1
# 장애물 회피 로봇
# ------------------------------

import asyncio
import time

from tiki.mini import TikiMini
from rplidarc1.scanner import RPLidar


# ------------------------------
# 설정
# ------------------------------

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

# 평상시 전진 속도
FORWARD_SPEED = 15

# 회전 속도
TURN_SPEED = 15

# 이 거리보다 가까우면 장애물로 판단
OBSTACLE_DISTANCE = 50.0

# 너무 가까운 경우 긴급 정지
EMERGENCY_DISTANCE = 20.0

# 회전하는 시간
TURN_TIME = 0.8


# ------------------------------
# 거리 계산용 변수
# ------------------------------

front_distance = None
left_distance = None
right_distance = None


# ------------------------------
# 거리값 업데이트
# ------------------------------

def update_distance(angle, distance):

    global front_distance
    global left_distance
    global right_distance

    # 거리값이 없는 데이터는 무시
    if angle is None or distance is None:
        return

    # mm → cm
    distance_cm = distance / 10.0

    # 너무 이상한 값은 무시
    if distance_cm <= 0:
        return

    # ------------------------------
    # 앞쪽
    # 330~360도 + 0~30도
    # ------------------------------

    if angle >= 330 or angle <= 30:

        if front_distance is None:
            front_distance = distance_cm
        else:
            front_distance = min(
                front_distance,
                distance_cm
            )

    # ------------------------------
    # 왼쪽
    # 60~120도
    # ------------------------------

    elif 60 <= angle <= 120:

        if left_distance is None:
            left_distance = distance_cm
        else:
            left_distance = min(
                left_distance,
                distance_cm
            )

    # ------------------------------
    # 오른쪽
    # 240~300도
    # ------------------------------

    elif 240 <= angle <= 300:

        if right_distance is None:
            right_distance = distance_cm
        else:
            right_distance = min(
                right_distance,
                distance_cm
            )


# ------------------------------
# 한 번의 스캔 결과 처리
# ------------------------------

def reset_distances():

    global front_distance
    global left_distance
    global right_distance

    front_distance = None
    left_distance = None
    right_distance = None


# ------------------------------
# RPLIDAR 데이터 처리
# ------------------------------

async def lidar_task(lidar):

    global front_distance
    global left_distance
    global right_distance

    while not lidar.stop_event.is_set():

        if not lidar.output_queue.empty():

            data = await lidar.output_queue.get()

            angle = data["a_deg"]
            distance = data["d_mm"]

            update_distance(
                angle,
                distance
            )

        else:

            await asyncio.sleep(0.005)


# ------------------------------
# 로봇 주행 제어
# ------------------------------

async def robot_control(tiki):

    global front_distance
    global left_distance
    global right_distance

    while True:

        # ------------------------------
        # 아직 거리 데이터가 없는 경우
        # ------------------------------

        if front_distance is None:

            tiki.stop()

            await asyncio.sleep(0.05)

            continue

        # ------------------------------
        # 현재 거리 출력
        # ------------------------------

        print(
            "앞: %s cm | 왼쪽: %s cm | 오른쪽: %s cm"
            % (
                "%.1f" % front_distance
                if front_distance is not None
                else "---",

                "%.1f" % left_distance
                if left_distance is not None
                else "---",

                "%.1f" % right_distance
                if right_distance is not None
                else "---"
            )
        )

        # ------------------------------
        # 긴급 정지
        # ------------------------------

        if front_distance <= EMERGENCY_DISTANCE:

            print("!!! 긴급 정지 !!!")

            tiki.stop()

            await asyncio.sleep(0.5)

            continue

        # ------------------------------
        # 장애물이 없는 경우
        # ------------------------------

        if front_distance > OBSTACLE_DISTANCE:

            print("전진")

            tiki.forward(FORWARD_SPEED)

            await asyncio.sleep(0.1)

            continue

        # ------------------------------
        # 장애물 발견
        # ------------------------------

        print("장애물 발견!")

        tiki.stop()

        await asyncio.sleep(0.3)

        # ------------------------------
        # 좌우 거리 확인
        # ------------------------------

        left = (
            left_distance
            if left_distance is not None
            else 0
        )

        right = (
            right_distance
            if right_distance is not None
            else 0
        )

        print(
            "회피 방향 확인 → 왼쪽: %.1f cm / 오른쪽: %.1f cm"
            % (left, right)
        )

        # ------------------------------
        # 왼쪽이 더 넓으면 왼쪽 회전
        # ------------------------------

        if left > right:

            print("왼쪽으로 회전")

            tiki.counter_clockwise(TURN_SPEED)

            await asyncio.sleep(TURN_TIME)

            tiki.stop()

        # ------------------------------
        # 오른쪽이 더 넓으면 오른쪽 회전
        # ------------------------------

        else:

            print("오른쪽으로 회전")

            tiki.clockwise(TURN_SPEED)

            await asyncio.sleep(TURN_TIME)

            tiki.stop()

        # ------------------------------
        # 회전 후 잠시 대기
        # ------------------------------

        await asyncio.sleep(0.2)


# ------------------------------
# 메인 함수
# ------------------------------

async def main():

    print()
    print("==============================")
    print(" TIKI-mini 장애물 회피 로봇")
    print("==============================")
    print()

    tiki = None
    lidar = None

    try:

        # ------------------------------
        # TIKI-mini 연결
        # ------------------------------

        print("TIKI-mini 연결 중...")

        tiki = TikiMini()

        tiki.set_motor_mode(
            tiki.MOTOR_MODE_PID
        )

        tiki.stop()

        print("TIKI-mini 연결 완료")

        # ------------------------------
        # RPLIDAR 연결
        # ------------------------------

        print("RPLIDAR C1 연결 중...")

        lidar = RPLidar(
            LIDAR_PORT,
            LIDAR_BAUDRATE
        )

        print("RPLIDAR 상태 확인 중...")

        lidar.healthcheck()

        print("RPLIDAR 연결 완료")

        print()
        print("==============================")
        print(" 장애물 회피 시작")
        print("==============================")
        print()

        # ------------------------------
        # 두 작업을 동시에 실행
        # ------------------------------

        async with asyncio.TaskGroup() as tg:

            # RPLIDAR 데이터 읽기
            tg.create_task(
                lidar.simple_scan()
            )

            # RPLIDAR 데이터 처리
            tg.create_task(
                lidar_task(lidar)
            )

            # TIKI 주행 제어
            tg.create_task(
                robot_control(tiki)
            )

    except KeyboardInterrupt:

        print()
        print("사용자가 프로그램을 중지했습니다.")

    except Exception as e:

        print()
        print("오류 발생:")
        print(e)

    finally:

        print()
        print("로봇 정지 중...")

        # ------------------------------
        # 모터 정지
        # ------------------------------

        if tiki is not None:

            try:
                tiki.stop()
            except Exception:
                pass

        # ------------------------------
        # RPLIDAR 종료
        # ------------------------------

        if lidar is not None:

            try:
                lidar.stop_event.set()
            except Exception:
                pass

            try:
                lidar.reset()
            except Exception:
                pass

        print("모든 장치 정지 완료")


# ------------------------------
# 프로그램 시작
# ------------------------------

asyncio.run(main())