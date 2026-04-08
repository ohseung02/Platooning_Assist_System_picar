#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import traceback
import sys 
import tty
import termios

from threading import Thread, Event
from sensor_interface import SensorInterface
from moter_control import MotorController
from main_lane_follow import init_hardware, start_flask_server, run_lane_follow
from utils import *
from leader_action_server import ActionBroadcastServer
from obstacle_detecting import detect_obstacle
import main_lane_follow as lane  # 모드 전역 접근

print("\n===============================")
print(" Rule-based Policy Runner ")
print("===============================\n")

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# 리더의 action(상수) → 팔로워에게 보낼 인덱스(0~4) 매핑
ACTION_TO_INDEX = {
    LANE_LEFT: 0,
    IDLE: 1,
    LANE_RIGHT: 2,
    FASTER: 3,
    SLOWER: 4,
}

action_server = None  # finally에서 참조를 위해 미리 선언

try:
    # -------------------------------
    # 1️⃣ 하드웨어 초기화
    # -------------------------------
    print("[INIT] 하드웨어 초기화 중...")
    pca, drive_motor, steering_servo = init_hardware()
    sensor_if = SensorInterface()

    # ====== Platooning Action Server 실행 ======
    action_server = ActionBroadcastServer(port=6000)
    action_server.start()
    print("[Leader] Action broadcast server started")

    # -------------------------------
    # 2️⃣ 차선 주행 스레드 시작
    # -------------------------------
    stop_event = Event()    # 전체 종료용
    pause_event = Event()   # lane_follow 일시정지용

    lane_thread = Thread(
        target=run_lane_follow,
        args=(stop_event, pause_event, sensor_if),
        daemon=True
    )
    lane_thread.start()
    print("[LANE] 차선 주행 스레드 실행 중...")

    # Flask 스트리밍 서버 시작
    start_flask_server()

    # -------------------------------
    # 3️⃣ MotorController 생성
    # -------------------------------
    motor_ctl = MotorController(
        drive_motor=drive_motor,
        steering_servo=steering_servo,
        sensor_if=sensor_if,
        stop_event=stop_event,
        pause_event=pause_event,
    )

    # -------------------------------
    # 4️⃣ 초기 속도 설정
    # -------------------------------
    motor_ctl._set_throttle(BASE_SPEED)
    print(f"[INIT] 기본 속도 설정 완료 (throttle={BASE_SPEED})")
    print("\n=== 키 입력 대기 중 ===")
    print("z → 우회전")
    print("x → 좌측 차선 변경")
    print("c → 우측 차선 변경")
    print("0/1/2/3 → 모드 변경 (0=Normal, 1=Platoon, 2=Overtake+YOLO, 3=Leader)")
    print("b → 배터리 퍼센트 수동 설정 (b 후 숫자 입력)")
    print("q → 종료\n")

    # -------------------------------
    # 5️⃣ 메인 루프 (규칙 기반 + 모드/배터리/차선 변경)
    # -------------------------------
    while True:
        key = getkey()

        # 종료
        if key == "q":
            print("[EXIT] 사용자 종료 요청")
            break

        # 모드 변경 0/1/2/3  → lane.current_mode 변경
        elif key in ("0", "1", "2", "3"):
            lane.current_mode = int(key)
            name = getattr(lane, "label_names", {}).get(int(key), "Unknown")
            print(f"[MODE] {key} → {name}")
            # 여기서 모드 2로 바꾸면 main_lane_follow 안에서 YOLO 조건(current_mode==2) 만족

        # 배터리 수동 설정: b
        elif key == "b":
            try:
                # b를 누른 뒤 숫자를 입력(엔터로 확정)
                sys.stdout.write("\n[BATT] 배터리 퍼센트 입력 (0~100) > ")
                sys.stdout.flush()
                percent_str = sys.stdin.readline().strip()
                if not percent_str:
                    print("[BATT] 입력 취소")
                else:
                    val = float(percent_str)
                    lane.set_manual_battery(val)
                    print(f"[BATT] {val}% 로 수동 설정")
            except ValueError:
                print("[BATT] 숫자를 입력하세요 (예: 30, 40, 75)")
            except Exception as e:
                print("[BATT] 설정 중 오류:", e)

        # 우회전: z
        elif key == "z":
            print("[KEY] 우회전 (z) → MotorController.turn_right() 호출")
            motor_ctl.turn_right()

        # 우측 차선 변경: c
        elif key == "c":
            print("[KEY] 우측 차선 변경 실행 (c)")
            motor_ctl.change_lane("right")

        # 좌측 차선 변경: x
        elif key == "x":
            print("[KEY] 좌측 차선 변경 실행 (x)")
            motor_ctl.change_lane("left")

        # 기타 키는 무시 (원하면 디버그 출력 가능)
        else:
            # print(f"[DEBUG] 무시된 키: {repr(key)}")
            pass

except KeyboardInterrupt:
    print("\n[INTERRUPT] 사용자가 중단함.")

finally:
    # -------------------------------
    # 6️⃣ 안전 종료
    # -------------------------------
    print("[SHUTDOWN] 시스템 종료 중...")
    try:
        # platooning 서버 종료
        try:
            if action_server is not None:
                action_server.stop()
        except Exception:
            pass

        # motor_ctl가 생성 안 된 상태에서 예외가 날 수도 있으니 방어코드
        try:
            motor_ctl.stop("Program End")
        except Exception:
            pass

        try:
            stop_event.set()   # lane_follow 종료
        except Exception:
            pass

        try:
            pca.deinit()
        except Exception:
            pass

    except Exception:
        traceback.print_exc()

    print("[DONE] 모든 시스템 종료 완료 ✅")
