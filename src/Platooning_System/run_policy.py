#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import traceback
from threading import Thread, Event
from stable_baselines3 import DQN

from sensor_interface import SensorInterface
from moter_control import MotorController
from main_lane_follow import init_hardware, start_flask_server, run_lane_follow
from utils import *

print("\n===============================")
print(" RL Policy Runner (PURE RL) ")
print("===============================\n")

# 강화학습 모델 위치
MODEL_PATH = "/home/urop2/Desktop/models/lanechange_dqn_sensor.zip"


try:
    # -------------------------------
    # 1) 하드웨어 초기화
    # -------------------------------
    print("[INIT] 하드웨어 초기화 중...")
    pca, drive_motor, steering_servo = init_hardware()
    sensor_if = SensorInterface()

    # -------------------------------
    # 2) 차선 주행 스레드 시작
    # -------------------------------
    stop_event = Event()
    pause_event = Event()

    lane_thread = Thread(
        target=run_lane_follow,
        args=(stop_event, pause_event),
        daemon=True,
    )
    lane_thread.start()
    print("[LANE] 차선 주행 스레드 실행 중...")

    # Flask 서버
    start_flask_server()

    # -------------------------------
    # 3) MotorController 생성
    # -------------------------------
    motor_ctl = MotorController(
        drive_motor=drive_motor,
        steering_servo=steering_servo,
        sensor_if=sensor_if,
        stop_event=stop_event,
        pause_event=pause_event,
    )

    # -------------------------------
    # 4) RL 정책 로드
    # -------------------------------
    print(f"[LOAD] Loading RL policy: {MODEL_PATH}")
    policy = DQN.load(
        MODEL_PATH,
        custom_objects={
            "lr_schedule": lambda _: 0.0001,
            "exploration_schedule": lambda _: 0.0,
            "optimizer": None,
        },
        print_system_info=True
    )
    print("[LOAD] RL policy 로드 완료 ✅")

    # -------------------------------
    # 5) 초기 속도 설정
    # -------------------------------
    motor_ctl._set_throttle(BASE_SPEED)
    print(f"[INIT] 기본 속도 설정 완료 (throttle={BASE_SPEED})")

    # -------------------------------
    # 6) 메인 루프 (PURE RL)
    # -------------------------------
    while True:
        try:
            obs = sensor_if.get_state()
            print(f"[DEBUG] front_dist_raw={sensor_if.front_distance}")
            if obs is None:
                print("[WARN] 센서 데이터 없음. 차량 정지.")
                motor_ctl.stop("Sensor Error")
                time.sleep(0.3)
                continue

            # RL 정책 예측
            a_pred, _ = policy.predict(obs, deterministic=True)
            a_pred = int(a_pred)

            # DQN → 실제 제어 action 매핑
            remap = {
                0: LANE_LEFT,
                1: IDLE,
                2: LANE_RIGHT,
                3: FASTER,
                4: SLOWER,
            }
            action = remap.get(a_pred, IDLE)

            # 액션 적용
            motor_ctl.apply_action(action)

            # 로그 출력
            print(f"[STEP] obs={obs}, a_pred={a_pred}, action={action}")

            time.sleep(0.2)

        except KeyboardInterrupt:
            raise
        except Exception as e:
            print("[ERROR] 루프 중 예외 발생:", e)
            traceback.print_exc()
            motor_ctl.stop("Exception 발생 - 안전 정지")
            time.sleep(1.0)

except KeyboardInterrupt:
    print("\n[INTERRUPT] 사용자가 중단함.")

finally:
    # -------------------------------
    # 7) 안전 종료
    # -------------------------------
    print("[SHUTDOWN] 시스템 종료 중...")
    try:
        try: motor_ctl.stop("Program End")
        except: pass

        try: stop_event.set()
        except: pass

        try: pca.deinit()
        except: pass

    except Exception:
        traceback.print_exc()

    print("[DONE] 모든 시스템 종료 완료 ✅")
