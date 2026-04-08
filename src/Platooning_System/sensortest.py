#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from sensor_interface import SensorInterface
from main_lane_follow import init_hardware

print("\n===============================")
print("   Ultrasonic Sensor Test")
print("   (No Movement, Only Sensing)")
print("===============================\n")

try:
    # 1) 하드웨어 초기화 (모터, 서보 포함)
    pca, drive_motor, steering_servo = init_hardware()

    # 2) 차량 절대 움직이지 않도록 설정
    drive_motor.throttle = 0.0
    steering_servo.angle = 85  # 중앙값
    print("[INIT] Motor OFF, Steering Centered")

    # 3) 센서 인터페이스 초기화
    sensor = SensorInterface()
    print("[INIT] Sensor Interface Ready")

    print("\n--- 시작합니다. Ctrl+C 로 종료하세요. ---\n")

    while True:
        sensor.update_front_distance()
        dist = sensor.front_distance
        print(f"[DIST] {dist:.4f} m" if dist >= 0 else "[DIST] 장애물 없음 (-1)")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n종료합니다.")

finally:
    # 안전 종료
    try: drive_motor.throttle = 0.0
    except: pass
    try: steering_servo.angle = 85
    except: pass
    try: pca.deinit()
    except: pass

    print("[DONE] 안전하게 종료되었습니다.")
