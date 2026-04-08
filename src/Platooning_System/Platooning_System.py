#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, cv2, math, threading, traceback
import numpy as np
from picamera2 import Picamera2
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo
from flask import Flask, Response, make_response, jsonify, request
from processor import ImageProcessor   # 깃허브에서 가져온 모듈

# YOLOv5
import torch

# ADS7830 배터리
from adafruit_bus_device.i2c_device import I2CDevice

from utils import (
    BASE_SPEED,
    MAX_STEER,
    KP,
    STEERING_CENTER,
    STEERING_KP,
    LANE_WIDTH_PX,
    LANE_CENTER_BIAS,
    CAMERA_CENTER_OFFSET,
)

# --------------------------------
# 모드 관련 (0~3)
# --------------------------------
current_mode = 0  # 0: Normal, 1: Platoon, 2: Overtake(YOLO), 3: Leader Change
label_names = {
    0: "Normal Mode",
    1: "Platoon Mode",
    2: "Overtake Mode",
    3: "Leader Change Mode",
}

# --------------------------------
# 전역 객체 (초기화 후 할당)
# --------------------------------
i2c = None
pca = None
drive_motor = None
steering_servo = None
picam2 = None
detector = None

# Flask 전역
app = Flask(__name__)
outputFrame = None
lock = threading.Lock()
flask_started = False

# --------------------------------
# 배터리(ADS7830) 설정
# --------------------------------
ads_device = None

Vref = 8.4
WarningThreshold = 6.75
R15 = 3000
R17 = 1000
DivisionRatio = R17 / (R15 + R17)

cmd = 0x84
channel = 0
control_byte = cmd | (((channel << 2 | channel >> 1) & 0x07) << 4)

battery_percent = 0.0
battery_voltage = 0.0
battery_adc = 0
battery_error = ""
manual_battery = False  # True면 수동값 유지

def read_battery_once():
    """ADS7830에서 한 번 읽어서 (전압, 퍼센트, ADC) 반환."""
    global ads_device, control_byte

    if ads_device is None:
        return 0.0, 0.0, 0

    buf = bytearray(1)
    ads_device.write_then_readinto(bytes([control_byte]), buf)
    adc_value = buf[0]

    a0_voltage = (adc_value / 255.0) * 5.0
    actual_v = a0_voltage / DivisionRatio

    percent = (actual_v - WarningThreshold) / (Vref - WarningThreshold) * 100.0
    percent = max(0.0, min(100.0, percent))

    return actual_v, percent, adc_value

def set_manual_battery(percent_value: float):
    """배터리 퍼센트 수동 지정 (b70 이런 식으로)."""
    global manual_battery, battery_percent, battery_voltage, battery_adc, battery_error

    manual_battery = True
    p = max(0.0, min(100.0, float(percent_value)))
    battery_percent = round(p, 1)
    v = WarningThreshold + (Vref - WarningThreshold) * (battery_percent / 100.0)
    battery_voltage = round(v, 2)
    battery_adc = -1
    battery_error = "manual override"
    print(f"[BATT] manual set: {battery_percent}% ({battery_voltage}V)")

def battery_thread():
    """1초마다 배터리 자동 측정."""
    global battery_percent, battery_voltage, battery_adc, battery_error, manual_battery

    print("[BATT] battery_thread 시작")
    while True:
        if manual_battery:
            time.sleep(1.0)
            continue

        try:
            v, p, adc = read_battery_once()
            battery_voltage = round(v, 2)
            battery_percent = round(p, 1)
            battery_adc = int(adc)
            battery_error = ""
        except Exception as e:
            battery_voltage = 0.0
            battery_percent = 0.0
            battery_adc = -1
            battery_error = str(e)
            print("[BATT] Exception:", e)

        time.sleep(1.0)



# --------------------------------
# 하드웨어 초기화
# --------------------------------
def init_hardware():
    global i2c, pca, drive_motor, steering_servo, picam2, detector, ads_device

    print("[INIT] 하드웨어 초기화 중...")

    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=0x5f)
    pca.frequency = 50

    # 뒷바퀴 모터
    drive_motor = motor.DCMotor(pca.channels[15], pca.channels[14])
    drive_motor.decay_mode = motor.SLOW_DECAY
    drive_motor.throttle = 0

    # 앞바퀴 서보
    steering_servo = servo.Servo(
        pca.channels[0],
        min_pulse=500,
        max_pulse=2400,
        actuation_range=160
    )
    steering_servo.angle = STEERING_CENTER

    # 카메라
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
    picam2.start()
    time.sleep(0.3)

    # 차선 검출기
    detector = ImageProcessor((640, 480), 20)

    # 배터리 ADC
    try:
        ads_device = I2CDevice(i2c, 0x48)
        print("[INIT] ADS7830 연결 완료")
    except Exception as e:
        ads_device = None
        print("[INIT] ADS7830 초기화 실패:", e)

    print("[INIT] 완료!")
    return pca, drive_motor, steering_servo

# --------------------------------
# 제어 함수
# --------------------------------
def simple_control(angle, center=90, Kp=KP):
    error = angle - center
    return float(np.clip(Kp * error, -MAX_STEER, MAX_STEER))

def drive(steer_control, speed=None):
    """lane_follow에서 사용하는 기본 조향/속도 함수."""
    servo_angle = float(np.clip(STEERING_CENTER - steer_control, 0, 160))
    steering_servo.angle = servo_angle

    # speed가 주어졌을 때만 스로틀 변경
    if speed is not None:
        drive_motor.throttle = speed
        current_speed = speed
    else:
        # 현재 스로틀 값을 그냥 로그에만 사용
        current_speed = drive_motor.throttle if drive_motor.throttle is not None else 0.0

    # 디버그 출력만
    print(f"[DRIVE] Steer={servo_angle:.1f}, Speed={current_speed:.2f}")

def full_stop(reason=""):
    drive_motor.throttle = 0
    steering_servo.angle = STEERING_CENTER
    if reason:
        print(f"[STOP] {reason}")
    else:
        print("[STOP]")

# --------------------------------
# Flask 스트리밍
# --------------------------------
def generate():
    global outputFrame
    while True:
        try:
            with lock:
                if outputFrame is not None:
                    ok, jpg = cv2.imencode(".jpg", outputFrame)
                    if ok:
                        yield (b"--frame\r\n"
                               b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
            time.sleep(0.02)
        except GeneratorExit:
            break
        except Exception as e:
            print("[Flask gen] Exception:", e)
            traceback.print_exc()
            time.sleep(0.1)

@app.after_request
def add_no_cache_headers(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp

@app.route("/video")
def video():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ---- 배터리 API ----
@app.route("/api/battery")
def api_battery():
    return jsonify(
        {
            "percent": battery_percent,
            "voltage": battery_voltage,
            "adc": battery_adc,
            "error": battery_error,
            "manual": manual_battery,
        }
    )

# --------------------------------
# 메인 페이지 (UI)
# --------------------------------
HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>PiCar Realtime Mode / Battery</title>
<style>
    body {
        font-family: Arial, sans-serif;
        background: #111;
        color: #eee;
        margin: 0;
        padding: 0;
    }
    .container {
        max-width: 1000px;
        margin: 20px auto;
        background: #1e1e1e;
        border-radius: 10px;
        padding: 20px;
        box-shadow: 0 0 20px rgba(0,0,0,0.7);
    }
    h2 { margin-top: 0; }
    .video-wrap {
        text-align: center;
        margin-bottom: 15px;
    }
    .video-wrap img {
        max-width: 100%;
        border-radius: 8px;
        border: 2px solid #333;
    }
    .info {
        display: flex;
        justify-content: space-between;
        flex-wrap: wrap;
        gap: 10px;
    }
    .card {
        flex: 1;
        min-width: 260px;
        background: #252525;
        border-radius: 8px;
        padding: 10px 15px;
    }
    .badge {
        display: inline-block;
        padding: 3px 8px;
        border-radius: 999px;
        font-size: 12px;
        margin-left: 5px;
    }
    .badge-normal  { background: #2e7d32; }
    .badge-platoon { background: #1565c0; }
    .badge-overtake{ background: #ef6c00; }
    .badge-leader  { background: #8e24aa; }
    .battery-bar {
        width: 100%;
        height: 20px;
        background: #444;
        border-radius: 999px;
        overflow: hidden;
        margin-top: 5px;
    }
    .battery-fill {
        height: 100%;
        width: 0%;
        background: #4caf50;
        transition: width 0.3s ease;
    }
    .battery-fill.low  { background: #f44336; }
    .battery-fill.warn { background: #ff9800; }
    .warning-text {
        color: #f44336;
        font-weight: bold;
        margin-top: 5px;
    }
    small { color: #aaa; }
    .mode-chip {
        display:inline-block;
        padding:2px 8px;
        border-radius:999px;
        background:#444;
        font-size:12px;
        margin-left:5px;
    }
</style>
</head>
<body>
<div class="container">
    <h2>PiCar Realtime Lane / Mode / Battery</h2>
    <div class="video-wrap">
        <img src="/video" />
    </div>
    <div class="info">
        <div class="card">
            <h3>Driving Mode</h3>
            <p>
                <b>0</b> - Normal Mode <span class="badge badge-normal">Normal</span><br>
                <b>1</b> - Platoon Mode <span class="badge badge-platoon">Platoon</span><br>
                <b>2</b> - Overtake Mode <span class="badge badge-overtake">Overtake + YOLOv5</span><br>
                <b>3</b> - Leader Change Mode <span class="badge badge-leader">Leader</span><br>
            </p>
        </div>
        <div class="card">
            <h3>Battery Status <span id="mode-chip" class="mode-chip">AUTO</span></h3>
            <p>
                <b id="batt-percent">--.-%</b>
                (<span id="batt-voltage">0.00</span> V)
            </p>
            <div class="battery-bar">
                <div id="batt-fill" class="battery-fill"></div>
            </div>
            <p id="batt-warning" class="warning-text"></p>
        </div>
    </div>
</div>

<script>
// 배터리 UI
async function updateBattery() {
    try {
        const res = await fetch('/api/battery');
        const data = await res.json();

        const percent = data.percent || 0.0;
        const voltage = data.voltage || 0.0;
        const manual = data.manual || false;

        const percentEl = document.getElementById('batt-percent');
        const voltageEl = document.getElementById('batt-voltage');
        const fillEl = document.getElementById('batt-fill');
        const warnEl = document.getElementById('batt-warning');
        const modeChip = document.getElementById('mode-chip');

        percentEl.textContent = percent.toFixed(1) + '%';
        voltageEl.textContent = voltage.toFixed(2);

        fillEl.style.width = percent + '%';
        fillEl.classList.remove('low', 'warn');
        warnEl.textContent = '';

        if (percent <= 40) {
            fillEl.classList.add('low');
            warnEl.textContent = '⚠ 배터리 부족! (40% 이하)';
        } else if (percent <= 50) {
            fillEl.classList.add('warn');
            warnEl.textContent = '주의: 배터리 잔량이 50% 이하입니다.';
        }

        modeChip.textContent = manual ? 'MANUAL' : 'AUTO';
    } catch (e) {
        console.log('배터리 정보 갱신 실패:', e);
    }
}

// 1초마다 배터리 갱신
updateBattery();
setInterval(updateBattery, 1000);
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return make_response(HTML)

# --------------------------------
# Flask + 배터리 스레드 시작
# --------------------------------
def start_flask_server():
    global flask_started
    if flask_started:
        print("[FLASK] 이미 실행 중, 재시작하지 않음.")
        return
    flask_started = True

    # 배터리 모니터링 스레드
    tb = threading.Thread(target=battery_thread, daemon=True)
    tb.start()

    # Flask 서버
    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True
    ).start()
    print("[FLASK] 스트리밍 서버 실행 중...")

# --------------------------------
# 메인 주행 루프 (lane_follow + YOLO + 오버레이)
# --------------------------------
def run_lane_follow(stop_event, pause_event=None, sensor_if=None):
    """외부에서 stop_event로 전체 종료, pause_event로 일시정지 가능"""
    global outputFrame, current_mode

    prev_lane_center = None

    try:
        while not stop_event.is_set():
            # 🔹 차선 변경 중이면 lane_follow를 잠깐 멈추고 직진/정지 유지
            if pause_event is not None and pause_event.is_set():
                time.sleep(0.02)
                continue

            rgb = picam2.capture_array("main")
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            vis = detector.process(frame)

            if sensor_if is not None:
                sensor_if.last_frame = frame

            left_ok = detector.left.poly is not None
            right_ok = detector.right.poly is not None

            if not (left_ok or right_ok):
                # 차선을 못 잡아도 멈추지 말고 직전 조향 유지
                cv2.putText(vis, "NO LANE - KEEP GOING",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 255), 2)

                with lock:
                    outputFrame = vis
                time.sleep(0.02)
                continue

            h, w = detector.h, detector.w
            frame_center = w / 2 + CAMERA_CENTER_OFFSET
            lookahead_y = int(h * 0.6)

            if left_ok:
                left_x = detector.left.poly(lookahead_y)
            if right_ok:
                right_x = detector.right.poly(lookahead_y)

            if left_ok and right_ok:
                # 오른쪽 차선이 정상적인 위치에 있는지 검사
                expected_min = left_x + LANE_WIDTH_PX * 0.7
                expected_max = left_x + LANE_WIDTH_PX * 1.3

                if not (expected_min < right_x < expected_max):
                    # 옆 차선을 감지한 것으로 판단 → 제거
                    right_ok = False
                    cv2.putText(vis, "RIGHT LANE REJECTED", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if left_ok and right_ok:
                lane_center = (left_x + right_x) / 2 + LANE_CENTER_BIAS
            elif left_ok:
                lane_center = detector.left.poly(lookahead_y) + (LANE_WIDTH_PX / 2)
            else:
                lane_center = detector.right.poly(lookahead_y) - (LANE_WIDTH_PX / 2)

            if prev_lane_center is None:
                smoothed_lane_center = lane_center
            else:
                max_jump = 0.8 * LANE_WIDTH_PX
                if abs(lane_center - prev_lane_center) > max_jump:
                    smoothed_lane_center = prev_lane_center
                else:
                    alpha = 0.2
                    smoothed_lane_center = (1 - alpha) * prev_lane_center + alpha * lane_center

            prev_lane_center = smoothed_lane_center

            x_offset = lane_center - frame_center           # 노란점 - 화면 중앙 (픽셀)
            x_norm = x_offset / (w / 2.0)                   # -1.0 ~ +1.0 로 정규화
            steer_control = float(np.clip(
                x_norm * MAX_STEER * STEERING_KP,           # 화면 끝 → MAX_STEER 로 매핑
                -MAX_STEER, MAX_STEER
            ))

            drive(steer_control, None)

            # ---- 차선/중앙점 오버레이 ----
            if left_ok:
                detector.drawPoly(vis, detector.left.poly, (0, 255, 0))
            if right_ok:
                detector.drawPoly(vis, detector.right.poly, (0, 255, 0))
            cv2.circle(vis, (int(lane_center), int(lookahead_y)), 5, (0, 255, 255), -1)
            cv2.putText(vis, "LANE OK", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (80, 255, 80), 2)




            # ---- 상단 모드/배터리 오버레이 ----
            overlay = vis.copy()
            cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
            vis = cv2.addWeighted(overlay, 0.4, vis, 0.6, 0)

            mode_name = label_names.get(current_mode, "Unknown")
            cv2.putText(
                vis,
                f"Mode: {mode_name} ({current_mode})",
                (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if battery_percent <= 40:
                batt_color = (0, 0, 255)
            elif battery_percent <= 50:
                batt_color = (0, 255, 255)
            else:
                batt_color = (0, 255, 0)

            batt_text = f"Batt: {battery_percent:.1f}% ({battery_voltage:.2f}V)"
            cv2.putText(
                vis,
                batt_text,
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                batt_color,
                2,
                cv2.LINE_AA,
            )

            # 배터리 부족 경고 (영상 중앙)
            if battery_percent <= 40:
                warning_msg = "BATTERY LOW! (<40%)"
                (tw, th), _ = cv2.getTextSize(
                    warning_msg, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
                )
                cx = (vis.shape[1] - tw) // 2
                cy = vis.shape[0] // 2
                overlay2 = vis.copy()
                cv2.rectangle(
                    overlay2,
                    (cx - 10, cy - th - 10),
                    (cx + tw + 10, cy + 10),
                    (0, 0, 255),
                    -1,
                )
                vis = cv2.addWeighted(overlay2, 0.5, vis, 0.5, 0)
                cv2.putText(
                    vis,
                    warning_msg,
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            with lock:
                outputFrame = vis

            time.sleep(0.02)

    except Exception as e:
        print("[LaneFollow] Exception:", e)
        traceback.print_exc()
    finally:
        full_stop("Lane follow stopped")
        print("[LaneFollow] 종료됨")

# 단독 실행용 (옵션)
if __name__ == "__main__":
    try:
        init_hardware()
        start_flask_server()

        print("[MAIN] main_lane_follow 단독 실행 중. Ctrl+C 로 종료.")
        stop_evt = threading.Event()
        run_lane_follow(stop_evt)
    except KeyboardInterrupt:
        full_stop("KeyboardInterrupt")
