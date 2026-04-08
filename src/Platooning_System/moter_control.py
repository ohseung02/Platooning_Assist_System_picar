import time
from utils import (
    BASE_SPEED, SPEED_STEP, MAX_SPEED, MIN_SPEED, STEERING_CENTER,
    LANE_LEFT, IDLE, LANE_RIGHT, FASTER, SLOWER
)

# 기본 차선 변경 파라미터 (utils에 있으면 그걸 쓰고, 아니면 fallback 사용)
from utils import (
        LANE_CHANGE_STEER,
        LANE_CHANGE_SPEED,
        LANE_CHANGE_DURATION,
        LANE_SETTLE_DURATION,
        LANE_CENTER_BIAS
    )

# ─────────────────────────────────
# 좌/우 차선 변경 각각 따로 튜닝용 파라미터
# 숫자는 대충 대칭으로 넣어놨으니, 직접 돌려보고 한쪽만 바꿔가면 됨
# ─────────────────────────────────

# 조향각 (센터에서 몇 도나 꺾을지)
LANE_CHANGE_STEER_LEFT  = LANE_CHANGE_STEER     # 필요하면 예: 38
LANE_CHANGE_STEER_RIGHT = LANE_CHANGE_STEER     # 필요하면 예: 32

# 차선 변경할 때 속도
LANE_CHANGE_SPEED_LEFT  = LANE_CHANGE_SPEED     # 필요하면 왼쪽만 0.75 등으로
LANE_CHANGE_SPEED_RIGHT = LANE_CHANGE_SPEED

# 각 단계 시간 (초 단위)
# 1단계: 목표 차선 쪽으로 파고들기
# 2단계: 반대로 확 틀어서 차체 방향 정면으로 맞추기
# 3단계: 센터로 두고 쭉
# Left -> 왼쪽으로 가는거
LANE_CHANGE_T1_LEFT  = 0.9
LANE_CHANGE_T2_LEFT  = 0.5
LANE_CHANGE_T3_LEFT  = 0.5

LANE_CHANGE_T1_RIGHT = 1.0
LANE_CHANGE_T2_RIGHT = 0.6
LANE_CHANGE_T3_RIGHT = 0.5  

# 반대쪽으로 틀 때 "얼마나 세게" 틀지 (1.0 ~ 1.3 정도 사이에서 튜닝)
LANE_CHANGE_OUT_GAIN_LEFT  = 1.2
LANE_CHANGE_OUT_GAIN_RIGHT = 1.15


class MotorController:
    def __init__(self, drive_motor, steering_servo, sensor_if=None,
                 stop_event=None, pause_event=None):
        """
        drive_motor: adafruit_motor.DCMotor
        steering_servo: adafruit_motor.servo.Servo
        sensor_if: SensorInterface 인스턴스 (선택)
        stop_event: 전체 프로그램 종료용
        pause_event: lane_follow 일시정지용
        """
        self.drive_motor = drive_motor
        self.steering_servo = steering_servo
        self.sensor_if = sensor_if
        self.stop_event = stop_event      # 진짜 종료용
        self.pause_event = pause_event    # 차선 변경 중 일시정지용

        self.current_speed = BASE_SPEED
        self.current_lane = 1
        self.lane_changing = False

    # -----------------------------
    # 내부 유틸
    # -----------------------------
    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _set_throttle(self, v):
        v = self._clamp(v, 0.0, 1.0)
        self.drive_motor.throttle = v
        if self.sensor_if:
            self.sensor_if.update_speed(v)

    def _set_servo_angle(self, angle):
        angle = self._clamp(angle, 0, 160)
        self.steering_servo.angle = angle

    def _steer_and_drive(self, servo_target, speed, duration, settle=0.3):
        """단순하게 한 번 꺾고 가는 동작에 쓸 수 있는 유틸 (현재 change_lane에서는 직접 제어로 사용 중)"""
        self._set_servo_angle(servo_target)
        self._set_throttle(self._clamp(speed, MIN_SPEED, MAX_SPEED))
        time.sleep(duration)

        # 중앙 복귀 및 안정화
        self._set_servo_angle(STEERING_CENTER)
        time.sleep(settle)

        # 원래 속도로 복귀
        self._set_throttle(self.current_speed)

    # -----------------------------
    # 속도 제어
    # -----------------------------
    def speed_up(self):
        self.current_speed = self._clamp(self.current_speed + SPEED_STEP, MIN_SPEED, MAX_SPEED)
        self._set_throttle(self.current_speed)
        print(f"[ACTION] 가속 → throttle={self.current_speed:.2f}")

    def slow_down(self):
        self.current_speed = self._clamp(self.current_speed - SPEED_STEP, MIN_SPEED, MAX_SPEED)
        self._set_throttle(self.current_speed)
        print(f"[ACTION] 감속 → throttle={self.current_speed:.2f}")

    def maintain(self):
        self._set_throttle(self.current_speed)
        print(f"[ACTION] 속도 유지 (throttle={self.current_speed:.2f})")


    def turn_right(self, steer: float = 45.0, speed: float = 0.7, duration: float = 2.0):
        """
        lane_follow 루프를 잠시 '일시정지'시키고
        S자 궤적으로 차선 변경을 수행한다.
        direction: 'left' 또는 'right'
        """
        if self.lane_changing:
            print("[INFO] 우회전 중")
            return

        # 1️⃣ lane_follow 일시정지
        if self.pause_event is not None:
            print("[INFO] lane_follow 일시정지")
            self.pause_event.set()
            time.sleep(0.2)

        self.lane_changing = True




        sign = -1
        steer = LANE_CHANGE_STEER_RIGHT
        speed = LANE_CHANGE_SPEED_RIGHT
        T1 = 2.0


        # 2️⃣ 조향 각도 설정
        #  1단계: 목표 차선 쪽으로 (우측이면 우측, 좌측이면 좌측)
        angle_in = STEERING_CENTER + sign * abs(steer)




        # 차선 변경 동안 일정 속도로 밀고 감
        self._set_throttle(self._clamp(speed, MIN_SPEED, MAX_SPEED))

        # ---- 1단계: 목표 차선 쪽으로 파고들기 ----
        self._set_servo_angle(angle_in)
        time.sleep(T1)


        # 속도 원래 값으로 복귀
        self._set_throttle(self.current_speed)


        self.lane_changing = False
        # lane_follow 재개
        if self.pause_event is not None:
            print("[INFO] lane_follow 재개")
            self.pause_event.clear()


    # -----------------------------
    # 차선 변경 (S-curve)
    # -----------------------------
    def change_lane(self, direction: str):
        """
        lane_follow 루프를 잠시 '일시정지'시키고
        S자 궤적으로 차선 변경을 수행한다.
        direction: 'left' 또는 'right'
        """
        if self.lane_changing:
            print("[INFO] 이미 차선 변경 중")
            return

        direction = (direction or "").lower()
        if direction not in ("left", "right"):
            print(f"[WARN] 잘못된 차선 변경 방향: {direction}")
            return

        print(f"[ACTION] 차선 변경 시작 → {direction}")

        # 1️⃣ lane_follow 일시정지
        if self.pause_event is not None:
            print("[INFO] lane_follow 일시정지")
            self.pause_event.set()
            time.sleep(0.2)

        self.lane_changing = True

        try:
            # 방향 부호: 오른쪽은 -1, 왼쪽은 +1
            if direction == "right":
                sign = -1
                steer = LANE_CHANGE_STEER_RIGHT
                speed = LANE_CHANGE_SPEED_RIGHT
                T1 = LANE_CHANGE_T1_RIGHT
                T2 = LANE_CHANGE_T2_RIGHT
                T3 = LANE_CHANGE_T3_RIGHT
                out_gain = LANE_CHANGE_OUT_GAIN_RIGHT
            else:
                sign = 1
                steer = LANE_CHANGE_STEER_LEFT
                speed = LANE_CHANGE_SPEED_LEFT
                T1 = LANE_CHANGE_T1_LEFT
                T2 = LANE_CHANGE_T2_LEFT
                T3 = LANE_CHANGE_T3_LEFT
                out_gain = LANE_CHANGE_OUT_GAIN_LEFT

            # 2️⃣ 조향 각도 설정
            #  1단계: 목표 차선 쪽으로 (우측이면 우측, 좌측이면 좌측)
            angle_in = STEERING_CENTER + sign * abs(steer)

            #  2단계: 차체를 정면으로 되돌리기 위해 반대쪽으로 "확" 트는 각도
            angle_out = STEERING_CENTER - sign * abs(steer) * out_gain

            print(
                f"[S-CURVE-{direction.upper()}] "
                f"angle_in={angle_in:.1f}, angle_out={angle_out:.1f}, "
                f"T1={T1:.2f}, T2={T2:.2f}, T3={T3:.2f}, speed={speed:.2f}"
            )

            # 차선 변경 동안 일정 속도로 밀고 감
            self._set_throttle(self._clamp(speed, MIN_SPEED, MAX_SPEED))

            # ---- 1단계: 목표 차선 쪽으로 파고들기 ----
            self._set_servo_angle(angle_in)
            time.sleep(T1)

            # ---- 2단계: 반대로 확 틀어서 차체 방향 정면으로 맞추기 ----
            self._set_servo_angle(angle_out)
            time.sleep(T2)

            # ---- 3단계: 바퀴를 다시 정중앙으로 두고 쭉 ----
            self._set_servo_angle(STEERING_CENTER)
            time.sleep(T3)

            # 속도 원래 값으로 복귀
            self._set_throttle(self.current_speed)

            # 차선 인덱스 업데이트 (원래 로직 유지)
            if direction == "right":
                self.current_lane = min(2, self.current_lane + 1)
            else:
                self.current_lane = max(1, self.current_lane - 1)

            if self.sensor_if:
                self.sensor_if.lane_id = float(self.current_lane)

            print(f"[INFO] 차선 변경 완료 → 현재 차선 {self.current_lane}")

        finally:
            self.lane_changing = False
            # lane_follow 재개
            if self.pause_event is not None:
                print("[INFO] lane_follow 재개")
                self.pause_event.clear()

    # -----------------------------
    # 정책 액션 처리
    # -----------------------------
    def apply_action(self, action):
        """
        정책에서 나온 액션을 실제 모터/서보 제어로 변환
        """
        if self.stop_event is not None and self.stop_event.is_set():
            self.stop("stop_event set")
            return

        # 센서에서 최신 전방 거리 읽기 (선택)
        front_dist = None
        if self.sensor_if is not None:
            front_dist = self.sensor_if.front_distance

        if action == LANE_LEFT:
            # 이미 최좌측면 무시
            if self.current_lane <= 1:
                print("[INFO] 이미 최좌측 차선, LANE_LEFT 무시")
                self.maintain()
                return

            self.change_lane("left")

        elif action == LANE_RIGHT:
            # 이미 최우측면 무시
            if self.current_lane >= 2:
                print("[INFO] 이미 최우측 차선, LANE_RIGHT 무시")
                self.maintain()
                return

            # 장애물 없으면 차선 변경 굳이 안 함 (옵션)
            if front_dist is not None and front_dist < 0:
                print("[INFO] 장애물 없음(front_dist<0), LANE_RIGHT 무시")
                self.maintain()
                return

            self.change_lane("right")

        elif action == FASTER:
            self.speed_up()
        elif action == SLOWER:
            self.slow_down()
        elif action == IDLE:
            self.maintain()
        else:
            print(f"[WARN] 알 수 없는 액션: {action}")

    # -----------------------------
    # 정지
    # -----------------------------
    def stop(self, reason=""):
        self._set_throttle(0.0)
        self._set_servo_angle(STEERING_CENTER)
        if reason:
            print(f"[STOP] {reason}")
        else:
            print("[STOP]")
