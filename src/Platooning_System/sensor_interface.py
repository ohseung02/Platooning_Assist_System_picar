import time
from gpiozero import DistanceSensor
from Kalman_filter import Kalman_filter
from utils import BASE_SPEED, SPEED_CONVERT, DISTANCE_SCALE, CALIBRATION_SCALE

class SensorInterface:
    """
    강화학습 정책 입력 4개를 제공하는 클래스.
    - ego_speed (자차 속도)
    - front_vehicle_speed (항상 0)
    - front_distance (초음파)
    - lane_id (현재 차선)
    """

    def __init__(self):
        # 기본 상태값
        self.ego_speed_throttle = BASE_SPEED
        self.lane_id = 1
        self.front_vehicle_speed = 0.0
        self.front_distance = None

        # 초음파 센서 초기화
        self.sensor = DistanceSensor(echo=24, trigger=23, max_distance=2.0) # maa distance = 200cm
        self.kf = Kalman_filter(Q=0.3, R=2)
        self.CALIBRATION_SCALE = CALIBRATION_SCALE

        self.last_frame = None

    def measure_distance(self):
        """
        초음파 센서로 거리(cm) 측정 후 Kalman 필터 및 보정 적용
        """
        try:
            raw_cm = self.sensor.distance * 100.0
            filtered_cm = self.kf.kalman(raw_cm)
            calibrated_cm = raw_cm * self.CALIBRATION_SCALE
            return calibrated_cm

        except Exception as e:
            print("[Sensor] Distance read error:", e)
            return None

    def scale_distance(self, dist):
        # (1) -1 또는 None → 장애물 없음 그대로 반환
        if dist <= 0:
            return -1.0
    
        if dist < 0.8:
            return dist ** 2.5          # ★ 핵심: 강한 non-linear compression

        else:
            return (dist ** 2) * 1.3     # ★ 멀리 있는 값은 확 키우기

    def update_front_distance(self):
        """
        실시간 거리 측정값 업데이트 (m 단위 저장)
        - 센서 값이 너무 멀면: 장애물 없음 → -1.0
        - 센서 에러: 매우 근접(기존 fallback 0.1m 유지)
        """
        dist_cm = self.measure_distance()

        if dist_cm is None:
            # fallback: 매우 가까운 장애물 가정
            self.front_distance_raw = 0.1
            self.front_distance = self.scale_distance(0.1)
            return
        
        raw_m = dist_cm * DISTANCE_SCALE
        self.front_distance_raw = raw_m

        # 2) scaling 적용
        scaled = self.scale_distance(raw_m)
        self.front_distance = scaled

    def update_lane(self, action):
        """
        RL 행동에 따라 lane_id 변경
        """
        if action == "LEFT":
            self.lane_id = max(1, self.lane_id - 1)
        elif action == "RIGHT":
            self.lane_id = min(2, self.lane_id + 1)


    def update_speed(self, throttle_value: float):
        """
        모터 throttle 값 갱신 (0~1)
        """
        self.ego_speed_throttle = max(0.0, min(1.0, throttle_value))

    def get_state(self):
        """
        강화학습 정책 입력값 반환
        [ego_speed, front_vehicle_speed, front_distance, lane_id]
        """
        self.update_front_distance()

        ego_speed_ms = (self.ego_speed_throttle * SPEED_CONVERT) / 3.6
        front_distance = self.front_distance if self.front_distance is not None else -1.0
                       
        obstacle_speed = self.front_vehicle_speed if self.front_vehicle_speed is not None else -1.0
        lane_id = 0.0 if self.lane_id == 1 else 1.0

        ego_speed_ms = round(ego_speed_ms, 4)
        front_distance = round(front_distance, 4)

        return [float(ego_speed_ms), float(front_distance), float(obstacle_speed), float(lane_id)]