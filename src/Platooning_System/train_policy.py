import highway_env
import gymnasium as gym
import numpy as np
from stable_baselines3 import DQN
from gymnasium import ObservationWrapper, Wrapper
from highway_env.vehicle.kinematics import Vehicle
from tqdm import trange
import random

# -------- Custom Observation Wrapper ----------
class SensorLikeObservation(ObservationWrapper):
    """
    관측값을 실차 센서 형태로 단순화:
    [ego_speed, front_distance, obstacle_speed, lane_id]

    - front_distance = -1 → 앞차 없음
    - obstacle_speed = -1 → 앞차 없음
    """
    def __init__(self, env):
        super().__init__(env)
        from gymnasium.spaces import Box
        self.observation_space = Box(
            low=np.array([0.0, -1.0, -1.0, 0.0], dtype=np.float32),
            high=np.array([50.0, 100.0, 30.0, 1.0], dtype=np.float32)  
            # lane_id ∈ {0,1} (2차선이니까)
        )

    def observation(self, obs):
        ego = self.unwrapped.vehicle
        ego_speed = ego.speed
        lane_id = float(ego.lane_index[2])  # lane index = (road_id, section, lane_id)

        # 기본값: 앞차 없음
        front_distance = -1.0
        obstacle_speed = -1.0

        # ego 앞쪽 같은 차선 차량 탐색
        for v in self.unwrapped.road.vehicles:
            if v is ego:
                continue
            if v.lane_index == ego.lane_index and v.position[0] > ego.position[0]:
                dist = v.position[0] - ego.position[0]
                if front_distance == -1.0 or dist < front_distance:
                    front_distance = dist
                    obstacle_speed = v.speed

        return np.array([ego_speed, front_distance, obstacle_speed, lane_id], dtype=np.float32)


# -------- AlwaysObstacle Wrapper --------
class AlwaysObstacleWrapper(Wrapper):
    """
    항상 ego 앞에 정지 차량을 두고,
    ego가 지나치면 새로운 장애물을 계속 생성
    """
    def __init__(self, env, gap_range=(70.0, 100.0)):
        super().__init__(env)
        self.gap_range = gap_range

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        ego = self.env.unwrapped.vehicle
        self.env.unwrapped.road.vehicles = [ego]  # ego만 남기고
        self._spawn_obstacle()
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        ego = self.env.unwrapped.vehicle

        if isinstance(obs, np.ndarray) and obs.ndim == 2:
            obs = obs[0]
        ego_speed, front_distance, obstacle_speed, lane_id = obs

        reward = 0.0

        if terminated:
            reward -= 50.0

        if front_distance != -1:
            if front_distance < 8:
                reward -= 5.0
            elif 8 <= front_distance < 15:
                reward -= 2.0 
            elif 15 <= front_distance < 30:
                reward += 1.0
            elif front_distance >= 30:
                reward += 3.0

        if abs(lane_id - 0.5) > 0.4:
            reward += 1.5

        if ego_speed < 5:
            reward -= 0.2

        has_front = any(
            (v is not ego) and (v.lane_index == ego.lane_index) and (v.position[0] > ego.position[0])
            for v in self.env.unwrapped.road.vehicles
        )
        if not has_front:  # 앞 차가 없으면 새로 생성
            self._spawn_obstacle()
        return obs, reward, terminated, truncated, info

    def _spawn_obstacle(self):
        ego = self.env.unwrapped.vehicle
        gap = random.uniform(*self.gap_range)
        obstacle = Vehicle.make_on_lane(
            self.env.unwrapped.road,
            ego.lane_index,
            ego.position[0] + gap,
            speed=0.0   # 정지 차량
        )
        self.env.unwrapped.road.vehicles.append(obstacle)


# -------- 환경 설정 ---------
config = {
    "lanes_count": 2,
    "vehicles_count": 1,
    "controlled_vehicles": 1,
    "duration": 40,
    "spawn_probability": 0,
}

# 환경 생성
env = gym.make("highway-v0", config=config)
env = SensorLikeObservation(env)
env = AlwaysObstacleWrapper(env, gap_range=(70.0, 100.0))  # 항상 앞에 장애물

# -------- RL 모델 ---------
model = DQN(
    "MlpPolicy",
    env,
    verbose=1,   
    tensorboard_log="./dqn_highway_log/",
    learning_rate=5e-4,
    buffer_size=50_000,
    batch_size=32,
    gamma=0.99,
    train_freq=1,
    target_update_interval=500,
    exploration_initial_eps=1.0,
    exploration_final_eps=0.05,   
    exploration_fraction=0.25,     
)

# -------- 학습 ---------
print("Training start...")

model.learn(
    total_timesteps=100_000,
    log_interval=10
)

print("Training finished")

# ———— 모델 저장 ————
model.save("lanechange_dqn_sensor")
print("모델 저장 완료: lanechange_dqn_sensor.zip")

