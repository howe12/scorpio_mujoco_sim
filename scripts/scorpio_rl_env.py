#!/usr/bin/env python3
"""
Scorpio Robot RL Environment (Gymnasium-compatible)

A MuJoCo-based reinforcement learning environment for the Scorpio robot.
Supports navigation, obstacle avoidance, and locomotion policy training.

Usage:
    # With gymnasium
    import gymnasium as gym
    env = gym.make('scorpio-nav-v0')  # after registration
    
    # Direct usage
    env = ScorpioNavEnv()
    obs, info = env.reset()
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
"""

import numpy as np
import mujoco
import os
from typing import Optional, Dict, Tuple


class ScorpioBaseEnv:
    """Base MuJoCo environment for Scorpio robot."""

    WHEEL_RADIUS = 0.0525
    WHEEL_SEPARATION = 0.1856
    MAX_LINEAR_VEL = 0.26
    MAX_ANGULAR_VEL = 1.0

    def __init__(self, model_path: Optional[str] = None, render_mode: str = "human"):
        if model_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(script_dir, '..', 'models', 'scorpio.xml')
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        
        # Sensor IDs
        self._base_pos_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_pos')
        self._base_quat_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_quat')
        self._base_linvel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_linvel')
        self._base_angvel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_angvel')
        self._imu_accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_accel')
        self._imu_gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_gyro')
        
        # LiDAR site
        self._lidar_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_site')
        
        # Action space: [linear_vel, angular_vel] normalized to [-1, 1]
        self.action_dim = 2
        
        # Observation dimension (set by subclass)
        self._observation_dim = 0
        
        # Viewer
        self._viewer = None

    @property
    def observation_dim(self):
        return self._observation_dim

    def _get_obs(self) -> np.ndarray:
        raise NotImplementedError

    def _compute_reward(self) -> float:
        raise NotImplementedError

    def _check_terminated(self) -> bool:
        raise NotImplementedError

    def reset(self, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment."""
        if seed is not None:
            np.random.seed(seed)
        
        mujoco.mj_resetData(self.model, self.data)
        
        # Random initial position (small perturbation)
        self.data.qpos[0] = np.random.uniform(-0.5, 0.5)
        self.data.qpos[1] = np.random.uniform(-0.5, 0.5)
        
        # Step once to settle
        mujoco.mj_forward(self.model, self.data)
        
        return self._get_obs(), {}

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Step environment with action [linear_vel_norm, angular_vel_norm]."""
        # Denormalize action
        linear = np.clip(action[0], -1.0, 1.0) * self.MAX_LINEAR_VEL
        angular = np.clip(action[1], -1.0, 1.0) * self.MAX_ANGULAR_VEL
        
        # Convert to wheel velocities
        left_vel = (linear + angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        right_vel = (linear - angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        
        self.data.ctrl[0] = left_vel
        self.data.ctrl[1] = left_vel
        self.data.ctrl[2] = right_vel
        self.data.ctrl[3] = right_vel
        
        # Step physics (multiple sub-steps for stability)
        n_substeps = 4
        for _ in range(n_substeps):
            mujoco.mj_step(self.model, self.data)
        
        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = self._check_terminated()
        truncated = False
        
        info = {
            'position': self.data.qpos[:3].copy(),
            'velocity': self._read_sensor(self._base_linvel_id, 3),
            'time': self.data.time,
        }
        
        return obs, reward, terminated, truncated, info

    def _read_sensor(self, sensor_id: int, dim: int) -> np.ndarray:
        """Read sensor data by ID."""
        addr = self.model.sensor_adr[sensor_id]
        return self.data.sensordata[addr:addr+dim].copy()

    def _render_lidar(self, num_beams: int = 64) -> np.ndarray:
        """Render simplified LiDAR for RL observation."""
        lidar_pos = self.data.site_xpos[self._lidar_site_id].copy()
        lidar_mat = self.data.site_xmat[self._lidar_site_id].reshape(3, 3)
        
        ranges = np.zeros(num_beams, dtype=np.float32)
        angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)
        
        for i, angle in enumerate(angles):
            direction_local = np.array([np.cos(angle), np.sin(angle), 0.0])
            direction_world = lidar_mat @ direction_local
            
            geom_id = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(
                self.model, self.data,
                lidar_pos, direction_world,
                None, 1, -1, geom_id
            )
            
            ranges[i] = max(0.12, min(dist, 12.0)) if dist >= 0 else 12.0
        
        return ranges

    def render(self):
        """Render the scene."""
        if self.render_mode == "human":
            if self._viewer is None:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(
                    self.model, self.data, show_left_ui=False, show_right_ui=False
                )
            self._viewer.sync()

    def close(self):
        """Clean up resources."""
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


class ScorpioNavEnv(ScorpioBaseEnv):
    """Navigation task: reach a target position while avoiding obstacles."""

    def __init__(self, model_path=None, render_mode="human", target_pos=None):
        super().__init__(model_path, render_mode)
        self.target_pos = target_pos or np.array([2.0, 0.0, 0.0])
        self._observation_dim = 2 + 3 + 6 + 64  # vel + pos_err + imu(accel+gyro) + lidar

    def _get_obs(self) -> np.ndarray:
        # Velocity (2D)
        linvel = self._read_sensor(self._base_linvel_id, 3)
        angvel = self._read_sensor(self._base_angvel_id, 3)
        vel_obs = np.array([linvel[0], angvel[2]])
        
        # Position error to target (in body frame)
        pos = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        pos_err = self.target_pos - pos
        
        # Rotate to body frame
        w, x, y, z = quat
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        pos_err_body = R.T @ pos_err
        
        # IMU (accel + gyro, 6D)
        accel = self._read_sensor(self._imu_accel_id, 3)
        gyro = self._read_sensor(self._imu_gyro_id, 3)
        imu_obs = np.concatenate([accel, gyro])
        
        # LiDAR (64 beams)
        lidar = self._render_lidar(64)
        lidar_normalized = lidar / 12.0  # Normalize to [0, 1]
        
        return np.concatenate([vel_obs, pos_err_body, imu_obs, lidar_normalized]).astype(np.float32)

    def _compute_reward(self) -> float:
        pos = self.data.qpos[:3]
        dist_to_target = np.linalg.norm(pos[:2] - self.target_pos[:2])
        
        # Reward components
        progress_reward = -dist_to_target  # Closer is better
        forward_reward = self._read_sensor(self._base_linvel_id, 3)[0] * 0.5  # Encourage forward motion
        collision_penalty = 0.0
        
        # Check for collisions (contact forces)
        if self.data.ncon > 0:
            for i in range(self.data.ncon):
                contact = self.data.contact[i]
                # Penalize body contacts (not wheel-ground)
                body1 = self.model.geom_bodyid[contact.geom1]
                body2 = self.model.geom_bodyid[contact.geom2]
                body_name1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1)
                body_name2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2)
                if body_name1 and 'wheel' not in body_name1 and 'ground' not in (body_name2 or ''):
                    collision_penalty -= 1.0
                elif body_name2 and 'wheel' not in body_name2 and 'ground' not in (body_name1 or ''):
                    collision_penalty -= 1.0
        
        # Goal reached bonus
        goal_bonus = 10.0 if dist_to_target < 0.3 else 0.0
        
        return progress_reward * 0.1 + forward_reward + collision_penalty + goal_bonus

    def _check_terminated(self) -> bool:
        pos = self.data.qpos[:3]
        dist = np.linalg.norm(pos[:2] - self.target_pos[:2])
        
        # Terminated if reached goal or fell off
        return dist < 0.3 or pos[2] < 0.0 or abs(pos[0]) > 20 or abs(pos[1]) > 20


class ScorpioLocomotionEnv(ScorpioBaseEnv):
    """Locomotion task: maintain stable forward motion."""

    def __init__(self, model_path=None, render_mode="human"):
        super().__init__(model_path, render_mode)
        self._observation_dim = 2 + 3 + 6  # vel + height/roll/pitch + imu

    def _get_obs(self) -> np.ndarray:
        linvel = self._read_sensor(self._base_linvel_id, 3)
        angvel = self._read_sensor(self._base_angvel_id, 3)
        vel_obs = np.array([linvel[0], angvel[2]])
        
        pos = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        # Height, roll, pitch approximation
        height_pitch_roll = np.array([pos[2], quat[1], quat[2]])
        
        accel = self._read_sensor(self._imu_accel_id, 3)
        gyro = self._read_sensor(self._imu_gyro_id, 3)
        imu_obs = np.concatenate([accel, gyro])
        
        return np.concatenate([vel_obs, height_pitch_roll, imu_obs]).astype(np.float32)

    def _compute_reward(self) -> float:
        linvel = self._read_sensor(self._base_linvel_id, 3)
        target_vel = 0.2  # Target forward velocity
        
        vel_tracking = -abs(linvel[0] - target_vel)
        stability = -abs(self.data.qpos[2] - 0.0525) * 10.0  # Height maintenance
        energy = -np.sum(np.abs(self.data.ctrl)) * 0.01  # Energy efficiency
        
        return vel_tracking + stability + energy

    def _check_terminated(self) -> bool:
        return self.data.qpos[2] < 0.02  # Fell over


def register_gym_envs():
    """Register environments with gymnasium (if available)."""
    try:
        import gymnasium as gym
        from gymnasium.envs.registration import register
        
        register(
            id='scorpio-nav-v0',
            entry_point=lambda: ScorpioNavEnv(),
            max_episode_steps=1000,
        )
        
        register(
            id='scorpio-locomotion-v0',
            entry_point=lambda: ScorpioLocomotionEnv(),
            max_episode_steps=500,
        )
        
        print("✅ Registered scorpio-nav-v0 and scorpio-locomotion-v0")
    except ImportError:
        print("⚠️  gymnasium not installed, skipping registration")


if __name__ == '__main__':
    # Quick test
    print("Testing ScorpioNavEnv...")
    env = ScorpioNavEnv(render_mode="human")
    obs, info = env.reset(seed=42)
    print(f"  Observation shape: {obs.shape}")
    print(f"  Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
    
    total_reward = 0
    for i in range(100):
        action = np.array([0.5, 0.0])  # Forward
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        env.render()
        if terminated or truncated:
            break
    
    print(f"  Total reward (100 steps): {total_reward:.3f}")
    env.close()
    print("✅ Environment test passed!")
