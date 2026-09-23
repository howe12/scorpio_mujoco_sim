#!/usr/bin/env python3
"""
Standalone MuJoCo visualization and testing for Scorpio robot.
No ROS2 dependency required - useful for quick model validation and RL experiments.

Usage:
    python3 test_scorpio_mujoco.py              # Interactive viewer
    python3 test_scorpio_mujoco.py --headless   # Headless benchmark
    python3 test_scorpio_mujoco.py --rl-test    # Simple RL policy test
"""

import mujoco
import mujoco.viewer
import numpy as np
import argparse
import time
import os
import sys


class ScorpioSimulator:
    """Standalone Scorpio robot simulator using MuJoCo."""

    WHEEL_RADIUS = 0.0525
    WHEEL_SEPARATION = 0.1856

    def __init__(self, model_path=None, headless=False):
        if model_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(script_dir, '..', 'models', 'scorpio.xml')
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.headless = headless
        
        # Joint indices
        self.joint_names = [
            'left_front_wheel_joint', 'left_rear_wheel_joint',
            'right_front_wheel_joint', 'right_rear_wheel_joint'
        ]
        
        print(f"Model loaded: {model_path}")
        print(f"  Bodies: {self.model.nbody}, Joints: {self.model.njnt}")
        print(f"  Actuators: {self.model.nu}, Sensors: {self.model.nsensordata}")
        print(f"  Timestep: {self.model.opt.timestep}s")

    def set_velocity(self, linear: float, angular: float):
        """Set differential drive velocity command."""
        left_vel = (linear + angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        right_vel = (linear - angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        
        self.data.ctrl[0] = left_vel   # left front
        self.data.ctrl[1] = left_vel   # left rear
        self.data.ctrl[2] = right_vel  # right front
        self.data.ctrl[3] = right_vel  # right rear

    def get_pose(self):
        """Get base position and orientation."""
        pos = self.data.qpos[:3].copy()
        quat = self.data.qpos[3:7].copy()
        return pos, quat

    def step(self):
        """Advance simulation by one timestep."""
        mujoco.mj_step(self.model, self.data)

    def run_interactive(self):
        """Run with interactive MuJoCo viewer."""
        print("\n🎮 Interactive Mode:")
        print("  Keyboard controls in viewer:")
        print("    Ctrl+Click+Drag to rotate camera")
        print("    Right-Click+Drag to pan")
        print("    Scroll to zoom")
        print("\n  Programmatic control: use set_velocity(linear, angular)")
        print("  Press ESC or close window to exit\n")
        
        # Set initial forward motion
        self.set_velocity(0.15, 0.0)
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                self.step()
                viewer.sync()
                
                # Optional: print pose periodically
                if int(self.data.time * 10) % 10 == 0:
                    pos, _ = self.get_pose()
                    print(f"\r  t={self.data.time:.1f}s  pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})", end='')

    def run_headless_benchmark(self, duration=10.0):
        """Run headless performance benchmark."""
        print(f"\n⏱️  Headless Benchmark ({duration}s simulated):")
        
        self.set_velocity(0.2, 0.1)
        
        start_time = time.time()
        steps = 0
        target_steps = int(duration / self.model.opt.timestep)
        
        while steps < target_steps:
            self.step()
            steps += 1
            
            if steps % 10000 == 0:
                elapsed = time.time() - start_time
                sps = steps / elapsed
                print(f"  Steps: {steps}/{target_steps} | "
                      f"Sim time: {self.data.time:.1f}s | "
                      f"Speed: {sps:.0f} steps/s")
        
        elapsed = time.time() - start_time
        pos, _ = self.get_pose()
        
        print(f"\n  ✅ Completed {steps} steps in {elapsed:.2f}s")
        print(f"  Simulation speed: {steps/elapsed:.0f} steps/s "
              f"({duration/elapsed:.1f}x realtime)")
        print(f"  Final position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    def run_rl_test(self, episodes=5, episode_length=500):
        """Simple random policy test for RL workflow validation."""
        print(f"\n🤖 RL Policy Test ({episodes} episodes x {episode_length} steps):")
        
        rewards = []
        
        for ep in range(episodes):
            # Reset
            mujoco.mj_resetData(self.model, self.data)
            episode_reward = 0.0
            
            for step in range(episode_length):
                # Random action (exploration)
                linear = np.random.uniform(-0.2, 0.2)
                angular = np.random.uniform(-0.5, 0.5)
                self.set_velocity(linear, angular)
                
                self.step()
                
                # Simple reward: forward progress + stability
                pos = self.data.qpos[:3]
                linvel = self.data.sensordata[
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_linvel') * 3
                    : mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_linvel') * 3 + 3
                ]
                
                # Reward: forward velocity - lateral drift - tilt penalty
                reward = linvel[0] - abs(linvel[1]) * 0.5 - abs(pos[2] - 0.0525) * 10.0
                episode_reward += reward
            
            rewards.append(episode_reward)
            pos, _ = self.get_pose()
            print(f"  Episode {ep+1}: reward={episode_reward:.3f} | "
                  f"final_pos=({pos[0]:.2f}, {pos[1]:.2f})")
        
        print(f"\n  Mean reward: {np.mean(rewards):.3f} ± {np.std(rewards):.3f}")
        print(f"  ✅ RL environment validation complete!")


def main():
    parser = argparse.ArgumentParser(description='Scorpio MuJoCo Simulator')
    parser.add_argument('--model', type=str, default=None, help='Path to MJCF model')
    parser.add_argument('--headless', action='store_true', help='Run without GUI')
    parser.add_argument('--rl-test', action='store_true', help='Run RL policy test')
    parser.add_argument('--benchmark-duration', type=float, default=10.0, help='Benchmark duration (s)')
    args = parser.parse_args()
    
    sim = ScorpioSimulator(model_path=args.model, headless=args.headless)
    
    if args.rl_test:
        sim.run_rl_test()
    elif args.headless:
        sim.run_headless_benchmark(duration=args.benchmark_duration)
    else:
        sim.run_interactive()


if __name__ == '__main__':
    main()
