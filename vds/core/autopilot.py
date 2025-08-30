# vds/core/autopilot.py

import numpy as np

class Autopilot:
    """
    A simple PID-based autopilot for track-keeping.
    항로 유지를 위한 간단한 PID 기반 오토파일럿.
    """
    def __init__(self, max_rudder_angle: float = 35.0):
        # PID 제어기 계수 (이 값들을 조정하여 성능 튜닝 가능)
        self.kp = 0.8  # Proportional gain (주 제어)
        self.ki = 0.01 # Integral gain (정상상태 오차 보정)
        self.kd = 1.5  # Derivative gain (급격한 변화 억제)
        
        self.max_rudder = max_rudder_angle
        self._integral = 0.0
        self._previous_error = 0.0

    def reset(self):
        """Resets the PID controller's internal state."""
        self._integral = 0.0
        self._previous_error = 0.0

    def calculate_rudder_angle(self, current_pos, current_heading_rad, target_pos, dt) -> float:
        """
        Calculates the required rudder angle to steer towards the target.
        목표 지점으로 향하기 위해 필요한 타각을 계산합니다.
        """
        # 1. 목표 방위각 계산 (Bearing to target)
        vector_to_target = target_pos - current_pos
        target_bearing_rad = np.arctan2(vector_to_target[1], vector_to_target[0])

        # 2. 방위각 오차 계산 (Angle error)
        error = target_bearing_rad - current_heading_rad
        # 오차를 -pi ~ +pi 범위로 정규화
        error = (error + np.pi) % (2 * np.pi) - np.pi

        # 3. PID 제어 계산
        # P (Proportional)
        p_term = self.kp * error

        # I (Integral)
        self._integral += error * dt
        i_term = self.ki * self._integral

        # D (Derivative)
        derivative = (error - self._previous_error) / dt
        d_term = self.kd * derivative
        self._previous_error = error
        
        # 최종 타각 계산
        demanded_rudder_rad = p_term + i_term + d_term
        
        # 최대 타각 제한
        demanded_rudder_deg = np.degrees(demanded_rudder_rad)
        
        return np.clip(demanded_rudder_deg, -self.max_rudder, self.max_rudder)
