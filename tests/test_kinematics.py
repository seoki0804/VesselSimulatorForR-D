# tests/test_kinematics.py

import numpy as np
import pytest
from vds.core.kinematics import update_kinematics_6dof
from vds.models.vessels.base_vessel import VesselState

def test_forward_movement():
    """
    Tests if the vessel moves straight forward correctly.
    선박이 정확하게 직진하는지 테스트합니다.
    """
    # Initial state: At origin (0,0), with 10 m/s forward speed
    # 초기 상태: 원점(0,0)에서 10m/s의 전진 속도
    initial_state = VesselState(nu=np.array([10.0, 0, 0, 0, 0, 0]))
    
    # Simulate for 5 seconds with a time step of 0.1s
    # 0.1초 간격으로 5초 동안 시뮬레이션
    state = initial_state
    dt = 0.1
    for _ in range(50):
        state = update_kinematics_6dof(state, dt)

    # After 5 seconds, the vessel should be at x=50, y=0
    # 5초 후, 선박은 x=50, y=0 위치에 있어야 합니다.
    final_pos = state.eta
    assert final_pos[0] == pytest.approx(50.0)
    assert final_pos[1] == pytest.approx(0.0)
    assert final_pos[5] == pytest.approx(0.0) # Heading should not change

def test_turning_movement():
    """
    Tests if the vessel turns correctly.
    선박이 정확하게 선회하는지 테스트합니다.
    """
    # Initial state: At origin, 10 m/s forward speed, and 0.1 rad/s yaw rate (~5.7 deg/s)
    # 초기 상태: 원점에서 10m/s 전진, 0.1 rad/s 선회 각속도
    initial_state = VesselState(nu=np.array([10.0, 0, 0, 0, 0, 0.1]))

    # Simulate for 10 seconds
    # 10초 동안 시뮬레이션
    state = initial_state
    dt = 0.1
    for _ in range(100):
        state = update_kinematics_6dof(state, dt)
        
    # After 10 seconds, the heading should be 0.1 rad/s * 10s = 1.0 radian
    # 10초 후, 선수각(heading)은 0.1 rad/s * 10s = 1.0 라디안이 되어야 합니다.
    final_heading_rad = state.eta[5]
    assert final_heading_rad == pytest.approx(1.0)
    
    # The vessel should have moved in an arc, so it should not be at x=100
    # 선박은 원호를 그리며 움직였으므로, x=100 위치에 있으면 안 됩니다.
    assert state.eta[0] < 100.0
    assert state.eta[1] > 0.0 # Should have moved to the right (East)

# --- 테스트 실행 방법 ---
# 1. 터미널에서 프로젝트 최상위 폴더로 이동합니다.
# 2. `pip install pytest` 명령어로 pytest를 설치합니다 (requirements.txt로 이미 설치했다면 생략).
# 3. 터미널에 `pytest` 라고 입력하고 엔터를 치면, tests/ 폴더의 모든 테스트를 자동으로 실행하고 결과를 보여줍니다.
