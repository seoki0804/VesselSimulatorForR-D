# tests/test_mmg_model.py

import pytest
import numpy as np
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.models.dynamics.mmg_model import MMGModel

@pytest.fixture
def kcs_model():
    """
    Provides a default KCS vessel and MMG model for testing.
    테스트를 위한 기본 KCS 선박과 MMG 모델을 제공합니다.
    """
    specs = VesselSpecifications(
        loa=232.5, beam=32.2, draft=10.8, mass=5.2e7, inertia_z=2.17e10,
        wind_area_longitudinal=800.0, wind_area_transverse=2500.0
    )
    hydro_params_path = 'data/vessel_params/kcs_hydrodynamics.json'
    model = MMGModel(specs, hydro_params_path)
    return model

def test_straight_running_symmetry(kcs_model):
    """
    Tests if the vessel runs straight with zero rudder angle.
    방향타가 0도일 때 선박이 직진하는지 테스트합니다.
    """
    # Initial state: 15 knots forward speed, no sway or yaw rate.
    # 초기 상태: 15노트 전진 속도, 횡방향 속도나 선회 각속도는 없음.
    state = VesselState(nu=np.array([7.7, 0, 0, 0, 0, 0]))
    control = {'rpm': 85.0, 'rudder_angle': 0.0}

    # Calculate forces
    # 힘 계산
    nu_dot = kcs_model.calculate_forces(state, control)

    # Assertions: There should be no significant sway force or yaw moment.
    # 검증: 유의미한 횡방향 힘이나 선회 모멘트가 발생해서는 안 됩니다.
    assert nu_dot[1] == pytest.approx(0.0, abs=1e-5) # Y-acceleration
    assert nu_dot[5] == pytest.approx(0.0, abs=1e-5) # N-acceleration

def test_starboard_turn_direction(kcs_model):
    """
    Tests if a starboard rudder command results in a starboard turn.
    우현 타각이 우현 선회를 유발하는지 테스트합니다.
    """
    # Initial state: 15 knots forward speed.
    # 초기 상태: 15노트 전진 속도.
    state = VesselState(nu=np.array([7.7, 0, 0, 0, 0, 0]))
    # Control: 35 degrees starboard rudder.
    # 제어: 우현 35도 타각.
    control = {'rpm': 85.0, 'rudder_angle': 35.0}

    # Calculate forces
    # 힘 계산
    nu_dot = kcs_model.calculate_forces(state, control)

    # Assertions: A starboard turn should result in a positive yaw acceleration.
    # 검증: 우현 선회는 양수(+)의 선회 가속도를 발생시켜야 합니다.
    assert nu_dot[5] > 0 # N-acceleration must be positive
