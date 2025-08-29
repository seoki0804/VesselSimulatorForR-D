# main.py

import numpy as np
from vds.core.simulator import Simulator
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.models.dynamics.simple_3dof_model import Simple3DOFModel

def run_turning_test():
    """
    Sets up and runs a simple turning maneuver test.
    """
    # 1. Define Vessel Specifications
    specs = VesselSpecifications(
        length=150.0,       # meters
        width=25.0,        # meters
        draft=10.0,        # meters
        displacement=20000 # tons
    )

    # 2. Define Initial State (start with some forward speed)
    initial_state = VesselState(
        eta=np.zeros(6),
        nu=np.array([5.0, 0, 0, 0, 0, 0]) # 5 m/s forward speed (~10 knots)
    )

    # 3. Create a Vessel Instance
    vessel = BaseVessel(specs, initial_state)

    # 4. Create a Dynamics Model Instance
    # Simple physics parameters (mass in kg)
    mass = specs.displacement * 1025 # mass = displacement * water density
    Iz = 0.5 * mass * (specs.width**2) # Simplified moment of inertia
    dynamics_model = Simple3DOFModel(vessel_mass=mass, inertia_z=Iz)

    # 5. Create the Simulator
    simulator = Simulator(vessel, dynamics_model)

    # 6. Define Control Inputs for the simulation run
    # Keep constant propeller RPM and apply 15 degrees of rudder
    control = {
        'n_rpm': 80.0,         # Propeller speed in RPM
        'delta_deg': 15.0      # Rudder angle in degrees
    }

    # 7. Run the simulation
    simulator.run(duration=300, dt=0.1, initial_control=control)


if __name__ == "__main__":
    run_turning_test()
