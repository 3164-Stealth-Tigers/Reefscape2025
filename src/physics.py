from typing import TYPE_CHECKING

from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import RoboRioSim, BatterySim

from helpers import SimHelper

if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller

    def update_sim(self, now: float, tm_diff: float) -> None:
        # print(f"Timestep: {now:.3f}  Currents: {[x[1] for x in SimHelper.currents]}")

        # Remove stale current loads and create a list of currents without timestamps
        amps_list = [x[1] for x in SimHelper.currents if now - x[0] <= tm_diff]
        # print(f"Pruned {len(SimHelper.currents) - len(amps_list)}")

        # Simulate the battery's output voltage based on all current loads
        RoboRioSim.setVInVoltage(BatterySim.calculate(amps_list))

        # Clear the queue for the next physics step
        SimHelper.clear_simulated_currents()
