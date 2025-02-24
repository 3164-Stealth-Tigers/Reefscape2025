from typing import Optional

import constants
from constants import ClawConstants
from subsystems import claw
import commands2

from container import RobotContainer
import playingwithfusion as pwf


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None
        self.elevator = self.container.elevator
        self.claw = self.container.claw

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def robotPeriodic(self):
        # if self.elevator.limit_switch:
        #     if self.elevator.lower_limit():
        #         print("Magnet detected")
        #     else:
        #         print("Magnet not detected")
        # else:
        pass


