from typing import Optional

import commands2
from wpilib import DataLogManager

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        # Start recording NetworkTables to data log
        DataLogManager.start()

        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()
