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

        self.claw = self.container.claw

        self.sensor = self.container.claw.distance_sensor
        self.sensor.setRangingMode(self.sensor.RangingMode.kShort, 100.0)


    # Sensor checking functions
    def isvalid(self): # Check if Sensor Range & Status are valid
        if self.isRangeValid() and self.getStatus() == self.Status.kValid:
            return True
        else:
            return False

    def getinvalid(self): # Find and return sensor issues
        invalid = []
        if not(self.isRangeValid()):
            invalid.append("Range")

        if self.getStatus() != self.Status.kValid:
            invalid.append("Status")

        return invalid
   # END OF SENSOR CHECKING FUNCTIONS

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def robotPeriodic(self):

        dist = self.sensor.getRange()
        status = self.sensor.getStatus()

        # sensor getStatus() returns status (valid, invalid, ???)
        # sensor getRange() returns distance

        if self.sensor.isvalid():
            if self.claw.has_possession():
                print('Claw has possesion')
                print(f"The distance is {dist}, and the status is {status}")
                pass
            else:
                print('Claw is empty')
                pass
        else:
            print('Sensor is invalid')

            invalids = self.sensor.getinvalid()

            for invalid in invalids:
                print("Issue: "+invalid)
            pass


