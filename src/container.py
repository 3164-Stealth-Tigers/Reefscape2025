from commands2 import Command, InstantCommand
from commands2.button import CommandXboxController, CommandPS4Controller
from wpilib import DriverStation, SmartDashboard

from subsystems.elevator import Elevator


class RobotContainer:
    def __init__(self):
        DriverStation.silenceJoystickConnectionWarning(True)

        self.joystick = CommandXboxController(0)

        # Configure elevator subsystem
        self.elevator = Elevator()
        SmartDashboard.putData("Elevator", self.elevator)

        # Button bindings must be configured after every subsystem has been set up
        self.configure_button_bindings()

    def get_autonomous_command(self) -> Command:
        return Command()

    def configure_button_bindings(self):
        # Elevator buttons
        self.joystick.a().onTrue(InstantCommand(lambda: self.elevator.set_height(1)))
        self.joystick.x().onTrue(InstantCommand(lambda: self.elevator.set_height(2)))
        self.joystick.y().onTrue(InstantCommand(lambda: self.elevator.set_height(3)))
