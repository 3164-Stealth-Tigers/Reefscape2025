from commands2 import Command, InstantCommand, RunCommand
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d

from commands.superstructure import Superstructure
from commands.swerve import ski_stop_command
from constants import DrivingConstants
from oi import XboxDriver, PS4Driver
from subsystems.arm import Arm
from subsystems.climber import Climber
from subsystems.elevator import Elevator
from swerve_config import SWERVE_MODULES, GYRO, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, AUTONOMOUS_PARAMS
from swervepy import SwerveDrive


class RobotContainer:
    def __init__(self):
        DriverStation.silenceJoystickConnectionWarning(True)

        self.joystick = XboxDriver(0)

        # Configure drivetrain
        self.swerve = SwerveDrive(SWERVE_MODULES, GYRO, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, AUTONOMOUS_PARAMS)
        self.teleop_drive_command = self.swerve.teleop_command(
            self.joystick.forward,
            self.joystick.strafe,
            self.joystick.turn,
            DrivingConstants.FIELD_RELATIVE,
            DrivingConstants.OPEN_LOOP,
        )
        self.swerve.setDefaultCommand(self.teleop_drive_command)

        # Configure elevator subsystem
        self.elevator = Elevator()
        SmartDashboard.putData("Elevator", self.elevator)

        # Configure arm subsystem
        self.arm = Arm()

        # Configure climber subsystem
        self.climber = Climber()

        # The superstructure contains commands that require multiple subsystems
        self.superstructure = Superstructure(self.elevator, self.arm, self.climber)

        # Register Named Commands for PathPlanner after initializing subsystems but before the rest of init
        self.register_named_commands()

        # Button bindings must be configured after every subsystem has been set up
        self.configure_button_bindings()

    def get_autonomous_command(self) -> Command:
        return Command()

    def configure_button_bindings(self):
        # Driving buttons
        self.joystick.reset_gyro.onTrue(InstantCommand(self.swerve.zero_heading))
        self.joystick.toggle_field_relative.onTrue(InstantCommand(self.teleop_drive_command.toggle_field_relative))
        self.joystick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.joystick.is_movement_commanded))

        """
        # Elevator buttons
        self.joystick.stick.povDown().onTrue(InstantCommand(lambda: self.elevator.set_height(1), self.elevator))
        self.joystick.stick.povLeft().onTrue(InstantCommand(lambda: self.elevator.set_height(2), self.elevator))
        self.joystick.stick.povUp().onTrue(InstantCommand(lambda: self.elevator.set_height(3), self.elevator))

        # Arm buttons
        self.joystick.stick.square().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(30)), self.arm))
        self.joystick.stick.circle().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(60)), self.arm))
        self.joystick.stick.cross().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(-45)), self.arm))
        """

        self.joystick.stick.square().onTrue(self.superstructure.SetEndEffectorHeight(2.5, Rotation2d.fromDegrees(-30)))

    def register_named_commands(self):
        pass
