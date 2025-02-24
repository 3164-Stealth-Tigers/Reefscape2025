"""
Commands Reference:

AutoBuilder.followPath(path) --> Follow an individual path created in PathPlanner.
AutoBuilder.buildAuto(auto_name) --> Follow a complete autonomous routine created in PathPlanner.
self.superstructure.SetEndEffectorHeight(height, angle)
DriveToPoseCommand(self.swerve, pose, parameters) --> Locks the robot to a position and rotation on the field.
self.claw.IntakeCommand()
self.claw.OuttakeCommand() --> Ejects the CORAL from the claw. This command ends after the CORAL has been ejected.
"""
import commands2
from commands2 import Command, InstantCommand, RunCommand
from commands2.button import CommandXboxController
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d

import oi
import constants
from commands.superstructure import Superstructure
from commands.swerve import SkiStopCommand, DriveToPoseCommand
from constants import DrivingConstants
from oi import XboxDriver, XboxOperator  # , PS4Driver
from subsystems.arm import Arm
from subsystems.claw import Claw
from subsystems.climber import Climber
from subsystems.elevator import Elevator
from swerve_config import SWERVE_MODULES, GYRO, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, AUTONOMOUS_PARAMS
from swervepy import SwerveDrive


class RobotContainer:
    def __init__(self):
        DriverStation.silenceJoystickConnectionWarning(True)

        self.driver_joystick = XboxDriver(0)
        self.operator_joystick = CommandXboxController(1)
        self.sysid_joystick = CommandXboxController(2)

        # Configure drivetrain
        self.swerve = SwerveDrive(SWERVE_MODULES, GYRO, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, AUTONOMOUS_PARAMS)
        self.teleop_drive_command = self.swerve.teleop_command(
            self.driver_joystick.forward,
            self.driver_joystick.strafe,
            self.driver_joystick.turn,
            DrivingConstants.FIELD_RELATIVE,
            DrivingConstants.OPEN_LOOP,
        )
        self.swerve.setDefaultCommand(self.teleop_drive_command)

        # Configure elevator subsystem
        self.elevator = Elevator()
        #self.elevator.setDefaultCommand(
        #    commands2.RunCommand(
        #        lambda: self.elevator.set_duty_cycle(oi.deadband(-self.operator_joystick.getLeftY(), 0.05)),
        #        self.elevator,
        #    )
        #)
        SmartDashboard.putData("Elevator", self.elevator)

        # Configure arm subsystem
        self.arm = Arm()
        #self.arm.setDefaultCommand(
        #    commands2.RunCommand(
        #        lambda: self.arm.set_duty_cycle(oi.deadband(-self.operator_joystick.getRightY(), 0.05)),
        #        self.arm,
        #    )
        #)
        SmartDashboard.putData("Arm", self.arm)

        # Configure climber subsystem
        self.climber = Climber()

        self.claw = Claw()

        # The superstructure contains commands that require multiple subsystems
        self.superstructure = Superstructure(self.elevator, self.arm, self.climber)

        # Register Named Commands for PathPlanner after initializing subsystems but before the rest of init
        self.register_named_commands()

        # Button bindings must be configured after every subsystem has been set up
        self.configure_button_bindings()

    def get_autonomous_command(self) -> Command:
        return self.elevator.HomeElevator()

        """
        first_path = PathPlannerPath.fromPathFile("Start to R (mixed)")
        return commands2.SequentialCommandGroup(
            AutoBuilder.resetOdom(first_path.getStartingHolonomicPose()),
            commands2.ParallelRaceGroup(
                self.superstructure.SetEndEffectorHeight(2.5, Rotation2d.fromDegrees(-30)),
                AutoBuilder.followPath(first_path),
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("R to Loader 1 (mixed)")),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Loader 1 to TR (mixed)")),
        )
        """

    def configure_button_bindings(self):
        # Driving buttons
        self.driver_joystick.reset_gyro.onTrue(InstantCommand(self.swerve.zero_heading))
        self.driver_joystick.toggle_field_relative.onTrue(InstantCommand(self.teleop_drive_command.toggle_field_relative))
        self.driver_joystick.ski_stop.onTrue(SkiStopCommand(self.swerve).until(self.driver_joystick.is_movement_commanded))

        # Elevator buttons
        self.driver_joystick.stick.povDown().whileTrue(RunCommand(lambda: self.elevator.set_height(1), self.elevator))
        self.driver_joystick.stick.povLeft().whileTrue(RunCommand(lambda: self.elevator.set_height(2), self.elevator))
        self.driver_joystick.stick.povUp().whileTrue(RunCommand(lambda: self.elevator.set_height(3), self.elevator))

        """
        # Arm buttons
        self.joystick.stick.square().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(30)), self.arm))
        self.joystick.stick.circle().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(60)), self.arm))
        self.joystick.stick.cross().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(-45)), self.arm))
        """
        # Configure Elevator Operator buttons

        # LOADER HEIGHT (LEVEL 0)
        # RIGHT TRIGGER
        self.workstick.stick.rightTrigger().onTrue(
            self.superstructure.SetEndEffectorHeight(constants.ElevatorConstants.LEVEL_0_HEIGHT, 0))

        self.driver_joystick.stick.a().onTrue(self.superstructure.SetEndEffectorHeight(2.5, Rotation2d.fromDegrees(-30)))
        self.driver_joystick.stick.b().whileTrue(DriveToPoseCommand(self.swerve, Pose2d(7, 6, Rotation2d.fromDegrees(30)), AUTONOMOUS_PARAMS))

        # SysId routines
        self.sysid_joystick.y().whileTrue(self.elevator.SysIdQuasistatic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.a().whileTrue(self.elevator.SysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        self.sysid_joystick.b().whileTrue(self.elevator.SysIdDynamic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.x().whileTrue(self.elevator.SysIdDynamic(SysIdRoutine.Direction.kReverse))

        self.sysid_joystick.povUp().whileTrue(self.arm.SysIdQuasistatic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.povDown().whileTrue(self.arm.SysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        self.sysid_joystick.povRight().whileTrue(self.arm.SysIdDynamic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.povLeft().whileTrue(self.arm.SysIdDynamic(SysIdRoutine.Direction.kReverse))
        # LEVEL 1 HEIGHT
        # A BUTTON
        self.workstick.stick.a().onTrue(
            self.superstructure.SetEndEffectorHeight(constants.ElevatorConstants.LEVEL_1_HEIGHT, 0))

        # LEVEL 2 HEIGHT
        # X BUTTON
        self.workstick.stick.x().onTrue(
            self.superstructure.SetEndEffectorHeight(constants.ElevatorConstants.LEVEL_2_HEIGHT, 0))

        # LEVEL 3 HEIGHT
        # B BUTTON
        self.workstick.stick.b().onTrue(
            self.superstructure.SetEndEffectorHeight(constants.ElevatorConstants.LEVEL_3_HEIGHT, 0))

        # LEVEL 4 HEIGHT
        # Y BUTTON
        self.workstick.stick.y().onTrue(
            self.superstructure.SetEndEffectorHeight(constants.ElevatorConstants.LEVEL_4_HEIGHT, 0))

        # self.joystick.stick.a().onTrue(self.superstructure.SetEndEffectorHeight(2.5, Rotation2d.fromDegrees(-30)))
        self.joystick.stick.b().whileTrue(DriveToPoseCommand(self.swerve, Pose2d(7, 6, Rotation2d.fromDegrees(30)), AUTONOMOUS_PARAMS))

    def register_named_commands(self):
        pass
