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
import commands2
from commands2 import Command, InstantCommand, RunCommand
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d

import oi
from commands.superstructure import Superstructure
from commands.swerve import SkiStopCommand, DriveToPoseCommand
from constants import DrivingConstants, ArmConstants, ElevatorConstants, ClimberConstants, ClawConstants
from oi import XboxDriver, XboxOperator, KeyboardScoringPositions
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
        self.operator_joystick = XboxOperator(1)
        self.button_board = KeyboardScoringPositions(2)
        self.sysid_joystick = CommandXboxController(3)

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
        self.elevator.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.elevator.set_duty_cycle(oi.deadband(-self.operator_joystick.stick.getLeftY(), 0.08)),
                self.elevator,
            )
        )
        SmartDashboard.putData("Elevator", self.elevator)

        # Configure arm subsystem
        self.arm = Arm()
        self.arm.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.arm.set_duty_cycle(oi.deadband(-self.operator_joystick.stick.getRightY(), 0.08)),
                self.arm,
            )
        )
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
        return Command()  # self.elevator.HomeElevator()

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
        self.driver_joystick.stick.povDown().whileTrue(RunCommand(lambda: self.elevator.set_height(53 * 0.0254), self.elevator))
        self.driver_joystick.stick.povLeft().whileTrue(RunCommand(lambda: self.elevator.set_height(71 * 0.0254), self.elevator))
        #self.driver_joystick.stick.povDown().whileTrue(RunCommand(lambda: self.elevator.set_height(0.956564), self.elevator))
        #self.driver_joystick.stick.povLeft().whileTrue(RunCommand(lambda: self.elevator.set_height(1.31191), self.elevator))
        self.driver_joystick.stick.povUp().whileTrue(RunCommand(lambda: self.elevator.set_height(80 * 0.0254), self.elevator))

        # Arm buttons
        self.operator_joystick.stick.y().whileTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(30)), self.arm))
        self.operator_joystick.stick.x().whileTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(60)), self.arm))
        #self.joystick.stick.cross().onTrue(RunCommand(lambda: self.arm.set_angle(Rotation2d.fromDegrees(-45)), self.arm))

        self.operator_joystick.stick.leftTrigger().whileTrue(self.claw.IntakeCommand().beforeStarting(commands2.PrintCommand("hi")))
        self.operator_joystick.stick.leftBumper().whileTrue(self.claw.OuttakeCommand())

        # Configure Elevator Operator buttons

        """
        # LOADER HEIGHT (LEVEL 0)
        # RIGHT TRIGGER
        self.operator_joystick.loading_level.onTrue(
            self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_0_HEIGHT, ArmConstants.LEVEL_0_ROTATION)
        )

        # LEVEL 1 HEIGHT
        # A BUTTON
        self.operator_joystick.level_1.onTrue(
            self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_1_HEIGHT, ArmConstants.LEVEL_1_ROTATION)
        )

        # LEVEL 2 HEIGHT
        # X BUTTON
        self.operator_joystick.level_2.onTrue(
            self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_2_HEIGHT, ArmConstants.LEVEL_2_ROTATION)
        )

        # LEVEL 3 HEIGHT
        # B BUTTON
        self.operator_joystick.level_3.onTrue(
            self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_3_HEIGHT, ArmConstants.LEVEL_3_ROTATION)
        )

        # LEVEL 4 HEIGHT
        # Y BUTTON
        self.operator_joystick.level_4.onTrue(
            self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_4_HEIGHT, ArmConstants.LEVEL_4_ROTATION)
        )
        """

        # Reef Positions
        for reef_label, coordinates in DrivingConstants.CORAL_LOCATIONS.items():
            # Get the Trigger object from the OI button board class by referencing its name
            # e.g, self.button_board.reef_a is referenced by "reef_a"
            button: Trigger = getattr(self.button_board, reef_label.lower())
            # Use the x, y, theta coordinates from the constants file to make a Pose2d
            pose = construct_Pose2d(*coordinates)
            # When the button is pushed, start pathfinding to the desired pose.
            # Then, run DriveToPoseCommand perpetually to keep the robot locked onto the desired position
            # even if we get pushed by another robot. The command stops when the button is released.
            button.whileTrue(
                AutoBuilder.pathfindToPose(pose, PathConstraints.unlimitedConstraints(12)) \
                    .andThen(commands2.PrintCommand("Finished")) \
                    .andThen(DriveToPoseCommand(self.swerve, pose, AUTONOMOUS_PARAMS))
            )

        # SysId routines
        self.sysid_joystick.y().whileTrue(self.elevator.SysIdQuasistatic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.a().whileTrue(self.elevator.SysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        self.sysid_joystick.b().whileTrue(self.elevator.SysIdDynamic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.x().whileTrue(self.elevator.SysIdDynamic(SysIdRoutine.Direction.kReverse))

        self.sysid_joystick.povUp().whileTrue(self.arm.SysIdQuasistatic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.povDown().whileTrue(self.arm.SysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        self.sysid_joystick.povRight().whileTrue(self.arm.SysIdDynamic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.povLeft().whileTrue(self.arm.SysIdDynamic(SysIdRoutine.Direction.kReverse))

        # self.joystick.stick.a().onTrue(self.superstructure.SetEndEffectorHeight(2.5, Rotation2d.fromDegrees(-30)))
        # self.driver_joystick.stick.b().whileTrue(DriveToPoseCommand(self.swerve, Pose2d(7, 6, Rotation2d.fromDegrees(30)), AUTONOMOUS_PARAMS))

    def register_named_commands(self):
        pass


def construct_Pose2d(x: float, y: float, degrees: float) -> Pose2d:
    return Pose2d(x, y, Rotation2d.fromDegrees(degrees))
