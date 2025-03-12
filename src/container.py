"""
Commands Reference:

AutoBuilder.followPath(path) --> Follow an individual path created in PathPlanner.
AutoBuilder.buildAuto(auto_name) --> Follow a complete autonomous routine created in PathPlanner.
self.superstructure.SetEndEffectorHeight(height, angle)
DriveToPoseCommand(self.swerve, pose, parameters) --> Locks the robot to a position and rotation on the field.
self.claw.IntakeCommand()
self.claw.OuttakeCommand() --> Ejects the CORAL from the claw. This command ends after the CORAL has been ejected.

Measurements:
Distance from front of robot frame to REEF when scoring L2-4: 11.75 inch


"""
import commands2
import wpilib
from commands2 import Command, InstantCommand, RunCommand
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d

import constants
import swerve_config
from constants import ElevatorConstants, ClawConstants
import oi
from commands.superstructure import Superstructure
from commands.swerve import SkiStopCommand, DriveToPoseCommand
from constants import DrivingConstants, CoralArmConstants, ElevatorConstants, ClimberConstants, ClawConstants
from oi import XboxDriver, XboxOperator, KeyboardScoringPositions, ArcadeScoringPositions
from subsystems.coral_arm import CoralArm
from subsystems.claw import Claw
from subsystems.climber import Climber
from subsystems.elevator import Elevator
from swerve_config import SWERVE_MODULES, GYRO, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, AUTONOMOUS_PARAMS
from swervepy import SwerveDrive
from vision import Vision


class RobotContainer:
    def __init__(self):
        DriverStation.silenceJoystickConnectionWarning(True)

        self.driver_joystick = XboxDriver(0)
        self.operator_joystick = XboxOperator(1)
        self.button_board = ArcadeScoringPositions(2)
        self.sysid_joystick = CommandXboxController(3)

        self.auto_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData(self.auto_chooser)

        camera_system = Vision()

        # Configure drivetrain
        self.swerve = SwerveDrive(
            SWERVE_MODULES,
            GYRO, MAX_VELOCITY,
            MAX_ANGULAR_VELOCITY,
            AUTONOMOUS_PARAMS,
            camera_system.get_pose_estimation,
        )
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
                lambda: self.elevator.set_voltage(0.96 + oi.deadband(-self.operator_joystick.stick.getLeftY(), 0.08) * 6),
                self.elevator,
            )
        )
        SmartDashboard.putData("Elevator", self.elevator)

        # Configure arm subsystem
        self.arm = CoralArm()
        self.arm.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.arm.set_duty_cycle(oi.deadband(-self.operator_joystick.stick.getRightY(), 0.08)),
                self.arm,
            )
        )
        SmartDashboard.putData("Coral Arm", self.arm)

        # Configure climber subsystem
        self.climber = Climber()

        self.claw = Claw()
        SmartDashboard.putData("Claw", self.claw)

        # The superstructure contains commands that require multiple subsystems
        self.superstructure = Superstructure(self.elevator, self.arm, self.climber)

        self.build_autos_speed1()
        #self.build_autos_speed2()

        # Register Named Commands for PathPlanner after initializing subsystems but before the rest of init
        self.register_named_commands()

        # Button bindings must be configured after every subsystem has been set up
        self.configure_button_bindings()

    def build_autos_speed1(self):
        first_path_speed1 = PathPlannerPath.fromPathFile("Start to TR (Speed1)")

        speed_1 = commands2.SequentialCommandGroup(
            commands2.ParallelCommandGroup(
                # Orient robot into starting position and move to TR loading station (left)
                AutoBuilder.pathfindThenFollowPath(first_path_speed1, swerve_config.PATHFINDING_CONSTRAINTS),
                self.level_2_command(),
            ),

            # Set height/rotation to level 4 height/rotation
            self.level_4_command(),

            # Place it
            # self.claw.OuttakeCommand(),

            # Set height/rotation to level 0 height/rotation and travel to loading station
            commands2.ParallelCommandGroup(
                self.level_0_command(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("TR to Loading (Speed1)")),
            ),

            # Accept new piece
            # self.claw.IntakeCommand(),

            # Set height/rotation to level 4 height/rotation and travel to TL loading (left)
            commands2.ParallelCommandGroup(
                self.level_2_command(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Load to TL (Speed1)")),
            ),

            self.level_4_command(),

            # self.claw.OuttakeCommand(),  # Deposit coral


            # Set height/rotation to level 0 height/rotation and travel to loading station
            commands2.ParallelCommandGroup(
                self.level_0_command(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("TL to Load (Speed1)")),
            ),
            # self.claw.IntakeCommand(),  # Receive coral

            # Set height/rotation to level 4 height/rotation and travel to TL loading (right)
            commands2.ParallelCommandGroup(
                self.level_2_command(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Load to TL - R (Speed1)")),
            ),
            self.level_4_command()


            # self.claw.OuttakeCommand(),  # Deposit coral

            # AutoBuilder.followPath(PathPlannerPath.fromPathFile("")),


        )
        self.auto_chooser.setDefaultOption("Speed 1", speed_1)

    def build_autos_speed2(self):
        first_path_speed2 = PathPlannerPath.fromPathFile("start to br")

        speed_2 = commands2.SequentialCommandGroup(
            # Orient robot into starting position
            AutoBuilder.resetOdom(first_path_speed2.getStartingHolonomicPose()),

            # Set height/rotation to level 4 height/rotation and move to br (Bottom Right - Right)
            #commands2.ParallelCommandGroup(
            #    self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_4_HEIGHT, CoralArmConstants.LEVEL_4_ROTATION),
            #    AutoBuilder.followPath(first_path_speed2),
            #),

            # Place it
            # self.claw.OuttakeCommand(),

            # Set height/rotation to level 0 height/rotation and travel to loading station
            commands2.ParallelCommandGroup(
                self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_0_HEIGHT, CoralArmConstants.LEVEL_0_ROTATION),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("br to macdonalds")),
            ),

            # Accept new piece
            # self.claw.IntakeCommand(),

            # Set height/rotation to level 4 height/rotation and travel to BL (left)
            #commands2.ParallelCommandGroup(
            #    self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_4_HEIGHT, CoralArmConstants.LEVEL_4_ROTATION),
            #    AutoBuilder.followPath(PathPlannerPath.fromPathFile("bk to bl")),
            #),
            # self.claw.OuttakeCommand(),  # Deposit coral

            # Set height/rotation to level 0 height/rotation and travel to loading station
            commands2.ParallelCommandGroup(
                self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_0_HEIGHT, CoralArmConstants.LEVEL_0_ROTATION),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("from bl back to bk")),
            ),
            # self.claw.IntakeCommand(),  # Receive coral

            # Set height/rotation to level 4 height/rotation and travel to BL loading station (right)
            #commands2.ParallelCommandGroup(
            #    self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_4_HEIGHT, CoralArmConstants.LEVEL_4_ROTATION),
            #    AutoBuilder.followPath(PathPlannerPath.fromPathFile("bk to bl RIGHT SIDE OF THE CORAL")),
            #),
            # self.claw.OuttakeCommand(),  # Deposit coral

            # AutoBuilder.followPath(PathPlannerPath.fromPathFile("")),

        )
        self.auto_chooser.addOption("Speed 2", speed_2)

    def get_autonomous_command(self) -> Command:
        return self.auto_chooser.getSelected()

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

        # LOADER HEIGHT (LEVEL 0)
        # RIGHT TRIGGER
        self.operator_joystick.loading_level.onTrue(self.level_0_command())


        # LEVEL 1 HEIGHT
        # A BUTTON
        #self.operator_joystick.level_1.whileTrue(
        #    self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_1_HEIGHT, CoralArmConstants.LEVEL_1_ROTATION, False)
        #)
        self.operator_joystick.level_1.onTrue(self.level_1_command())

        # LEVEL 2 HEIGHT
        # X BUTTON
        self.operator_joystick.level_2.onTrue(self.level_2_command())

        # LEVEL 3 HEIGHT
        # B BUTTON
        self.operator_joystick.level_3.onTrue(self.level_3_command())

        # LEVEL 4 HEIGHT
        # Y BUTTON
        #self.operator_joystick.level_4.onTrue(
        #    self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_4_HEIGHT, CoralArmConstants.LEVEL_4_ROTATION, False)
        #)
        self.operator_joystick.level_4.onTrue(self.level_4_command())

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
                AutoBuilder.pathfindToPose(pose, swerve_config.PATHFINDING_CONSTRAINTS) \
                    .andThen(commands2.PrintCommand("Finished"))
                    .andThen(DriveToPoseCommand(self.swerve, pose, AUTONOMOUS_PARAMS)).beforeStarting(
                    commands2.PrintCommand(f"{reef_label} pressed!")
                    )
            )

        # Climber buttons
        self.operator_joystick.climber_up.whileTrue(self.climber.RaiseRobot())
        self.operator_joystick.climber_down.whileTrue(self.climber.LowerRobot())

        # SysId routines

        self.sysid_joystick.y().whileTrue(self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.a().whileTrue(self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
        self.sysid_joystick.b().whileTrue(self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.x().whileTrue(self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kReverse))

        self.sysid_joystick.povUp().whileTrue(self.arm.SysIdQuasistatic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.povDown().whileTrue(self.arm.SysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        self.sysid_joystick.povRight().whileTrue(self.arm.SysIdDynamic(SysIdRoutine.Direction.kForward))
        self.sysid_joystick.povLeft().whileTrue(self.arm.SysIdDynamic(SysIdRoutine.Direction.kReverse))

        # self.joystick.stick.a().onTrue(self.superstructure.SetEndEffectorHeight(2.5, Rotation2d.fromDegrees(-30)))
        # self.driver_joystick.stick.b().whileTrue(DriveToPoseCommand(self.swerve, Pose2d(7, 6, Rotation2d.fromDegrees(30)), AUTONOMOUS_PARAMS))

    def register_named_commands(self):
        pass

    def level_0_command(self):
        return self.elevator.SetHeightCommand(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT).alongWith(
            self.arm.SetAngleCommand(CoralArmConstants.LEVEL_0_ROTATION)
        )

    def level_1_command(self):
        return self.elevator.SetHeightCommand(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT).alongWith(
            self.arm.SetAngleCommand(CoralArmConstants.LEVEL_1_ROTATION)
        )

    def level_2_command(self):
        return self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_2_HEIGHT, CoralArmConstants.LEVEL_2_ROTATION)

    def level_3_command(self):
        return self.superstructure.SetEndEffectorHeight(ElevatorConstants.LEVEL_3_HEIGHT, CoralArmConstants.LEVEL_3_ROTATION)

    def level_4_command(self):
        return self.elevator.SetHeightCommand(ElevatorConstants.LEVEL_4_HEIGHT).alongWith(
            self.arm.SetAngleCommand(CoralArmConstants.LEVEL_4_ROTATION)
        )



def construct_Pose2d(x: float, y: float, degrees: float) -> Pose2d:
    return Pose2d(x, y, Rotation2d.fromDegrees(degrees))
