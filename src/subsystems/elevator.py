import math
from typing import Optional, Set

import commands2
import rev
import wpilib
import wpimath.controller
from commands2 import TrapezoidProfileCommand, Subsystem
from rev import ClosedLoopSlot
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine
from wpilib import RobotController
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim, DIOSim
from wpimath._controls._controls.trajectory import TrapezoidProfile
from wpimath.system.plant import DCMotor, LinearSystemId
from wpiutil import SendableBuilder, Sendable

from constants import ElevatorConstants, CoralArmConstants
from helpers import SimHelper


class Elevator(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Elevator")

        # Initialize primary SPARK MAX
        self.leader = rev.SparkMax(ElevatorConstants.LEFT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        self.controller = self.leader.getClosedLoopController()
        self.encoder = self.leader.getEncoder()

        # Initialize follower SPARK MAX
        self.follower = rev.SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)

        # Magnetic limit switch for lower limit and homing
        self.limit_switch = wpilib.DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_ID)
        self.limit_switch_sim = DIOSim(self.limit_switch)

        self.feedforward = wpimath.controller.ElevatorFeedforward(*ElevatorConstants.FEEDFORWARD_CONSTANTS)

        self.config()
        self.reset()

        # Setup mechanism and gearbox for simulation
        gearbox = DCMotor.NEO(2)
        self.motor_sim = rev.SparkMaxSim(self.leader, gearbox)
        plant = LinearSystemId.elevatorSystem(
            gearbox,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.SPROCKET_PITCH_DIAMETER / 2,
            ElevatorConstants.GEAR_RATIO,
        )
        self.elevator_sim = ElevatorSim(
            plant,
            gearbox,
            ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT,
            ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT,
            True,
            ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT,
        )

        # Visual display of the elevator
        mech = wpilib.Mechanism2d(3, 4)
        root = mech.getRoot("elevator", 2, 0)
        self.elevator = root.appendLigament(
            "carriage", self.elevator_sim.getPosition(), 90
        )

        # Create a new SysId routine for characterizing the arm
        self.sysId_routine = SysIdRoutine(
            SysIdRoutine.Config(0.3, 2),
            SysIdRoutine.Mechanism(
                self.set_voltage,
                self.sysId_log,
                self,
            ),
        )

        wpilib.SmartDashboard.putData("Elevator Mechanism", mech)

        # Goal height that the elevator will try to reach
        self.goal_height: Optional[float] = None
        self.goal_velocity: Optional[float] = None

    def periodic(self) -> None:
        # Update SmartDashboard visualization
        self.elevator.setLength(self.carriage_height())

    def simulationPeriodic(self) -> None:
        self.elevator_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage())

        self.elevator_sim.update(0.02)

        self.motor_sim.iterate(
            self.elevator_sim.getVelocity() * 2,
            RoboRioSim.getVInVoltage(),
            0.02
        )

        SimHelper.add_simulated_current_load(RobotController.getTime(), self.elevator_sim.getCurrentDraw())

    def config(self):
        global_config = rev.SparkBaseConfig()

        global_config \
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) \
            .smartCurrentLimit(60)

        # 2.376 zero rot. height
        # 4.043 1 rot. height

        global_config.encoder \
            .positionConversionFactor(2 * (1 / ElevatorConstants.GEAR_RATIO) * (ElevatorConstants.SPROCKET_PITCH_DIAMETER * math.pi)) \
            .velocityConversionFactor(2 * (1 / ElevatorConstants.GEAR_RATIO) * (ElevatorConstants.SPROCKET_PITCH_DIAMETER * math.pi) * (1 / 60))

        global_config.closedLoop \
            .pid(ElevatorConstants.POSITION_kP, 0, ElevatorConstants.POSITION_kD, ClosedLoopSlot.kSlot0) \
            .outputRange(-0.5, ElevatorConstants.MAX_OUT_UP, ClosedLoopSlot.kSlot0)

        global_config.closedLoop \
            .pid(ElevatorConstants.VELOCITY_kP, 0, 0, ClosedLoopSlot.kSlot1) \
            .outputRange(ElevatorConstants.MAX_OUT_DOWN, ElevatorConstants.MAX_OUT_UP, ClosedLoopSlot.kSlot1)

        leader_config = rev.SparkBaseConfig()
        leader_config \
            .apply(global_config) \
            .inverted(ElevatorConstants.INVERT_LEFT_MOTOR)

        '''
        Comment out the maxMotion below if you use this
        NEED ElevatorConstants for MIN_VELOCITY << start with MAX_VELOCITY /2 maybe
        '''
        leader_config.closedLoop.smartMotion \
            .maxVelocity(ElevatorConstants.MAX_VELOCITY, rev.ClosedLoopSlot.kSlot1) \
            .minOutputVelocity(ElevatorConstants.MAX_VELOCITY/2, rev.ClosedLoopSlot.kSlot1) \
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION, rev.ClosedLoopSlot.kSlot1) \
            .allowedClosedLoopError(ElevatorConstants.HEIGHT_TOLERANCE, rev.ClosedLoopSlot.kSlot1)

        '''
        leader_config.closedLoop.maxMotion \
            .maxVelocity(ElevatorConstants.MAX_VELOCITY) \
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION) \
            .allowedClosedLoopError(ElevatorConstants.HEIGHT_TOLERANCE)  # Affected by position conversion factor
        '''

        leader_config.softLimit \
            .forwardSoftLimit(ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT) \
            .reverseSoftLimit(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT) \
            .forwardSoftLimitEnabled(True) \
            .reverseSoftLimitEnabled(True)

        follower_config = rev.SparkBaseConfig()
        follower_config \
            .apply(global_config) \
            .follow(ElevatorConstants.LEFT_MOTOR_ID, ElevatorConstants.INVERT_RIGHT_MOTOR)

        self.leader.configure(
            leader_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )
        self.follower.configure(
            follower_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

    def reset(self, height = ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT):
        self.encoder.setPosition(height)

    def set_height(self, height: float):
        #self.goal_height = height
        self.controller.setReference(
            height,
            rev.SparkBase.ControlType.kPosition,  # rev.SparkBase.ControlType.kMAXMotionPositionControl
            rev.ClosedLoopSlot.kSlot0,
        )

    def set_velocity(self, velocity: float):
        self.goal_velocity = velocity
        self.controller.setReference(
            velocity,
            rev.SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot1,
            1 / 473,
        )

    def set_duty_cycle(self, output: float):
        self.leader.set(output)

    def set_voltage(self, volts: float):
        self.controller.setReference(volts, rev.SparkBase.ControlType.kVoltage)

    def enable_soft_limits(self, enable: bool):
        motor_config = rev.SparkBaseConfig()
        motor_config.softLimit.forwardSoftLimitEnabled(enable)
        motor_config.softLimit.reverseSoftLimitEnabled(enable)
        # Configure the motor without resetting parameters to defaults. We only want to change the soft limit enable.
        self.leader.configure(
            motor_config, rev.SparkBase.ResetMode.kNoResetSafeParameters, rev.SparkBase.PersistMode.kNoPersistParameters
        )

    def carriage_height(self) -> float:
        return self.encoder.getPosition()

    def carriage_height_inches(self) -> float:
        return self.carriage_height() * 39.37

    def lower_limit(self) -> bool:
        """Return True if the lower limit switch is triggered."""
        # Hall Effect sensor returns False when magnet is detected.
        return not self.limit_switch.get()

    def at_height(self, height: float) -> bool:
        return height - ElevatorConstants.HEIGHT_TOLERANCE < self.carriage_height() < height + ElevatorConstants.HEIGHT_TOLERANCE

    def at_goal_height(self) -> bool:
        return self.at_height(self.goal_height) if self.goal_height is not None else False

    def sysId_log(self, log: SysIdRoutineLog):
        log.motor("elevator") \
            .voltage(self.leader.getAppliedOutput() * self.leader.getBusVoltage()) \
            .position(self.encoder.getPosition()) \
            .velocity(self.encoder.getVelocity())

    def velocity_error(self):
        if self.goal_velocity is not None:
            error = self.goal_velocity - self.encoder.getVelocity()
        else:
            error = 0
        return error

    def height_error(self):
        if self.goal_height is not None:
            error = self.goal_height - self.carriage_height()
        else:
            error = 0
        return error

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty("Height", self.carriage_height, lambda _: None)
        builder.addDoubleProperty("Velocity", self.encoder.getVelocity, lambda _: None)
        builder.addBooleanProperty("Limit Switch Triggered", self.lower_limit, lambda value: self.limit_switch_sim.setValue(not value))
        builder.addDoubleProperty("Applied Voltage", lambda: self.leader.getAppliedOutput() * self.leader.getBusVoltage(), lambda _: None)
        builder.addStringProperty("Command", self.current_command_name, lambda _: None)
        builder.addDoubleProperty("hError", self.height_error, lambda _: None)
        builder.addDoubleProperty("Goal Height", lambda: self.goal_height if self.goal_height is not None else 0, lambda _: None)
        builder.addDoubleProperty("vError", self.velocity_error, lambda _: None)
        builder.addDoubleProperty("Goal Velocity", lambda: self.goal_velocity if self.goal_velocity is not None else 0, lambda _: None)
        builder.addBooleanProperty("At Goal", self.at_goal_height, lambda _: False)

    def current_command_name(self) -> str:
        try:
            return self.getCurrentCommand().getName()
        except AttributeError:
            return ""

    def SysIdQuasistatic(self, direction: SysIdRoutine.Direction):
        return self.sysId_routine.quasistatic(direction)

    def SysIdDynamic(self, direction: SysIdRoutine.Direction):
        return self.sysId_routine.dynamic(direction)

    def HomeElevator(self):
        command = commands2.SequentialCommandGroup(
            # Disable soft limits
            commands2.InstantCommand(lambda: self.enable_soft_limits(False)),
            # If the limit switch is already triggered, move the carriage up, away from the switch
            commands2.RunCommand(lambda: self.set_voltage(1.5)).onlyWhile(self.lower_limit),
            commands2.WaitCommand(0.2),
            # Move the carriage down until it triggers the limit switch
            commands2.RunCommand(lambda: self.set_voltage(-1)).until(self.lower_limit),
            # Reset the zero position
            commands2.InstantCommand(lambda: self.reset(ElevatorConstants.LIMIT_SWITCH_HEIGHT)),
            # Stop moving the motors
            commands2.InstantCommand(lambda: self.set_voltage(0)),
            # Re-enable soft limits
            commands2.InstantCommand(lambda: self.enable_soft_limits(True)),
        )
        command.setName("Home Elevator")
        command.addRequirements(self)
        return command

    def HomeElevatorWithHardLimit(self):
        return commands2.StartEndCommand(
            lambda: self.set_voltage(-1),
            lambda: self.set_voltage(0),
            self,
        ) \
        .beforeStarting(commands2.InstantCommand(lambda: self.enable_soft_limits(False))) \
        .andThen(commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.reset),
            commands2.InstantCommand(lambda: self.enable_soft_limits(True)),
        ))


class SetProfiledHeightCommand(commands2.Command):
    def __init__(self, height: float, elevator: Elevator):
        super().__init__()

        self.setName("ProfiledHeightCommand")
        self.goal = TrapezoidProfile.State(height)
        self.elevator = elevator
        self.profile = TrapezoidProfile(TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION))
        self.timer = wpilib.Timer()

    def get_state(self):
        return TrapezoidProfile.State(
            self.elevator.carriage_height(),
            self.elevator.encoder.getVelocity(),
        )

    def initialize(self):
        self.setpoint = self.get_state()
        self.elevator.goal_height = self.goal.position
        self.timer.restart()

    def execute(self):
        self.elevator.set_height(
            self.setpoint.position
        )
        self.setpoint = self.profile.calculate(0.02, self.setpoint, self.goal)

    def end(self, interrupted: bool):
        self.timer.stop()

    def isFinished(self) -> bool:
        return self.profile.isFinished(self.timer.get()) and self.elevator.at_goal_height()

    def getRequirements(self) -> Set[Subsystem]:
        return {self.elevator}


def SetHeightCommand(height: float, elevator: Elevator):
    return SetProfiledHeightCommand(height, elevator).andThen(
        commands2.RunCommand(lambda: elevator.set_height(height), elevator) \
            .until(elevator.at_goal_height)
    )
