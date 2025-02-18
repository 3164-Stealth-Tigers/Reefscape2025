import math

import commands2
import rev
import wpilib
import wpimath.controller
from wpilib import RobotController
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpiutil import SendableBuilder, Sendable

from constants import ElevatorConstants
from sim_helper import SimHelper


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

        self.feedforward = wpimath.controller.ElevatorFeedforward(*ElevatorConstants.FEEDFORWARD_CONSTANTS)

        self.config()

        # Setup mechanism and gearbox for simulation
        gearbox = DCMotor.NEO(2)
        self.motor_sim = rev.SparkMaxSim(self.leader, gearbox)
        plant = LinearSystemId.elevatorSystem(
            gearbox,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.PULLEY_DIAMETER / 2,
            ElevatorConstants.GEAR_RATIO,
        )
        self.elevator_sim = ElevatorSim(
            plant,
            gearbox,
            ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT,
            ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT,
            True,
            0,
        )

        # Visual display of the elevator
        mech = wpilib.Mechanism2d(3, 4)
        root = mech.getRoot("elevator", 2, 0)
        self.elevator = root.appendLigament(
            "carriage", self.elevator_sim.getPosition(), 90
        )

        wpilib.SmartDashboard.putData("Elevator Mechanism", mech)

    def periodic(self) -> None:
        self.elevator.setLength(self.carriage_height())

    def simulationPeriodic(self) -> None:
        self.elevator_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage())

        self.elevator_sim.update(0.02)

        self.motor_sim.iterate(
            self.elevator_sim.getVelocity(),
            RoboRioSim.getVInVoltage(),
            0.02
        )

        SimHelper.add_simulated_current_load(RobotController.getTime(), self.elevator_sim.getCurrentDraw())

    def config(self):
        global_config = rev.SparkBaseConfig()

        global_config.encoder \
            .positionConversionFactor(ElevatorConstants.PULLEY_DIAMETER * math.pi / ElevatorConstants.GEAR_RATIO) \
            .velocityConversionFactor(ElevatorConstants.PULLEY_DIAMETER * math.pi / ElevatorConstants.GEAR_RATIO / 60)

        global_config.closedLoop \
            .pid(10, 0, 0) \
            .outputRange(-1, 1)

        global_config.closedLoop.maxMotion \
            .maxVelocity(1) \
            .maxAcceleration(10) \
            .allowedClosedLoopError(0.01)  # Affected by position conversion factor

        leader_config = rev.SparkBaseConfig()
        leader_config \
            .apply(global_config) \
            .inverted(ElevatorConstants.INVERT_LEFT_MOTOR)

        follower_config = rev.SparkBaseConfig()
        follower_config \
            .apply(global_config) \
            .inverted(ElevatorConstants.INVERT_RIGHT_MOTOR)

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

    def set_height(self, height: float):
        ff = self.feedforward.calculate(self.encoder.getPosition(), self.encoder.getVelocity())
        self.controller.setReference(
            height,
            rev.SparkBase.ControlType.kPosition,  # rev.SparkBase.ControlType.kMAXMotionPositionControl
            arbFeedforward=ff,
            arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

    def set_duty_cycle(self, output: float):
        self.leader.set(output)

    def carriage_height(self) -> float:
        return self.encoder.getPosition()

    def lower_limit(self) -> bool:
        return self.limit_switch.get()

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty("Height", self.carriage_height, lambda _: None)
