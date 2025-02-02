import math

import commands2
import rev
import wpilib
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from wpimath.system.plant import DCMotor, LinearSystemId

from constants import ElevatorConstants


class Elevator(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # Initialize SPARK MAX
        self.motor = rev.SparkMax(ElevatorConstants.LEFT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()
        self.encoder = self.motor.getEncoder()
        self.config()

        # Magnetic limit switch for lower limit and homing
        self.limit_switch = wpilib.DigitalInput(0)

        # Setup mechanism and gearbox for simulation
        gearbox = DCMotor.NEO(2)
        self.motor_sim = rev.SparkMaxSim(self.motor, gearbox)
        plant = LinearSystemId.elevatorSystem(gearbox, 9, 0.8125 / 39.37, 5.45)
        self.elevator_sim = ElevatorSim(plant, gearbox, 0, 3, True, 0)

        # Visual display of the elevator
        mech = wpilib.Mechanism2d(3, 4)
        root = mech.getRoot("robot", 2, 0)
        self.elevator = root.appendLigament(
            "elevator", 0.1524, 90
        )

        wpilib.SmartDashboard.putData("Mech", mech)

    def simulationPeriodic(self) -> None:
        self.elevator_sim.setInput(self.motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage())

        self.elevator_sim.update(0.02)

        self.motor_sim.iterate(
            self.elevator_sim.getVelocity() * (1.625 / 39.37) * math.pi / 60,  # Convert from m/s to rotations per min
            RoboRioSim.getVInVoltage(),
            0.02
        )

        RoboRioSim.setVInVoltage(BatterySim.calculate(self.elevator_sim.getCurrentDraw()))

        self.elevator.setLength(0.1524 + self.elevator_sim.getPosition())

    def config(self):
        motor_config = rev.SparkBaseConfig()

        motor_config.encoder \
            .positionConversionFactor(1) \
            .velocityConversionFactor(1)

        motor_config.closedLoop \
            .pid(0.4, 0, 0) \
            .outputRange(-1, 1)

        motor_config.closedLoop.maxMotion \
            .maxVelocity(1000) \
            .maxAcceleration(1000) \
            .allowedClosedLoopError(1)  # Affected by position conversion factor

        self.motor.configure(
            motor_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters,
        )

    def set_height(self, height: float):
        # Use MAXMotion for smoother position control
        self.controller.setReference(height, rev.SparkBase.ControlType.kMAXMotionPositionControl)

    def set_duty_cycle(self, output: float):
        self.motor.set(output)

    def lower_limit(self) -> bool:
        return self.limit_switch.get()

