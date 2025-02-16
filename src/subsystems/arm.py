import math

import commands2
import rev
import wpilib
import wpimath.controller
from wpilib import RobotController
from wpilib.simulation import SingleJointedArmSim, RoboRioSim
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor, LinearSystemId

from constants import ArmConstants
from sim_helper import SimHelper


class Arm(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Arm")

        # Setup motors
        self.motor = rev.SparkFlex(ArmConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.absolute_encoder = self.motor.getAbsoluteEncoder()  # REV Through Bore Encoder
        self.controller = self.motor.getClosedLoopController()

        self.feedforward = wpimath.controller.ArmFeedforward(*ArmConstants.FEEDFORWARD_CONSTANTS)

        self.config()

        # Setup mechanism and gearbox for simulation
        self.absolute_encoder_sim = rev.SparkAbsoluteEncoderSim(self.motor)
        gearbox = DCMotor.neoVortex(1)
        self.motor_sim = rev.SparkFlexSim(self.motor, gearbox)
        plant = LinearSystemId.singleJointedArmSystem(gearbox, SingleJointedArmSim.estimateMOI(0.3048, 5), 200)
        self.arm_sim = SingleJointedArmSim(plant, gearbox, 200, 0.3048, -1 * math.pi / 3, math.pi / 3, True, 0)

        # Visual display of the arm
        mech = wpilib.Mechanism2d(5, 5)
        root = mech.getRoot("armPivot", 1, 2.5)
        self.arm = root.appendLigament("arm", 3, self.arm_sim.getAngleDegrees())

        wpilib.SmartDashboard.putData("Arm Mechanism", mech)

    def simulationPeriodic(self) -> None:
        self.arm_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage())

        self.arm_sim.update(0.02)

        self.motor_sim.iterate(
            self.arm_sim.getVelocityDps(),
            RoboRioSim.getVInVoltage(),
            0.02
        )

        self.absolute_encoder_sim.iterate(self.arm_sim.getVelocityDps(), 0.02)

        SimHelper.add_simulated_current_load(RobotController.getTime(), self.arm_sim.getCurrentDraw())

        self.arm.setAngle(self.arm_sim.getAngleDegrees())

    def config(self):
        motor_config = rev.SparkBaseConfig()

        motor_config \
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        motor_config.absoluteEncoder \
            .positionConversionFactor(360) \
            .velocityConversionFactor(360 / 60)

        motor_config.encoder \
            .positionConversionFactor(360 / 200) \
            .velocityConversionFactor(360 / 200 / 60)

        motor_config.closedLoop \
            .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) \
            .pid(0.2, 0, 0) \
            .outputRange(-1, 1) \

        self.motor.configure(
            motor_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

    def set_angle(self, angle: Rotation2d):
        """
        Command the arm to move to a specific angle. 0 degrees is considered horizontal, facing toward
        the front of the robot.

        :param angle: Counter-clockwise positive angle where 0 degrees is horizontal, facing the front of the robot.
        """
        ff = self.feedforward.calculate(self.absolute_encoder.getPosition(), self.absolute_encoder.getVelocity())
        self.controller.setReference(
            angle.degrees(),
            rev.SparkBase.ControlType.kPosition,
            arbFeedforward=ff,
            arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

    def angle(self) -> Rotation2d:
        degrees = self.absolute_encoder.getPosition()
        return Rotation2d.fromDegrees(degrees)