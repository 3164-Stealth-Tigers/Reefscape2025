import commands2
import rev
import wpilib
import wpimath.controller
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine
from wpilib import RobotController
from wpilib.simulation import SingleJointedArmSim, RoboRioSim
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor, LinearSystemId
from wpiutil import SendableBuilder

from constants import CoralArmConstants
from sim_helper import SimHelper


class CoralArm(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Coral Arm")

        # Setup motors
        self.motor = rev.SparkFlex(CoralArmConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.absolute_encoder = self.motor.getAbsoluteEncoder()  # REV Through Bore Encoder
        self.controller = self.motor.getClosedLoopController()

        self.feedforward = wpimath.controller.ArmFeedforward(*CoralArmConstants.FEEDFORWARD_CONSTANTS)

        # Use an encoder offset on the real robot, but don't offset the encoder in simulation
        self.encoder_offset = CoralArmConstants.ENCODER_OFFSET if wpilib.RobotBase.isReal() else 0

        self.config()

        # Setup mechanism and gearbox for simulation
        self.absolute_encoder_sim = rev.SparkAbsoluteEncoderSim(self.motor)
        gearbox = DCMotor.neoVortex(1)
        self.motor_sim = rev.SparkFlexSim(self.motor, gearbox)
        moment = SingleJointedArmSim.estimateMOI(CoralArmConstants.ARM_LENGTH, CoralArmConstants.ARM_MASS)
        plant = LinearSystemId.singleJointedArmSystem(gearbox, moment, CoralArmConstants.GEAR_RATIO)
        self.arm_sim = SingleJointedArmSim(
            plant,
            gearbox,
            CoralArmConstants.ARM_MASS,
            CoralArmConstants.ARM_MASS,
            CoralArmConstants.MINIMUM_ANGLE.radians(),
            CoralArmConstants.MAXIMUM_ANGLE.radians(),
            True,
            0,
        )

        # Visual display of the arm
        mech = wpilib.Mechanism2d(5, 5)
        root = mech.getRoot("armPivot", 1, 2.5)
        self.arm = root.appendLigament("arm", 3, self.arm_sim.getAngleDegrees())

        # Create a new SysId routine for characterizing the arm
        self.sysId_routine = SysIdRoutine(
            SysIdRoutine.Config(0.25, 3),
            SysIdRoutine.Mechanism(
                self.set_voltage,
                self.sysId_log,
                self,
            ),
        )

        # Reset NEO encoder to absolute encoder
        self.motor.getEncoder().setPosition(
            self.absolute_encoder.getPosition()
        )

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
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) \

        motor_config.absoluteEncoder \
            .positionConversionFactor(360) \
            .velocityConversionFactor(360 / 60)

        motor_config.encoder \
            .positionConversionFactor(360 / CoralArmConstants.GEAR_RATIO) \
            .velocityConversionFactor(360 / CoralArmConstants.GEAR_RATIO / 60)

        motor_config.closedLoop \
            .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) \
            .pid(CoralArmConstants.kP, 0, 0) \
            .outputRange(-1, 1) \

        motor_config.softLimit \
            .forwardSoftLimit(CoralArmConstants.MAXIMUM_ANGLE.degrees() + self.encoder_offset) \
            .reverseSoftLimit(CoralArmConstants.MINIMUM_ANGLE.degrees() + self.encoder_offset) \
            .forwardSoftLimitEnabled(True) \
            .reverseSoftLimitEnabled(True) \

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
        ff = self.feedforward.calculate(self.angle().degrees(), self.absolute_encoder.getVelocity())
        self.controller.setReference(
            angle.degrees() + self.encoder_offset,
            rev.SparkBase.ControlType.kPosition,
            arbFeedforward=ff,
            arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

    def set_duty_cycle(self, output: float):
        self.motor.set(output)

    def at_rotation(self, goal_rotation: Rotation2d) -> bool:
        return (goal_rotation - CoralArmConstants.ARM_TOLERANCE).radians() < self.angle().radians() < (goal_rotation + CoralArmConstants.ARM_TOLERANCE).radians()

    def set_voltage(self, volts: float):
        self.controller.setReference(volts, rev.SparkBase.ControlType.kVoltage)

    def angle(self) -> Rotation2d:
        degrees = self.absolute_encoder.getPosition() - self.encoder_offset
        return Rotation2d.fromDegrees(degrees)

    def sysId_log(self, log: SysIdRoutineLog):
        log.motor("arm-pivot") \
            .voltage(self.motor.getAppliedOutput() * self.motor.getBusVoltage()) \
            .angularPosition(self.angle().degrees()) \
            .angularVelocity(self.absolute_encoder.getVelocity())

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addStringProperty("Command", self.current_command_name, lambda _: None)
        builder.addDoubleProperty("Angle", lambda: self.angle().degrees(), lambda degrees: self.set_angle(Rotation2d.fromDegrees(degrees)))

    def current_command_name(self) -> str:
        try:
            return self.getCurrentCommand().getName()
        except AttributeError:
            return ""

    def SetAngleCommand(self, angle: Rotation2d):
        return commands2.RunCommand(lambda: self.set_angle(angle), self) \
            .until(lambda: self.at_rotation(angle))

    def SysIdQuasistatic(self, direction: SysIdRoutine.Direction):
        return self.sysId_routine.quasistatic(direction)

    def SysIdDynamic(self, direction: SysIdRoutine.Direction):
        return self.sysId_routine.dynamic(direction)
