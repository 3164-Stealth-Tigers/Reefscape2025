import math

import commands2
import rev
import playingwithfusion as pwf
import wpilib
from commands2 import Command
from wpilib import SmartDashboard
from wpiutil import SendableBuilder

from constants import ClawConstants, ClimberConstants
from subsystems.climber import Climber


class Claw(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Claw")

        # NEO 550 motor to spin intake wheels
        self.motor = rev.SparkMax(ClawConstants.MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        motor_config = rev.SparkBaseConfig()
        motor_config \
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) \
            .smartCurrentLimit(20) \
            .inverted(True)
        motor_config.encoder \
            .positionConversionFactor(360) \
            .velocityConversionFactor(360 / 60)
        self.motor.configure(
            motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

        # Range sensor to detect coral possession
        self.distance_sensor = pwf.TimeOfFlight(ClawConstants.ToF_SENSOR_ID)
        self.distance_sensor.setRangeOfInterest(8, 8, 12, 12)
        self.distance_sensor.setRangingMode(pwf.TimeOfFlight.RangingMode.kShort, 500)

        self.timer = wpilib.Timer()
        self.has_piece_sim = False

    def intake(self):
        """Run the intake motors at a constant power, pulling CORAL into the claw."""
        self.motor.set(0.2)

    def outtake(self):
        """Run the intake motors at a constant power, pushing CORAL out of the claw."""
        self.motor.set(-0.3)

    def stop(self):
        """Stop running the intake motors."""
        self.motor.set(0)

    def has_possession(self) -> bool:
        """Return whether the claw is currently holding CORAL."""
        if wpilib.RobotBase.isReal():
            motor_current = self.motor.getOutputCurrent()
            motor_speed = abs(self.encoder.getVelocity())
            time_since_spinup = self.timer.get()

            has_piece = (motor_current > ClawConstants.THRESHOLD_CURRENT) and (motor_speed < 500.0) and (time_since_spinup > 0.5)
        else:
            has_piece = self.has_piece_sim

        return has_piece

       #print(self.distance_sensor.getStatus().name)
        # return self.distance_sensor.getStatus().name in ("Status.kValid", "Status.???")


        # If the intake motors are stalled, the claw has possession of a game piece
         #return self.motor.getOutputCurrent() > (ClawConstants.CURRENT_LIMIT_AMPS - 2)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addBooleanProperty("Has Possession?", self.has_possession, lambda _: setattr(self, "has_piece_sim", not self.has_piece_sim))
        builder.addDoubleProperty("Motor Current", self.motor.getOutputCurrent, lambda _: None)
        builder.addDoubleProperty("Wheel Velocity", self.encoder.getVelocity, lambda _: None)
        builder.addDoubleProperty("Time Since Spinup", self.timer.get, lambda _: None)

    def IntakeCommand(self):
        """Pulls the CORAL into the claw. This command will not end on its own; it must be interrupted by the user."""
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.timer.restart, self),
            commands2.RunCommand(self.intake, self),
            commands2.InstantCommand(self.stop, self),
        )

    def OuttakeCommand(self):
        """Ejects the CORAL from the claw. This command ends after the CORAL has been ejected."""
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.timer.restart, self),
            commands2.RunCommand(self.outtake, self),
            commands2.InstantCommand(self.stop, self),
        )
