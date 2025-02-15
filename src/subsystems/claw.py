import commands2
import rev
import playingwithfusion as pwf

from constants import ClawConstants


class Claw(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Claw")

        # NEO 550 motor to spin intake wheels
        self.motor = rev.SparkMax(ClawConstants.MOTOR_ID, rev.SparkMax.MotorType.kBrushless)

        # Range sensor to detect coral possession
        self.distance_sensor = pwf.TimeOfFlight(ClawConstants.ToF_SENSOR_ID)

    def intake(self):
        """Run the intake motors at a constant power, pulling CORAL into the claw."""

    def outtake(self):
        """Run the intake motors at a constant power, pushing CORAL out of the claw."""

    def stop(self):
        """Stop running the intake motors."""

    def has_possession(self) -> bool:
        """Return whether the claw is currently holding CORAL."""
        # If the distance reported by the sensor is less than a certain known distance,
        # a CORAL is sitting above the sensor; therefore, the claw has possession
        return self.distance_sensor.getRange() < ClawConstants.ToF_MIN_DISTANCE

        # If the intake motors are stalled, the claw has possession of a game piece
        # return self.motor.getOutputCurrent() > (ClawConstants.CURRENT_LIMIT_AMPS - 2)
