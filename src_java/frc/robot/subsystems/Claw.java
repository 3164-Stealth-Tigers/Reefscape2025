package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * Claw subsystem for intake and outtake of CORAL game pieces.
 * Uses a NEO 550 motor and a Time of Flight sensor for piece detection.
 */
public class Claw extends SubsystemBase {
    // Motor and encoder
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    // Range sensor for piece detection
    private final TimeOfFlight distanceSensor;

    // Timer for spinup delay
    private final Timer timer = new Timer();

    // Simulation flag for piece possession
    private boolean hasPieceSim = false;

    /**
     * Creates a new Claw subsystem.
     */
    public Claw() {
        // Initialize motor
        motor = new SparkMax(ClawConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .inverted(true);
        config.encoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360.0 / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize distance sensor
        distanceSensor = new TimeOfFlight(ClawConstants.TOF_SENSOR_ID);
        distanceSensor.setRangeOfInterest(8, 8, 12, 12);
        distanceSensor.setRangingMode(RangingMode.Short, 500);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw/HasPossession", hasPossession());
        SmartDashboard.putNumber("Claw/MotorCurrent", motor.getOutputCurrent());
        SmartDashboard.putNumber("Claw/WheelVelocity", encoder.getVelocity());
        SmartDashboard.putNumber("Claw/TimeSinceSpinup", timer.get());
    }

    /**
     * Run the intake motors to pull CORAL into the claw.
     */
    public void intake() {
        motor.set(0.2);
    }

    /**
     * Run the intake motors to push CORAL out of the claw.
     */
    public void outtake() {
        motor.set(-0.3);
    }

    /**
     * Stop the intake motors.
     */
    public void stop() {
        motor.set(0);
    }

    /**
     * Check if the claw is currently holding a CORAL piece.
     * Detection is based on motor current and wheel stall.
     *
     * @return True if holding a piece
     */
    public boolean hasPossession() {
        if (RobotBase.isReal()) {
            double motorCurrent = motor.getOutputCurrent();
            double motorSpeed = Math.abs(encoder.getVelocity());
            double timeSinceSpinup = timer.get();

            return (motorCurrent > ClawConstants.THRESHOLD_CURRENT)
                && (motorSpeed < 500.0)
                && (timeSinceSpinup > 0.5);
        } else {
            return hasPieceSim;
        }
    }

    /**
     * Toggle the simulated piece possession (for testing).
     */
    public void toggleSimPossession() {
        hasPieceSim = !hasPieceSim;
    }

    // ==================== Commands ====================

    /**
     * Command to intake CORAL.
     * This command will not end on its own; it must be interrupted.
     *
     * @return The intake command
     */
    public Command intakeCommand() {
        return Commands.sequence(
            Commands.runOnce(timer::restart, this),
            Commands.run(this::intake, this),
            Commands.runOnce(this::stop, this)
        ).withName("Intake");
    }

    /**
     * Command to outtake CORAL.
     * This command will not end on its own; it must be interrupted.
     *
     * @return The outtake command
     */
    public Command outtakeCommand() {
        return Commands.sequence(
            Commands.runOnce(timer::restart, this),
            Commands.run(this::outtake, this),
            Commands.runOnce(this::stop, this)
        ).withName("Outtake");
    }
}
