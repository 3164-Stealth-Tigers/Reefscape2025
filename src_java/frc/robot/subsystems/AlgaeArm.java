package frc.robot.subsystems;

/*
 * ========================================================================
 * ALGAE ARM SUBSYSTEM - Secondary Game Piece Manipulation
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls a secondary arm mechanism for handling ALGAE game pieces.
 * In REEFSCAPE, ALGAE is a secondary game piece type (in addition to CORAL).
 *
 * PHYSICAL DESCRIPTION:
 * --------------------
 * A single motor arm that can rotate to different positions.
 * Simpler than the CoralArm - just basic position control.
 *
 *       ───○─── Arm
 *          │
 *          │ Motor
 *        ┌─┴─┐
 *        │   │ Robot
 *
 * ABSOLUTE ENCODER:
 * ----------------
 * Uses an absolute encoder (built into SparkFlex) to always know
 * the exact position even after power cycling.
 *
 * HOW TO MODIFY:
 * --------------
 * - Change motor ID: AlgaeArmConstants.MOTOR_ID
 * - Tune position control: AlgaeArmConstants.kP
 * - Change gear ratio: AlgaeArmConstants.GEAR_RATIO
 *
 * QUICK REFERENCE:
 * ----------------
 * → Set angle: algaeArm.setAngle(angle) or algaeArm.setAngleCommand(angle)
 * → Get angle: algaeArm.getAngle()
 * → Manual control: algaeArm.setDutyCycle(power)
 *
 * ========================================================================
 */

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;

/**
 * ========================================================================
 * ALGAE ARM SUBSYSTEM
 * ========================================================================
 *
 * Simple arm for secondary game piece manipulation.
 * Uses position control with an absolute encoder.
 *
 * [SIMPLER THAN CORAL ARM]
 * This arm is simpler - no feedforward, no motion profiling.
 * Just basic PID position control to move to desired angles.
 */
public class AlgaeArm extends SubsystemBase {

    // ========================================================================
    // HARDWARE
    // ========================================================================

    /**
     * SparkFlex motor controller with NEO Vortex motor.
     * SparkFlex has a built-in absolute encoder option.
     */
    private final SparkFlex motor;

    /**
     * Absolute encoder for position feedback.
     * Remembers position even after power off.
     */
    private final AbsoluteEncoder absoluteEncoder;

    /**
     * PID controller for position control.
     * Runs on the motor controller (not the roboRIO).
     */
    private final SparkClosedLoopController controller;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new AlgaeArm subsystem.
     * Initializes motor, encoder, and controller.
     */
    public AlgaeArm() {
        // Initialize motor with CAN ID from Constants
        motor = new SparkFlex(AlgaeArmConstants.MOTOR_ID, MotorType.kBrushless);

        // Get built-in absolute encoder and PID controller
        absoluteEncoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Apply motor configuration
        configureMotor();
    }

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    /**
     * Configure the motor with appropriate settings.
     *
     * [WHAT WE CONFIGURE]
     * - Brake mode (hold position when stopped)
     * - Encoder conversion factors (motor rotations to degrees)
     * - PID gains for position control
     */
    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        // Brake mode - hold arm in place when not powered
        config.idleMode(IdleMode.kBrake);

        // ================================================================
        // ABSOLUTE ENCODER CONFIGURATION
        // ================================================================
        // The absolute encoder reads the arm position directly
        config.absoluteEncoder
            .positionConversionFactor(360)           // Output in degrees (0-360)
            .velocityConversionFactor(360.0 / 60.0)  // Output in deg/s
            .inverted(true);                          // Flip direction if needed

        // ================================================================
        // RELATIVE ENCODER CONFIGURATION
        // ================================================================
        // Also configure relative encoder (may be used for velocity)
        config.encoder
            .positionConversionFactor(360.0 / AlgaeArmConstants.GEAR_RATIO)     // Account for gearing
            .velocityConversionFactor(360.0 / AlgaeArmConstants.GEAR_RATIO / 60.0);

        // ================================================================
        // CLOSED-LOOP (PID) CONFIGURATION
        // ================================================================
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)  // Use absolute encoder
            .p(AlgaeArmConstants.kP)  // Proportional gain
            .i(0)                      // No integral (keeps things simple)
            .d(0)                      // No derivative
            .outputRange(-1, 1);       // Full power range

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================

    @Override
    public void periodic() {
        // Log current angle for debugging
        SmartDashboard.putNumber("AlgaeArm/Angle", getAngle().getDegrees());
    }

    // ========================================================================
    // CONTROL METHODS
    // ========================================================================

    /**
     * Set the arm to a specific angle using position control.
     *
     * [ANGLE CONVENTION]
     * 0 degrees = horizontal, facing toward front of robot
     * Positive = counter-clockwise (looking from the right side)
     *
     * @param angle The target angle
     */
    public void setAngle(Rotation2d angle) {
        controller.setReference(angle.getDegrees(), ControlType.kPosition);
    }

    /**
     * Get the current angle of the arm.
     *
     * @return The current angle as a Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
    }

    /**
     * Set the motor duty cycle directly (manual control).
     *
     * [WHEN TO USE]
     * For testing or manual override. Bypasses position control.
     *
     * @param output The duty cycle (-1 to 1)
     */
    public void setDutyCycle(double output) {
        motor.set(output);
    }

    // ========================================================================
    // COMMANDS
    // ========================================================================

    /**
     * Command to set the arm to a specific angle.
     *
     * [NOTE]
     * This is a "runOnce" command - it sets the target angle and finishes.
     * The motor controller's PID will continue to maintain the position.
     *
     * @param angle The target angle
     * @return A command that sets the arm angle (finishes immediately)
     */
    public Command setAngleCommand(Rotation2d angle) {
        return Commands.runOnce(() -> setAngle(angle), this)
            .withName("SetAlgaeAngle(" + angle.getDegrees() + ")");
    }

}  // End of AlgaeArm class
