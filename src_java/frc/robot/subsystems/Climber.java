package frc.robot.subsystems;

/*
 * ========================================================================
 * CLIMBER SUBSYSTEM - End-Game Climbing Mechanism
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls the climber mechanism for end-game climbing. In REEFSCAPE,
 * robots can earn points by climbing structures at the end of the match.
 *
 * PHYSICAL DESCRIPTION:
 * --------------------
 * The climber typically consists of:
 *   - Two motors (leader/follower configuration)
 *   - A hook or arm that extends upward
 *   - A winch system to pull the robot up
 *
 *   CLIMBING SEQUENCE:
 *   ┌─────────┐        ┌─────────┐        ┌─────────┐
 *   │         │        │    ↑    │        │ HANGING │
 *   │  ROBOT  │  →     │  ROBOT  │  →     │ ↓ROBOT↓ │
 *   │  ═══╤   │        │  ═══╤───┤        │  ═══╤───┤
 *   └─────────┘        └─────────┘        └─────╧───┘
 *    Approach          Hook extends       Winch pulls up
 *
 * LEADER/FOLLOWER:
 * ----------------
 * Two motors work together - one "leads" (receives commands) and one
 * "follows" (copies the leader). This is more efficient than controlling
 * both motors separately.
 *
 * SOFT LIMITS:
 * -----------
 * We set software limits to prevent the climber from going too far
 * in either direction. This protects the mechanism from damage.
 *
 * HOW TO MODIFY:
 * --------------
 * - Change motor IDs: ClimberConstants.LEADER_MOTOR_ID, FOLLOWER_MOTOR_ID
 * - Change soft limits: FORWARD_LIMIT_DEGREES, BACKWARD_LIMIT_DEGREES
 * - Change speed: Modify the values in raiseRobotCommand/lowerRobotCommand
 *
 * QUICK REFERENCE:
 * ----------------
 * → Extend climber: climber.raiseRobotCommand()
 * → Retract climber: climber.lowerRobotCommand()
 * → Manual control: climber.moveClimber(power)
 * → Get position: climber.getAngle()
 *
 * ========================================================================
 */

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
import frc.robot.Constants.ClimberConstants;

/**
 * ========================================================================
 * CLIMBER SUBSYSTEM
 * ========================================================================
 *
 * Controls the end-game climbing mechanism using two motors
 * in leader/follower configuration.
 *
 * [TIMING]
 * End-game climbing typically happens in the last 20 seconds of the match.
 * Make sure your strategy allows enough time!
 */
public class Climber extends SubsystemBase {

    // ========================================================================
    // HARDWARE - Motors
    // ========================================================================

    /**
     * Leader motor - receives all commands.
     * SparkFlex is REV's newer motor controller for NEO Vortex motors.
     */
    private final SparkFlex leaderMotor;

    /**
     * Follower motor - automatically copies the leader.
     *
     * [WHY FOLLOWER?]
     * Climbing requires a lot of force, so we use two motors.
     * Instead of controlling both, we just tell one to "follow" the other.
     * This simplifies code and ensures perfect synchronization.
     */
    private final SparkFlex followerMotor;

    // ========================================================================
    // SENSORS - Position feedback
    // ========================================================================

    /**
     * Absolute encoder for tracking climber position.
     * Useful for knowing if we're extended, retracted, or in between.
     */
    private final AbsoluteEncoder encoder;

    // ========================================================================
    // CONSTANTS - Soft limits and configuration
    // ========================================================================

    /**
     * Offset to correct the encoder's zero position.
     * Adjust this so 0° corresponds to "fully retracted" or "starting position".
     */
    private static final double ZERO_OFFSET = 0.0;

    /**
     * Maximum forward (extend) position in degrees.
     * The climber cannot go past this point.
     */
    private static final double FORWARD_LIMIT_DEGREES = 360.0;

    /**
     * Minimum reverse (retract) position in degrees.
     * The climber cannot go below this point.
     */
    private static final double BACKWARD_LIMIT_DEGREES = 0.0;

    /**
     * Whether soft limits are active.
     * Set to false for testing, but keep true for competition!
     */
    private static final boolean LIMITS_ENABLED = true;

    /**
     * Whether to invert the follower motor.
     * If motors are mounted facing opposite directions, set to true.
     */
    private static final boolean INVERT_FOLLOWER = false;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new Climber subsystem.
     * Initializes both motors and configures leader/follower relationship.
     */
    public Climber() {
        // Initialize motors with CAN IDs from Constants
        // Note: Follower is created first - order shouldn't matter but be aware
        followerMotor = new SparkFlex(ClimberConstants.FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        leaderMotor = new SparkFlex(ClimberConstants.LEADER_MOTOR_ID, MotorType.kBrushless);

        // Get encoder from leader motor
        encoder = leaderMotor.getAbsoluteEncoder();

        // Apply all motor configurations
        configureMotors();
    }

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    /**
     * Configure both motors with appropriate settings.
     *
     * [LEADER CONFIGURATION]
     * - Brake mode (hold position when stopped - important for climbing!)
     * - Current limit (50A - prevent motor damage)
     * - Soft limits (prevent over-extension)
     * - Absolute encoder setup
     *
     * [FOLLOWER CONFIGURATION]
     * - Brake mode
     * - Current limit
     * - Follow mode (just copy the leader)
     */
    private void configureMotors() {
        // ================================================================
        // LEADER MOTOR CONFIGURATION
        // ================================================================
        SparkFlexConfig leaderConfig = new SparkFlexConfig();

        leaderConfig
            .idleMode(IdleMode.kBrake)      // BRAKE - crucial for holding robot weight!
            .smartCurrentLimit(50)           // 50A current limit
            .inverted(true);                 // Reverse direction if needed

        // Configure the absolute encoder
        leaderConfig.absoluteEncoder
            .positionConversionFactor(360)           // Output in degrees
            .velocityConversionFactor(360.0 / 60.0)  // Output in deg/s
            .zeroOffset(ZERO_OFFSET)                  // Calibration offset
            .inverted(false);

        // Use absolute encoder for feedback
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // SOFT LIMITS - prevent mechanical damage
        // The motor controller will refuse to go past these positions
        leaderConfig.softLimit
            .forwardSoftLimit(FORWARD_LIMIT_DEGREES)         // Max extend
            .reverseSoftLimit(BACKWARD_LIMIT_DEGREES)        // Max retract
            .forwardSoftLimitEnabled(LIMITS_ENABLED)         // Enable forward limit
            .reverseSoftLimitEnabled(LIMITS_ENABLED);        // Enable reverse limit

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ================================================================
        // FOLLOWER MOTOR CONFIGURATION
        // ================================================================
        SparkFlexConfig followerConfig = new SparkFlexConfig();

        followerConfig
            .idleMode(IdleMode.kBrake)      // Also brake - both motors hold
            .smartCurrentLimit(50)           // Same current limit
            .follow(ClimberConstants.LEADER_MOTOR_ID, INVERT_FOLLOWER);  // Follow the leader!

        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ========================================================================
    // PERIODIC - Runs every robot loop
    // ========================================================================

    @Override
    public void periodic() {
        // Log current position for debugging
        SmartDashboard.putNumber("Climber/Angle", getAngle().getDegrees());
    }

    // ========================================================================
    // CONTROL METHODS
    // ========================================================================

    /**
     * Move the climber at a specified power.
     *
     * [NOTE]
     * Only the leader motor is controlled directly. The follower automatically
     * copies whatever the leader does.
     *
     * @param power The power to apply (-1 to 1). Positive = extend, negative = retract.
     */
    public void moveClimber(double power) {
        leaderMotor.set(power);  // Follower automatically follows
    }

    /**
     * Stop the climber motors.
     * Motors will brake and hold position (brake mode).
     */
    public void stop() {
        leaderMotor.set(0);
    }

    /**
     * Get the current angle/position of the climber.
     *
     * @return The current angle as a Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    // ========================================================================
    // COMMANDS
    // ========================================================================

    /**
     * Command to raise the robot (extend climber / pull up).
     *
     * [USAGE]
     * Bind to a button that the operator holds during climbing.
     * Stops automatically when button is released.
     *
     * @return A command that runs climber at full power forward
     */
    public Command raiseRobotCommand() {
        return Commands.startEnd(
            () -> moveClimber(1),   // On start: run at full power
            this::stop,              // On end: stop
            this                     // Requires this subsystem
        ).withName("Raise Robot");
    }

    /**
     * Command to lower the robot (retract climber / let down).
     *
     * [USAGE]
     * Use if you need to lower after extending, or reset the climber.
     * Hold button to lower, releases when button released.
     *
     * @return A command that runs climber at full power backward
     */
    public Command lowerRobotCommand() {
        return Commands.startEnd(
            () -> moveClimber(-1),  // On start: run at full power reverse
            this::stop,              // On end: stop
            this                     // Requires this subsystem
        ).withName("Lower Robot");
    }

}  // End of Climber class
