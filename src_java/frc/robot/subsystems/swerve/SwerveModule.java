package frc.robot.subsystems.swerve;

/*
 * ========================================================================
 * SWERVE MODULE - Individual Wheel Control Unit
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls ONE wheel of the swerve drive. A swerve drive has 4 modules
 * (Front Left, Front Right, Rear Left, Rear Right), and each module
 * can independently:
 *   - Spin the wheel at any speed (DRIVE motor)
 *   - Point the wheel in any direction (AZIMUTH motor)
 *
 * WHAT IS SWERVE DRIVE?
 * --------------------
 * Unlike tank drive (left/right wheels) or mecanum, swerve modules can
 * rotate 360 degrees while spinning. This lets the robot:
 *   - Move in any direction without rotating the chassis
 *   - Rotate while moving in a straight line
 *   - Strafe sideways
 *
 *   TANK DRIVE:              SWERVE DRIVE:
 *   Can only go ↑ ↓          Can go in ANY direction!
 *
 *   ┌─────────┐              ┌─────────┐
 *   │ ║     ║ │              │ ◇     ◇ │  ← Modules can rotate
 *   │ ║     ║ │              │         │
 *   │ ║     ║ │              │ ◇     ◇ │
 *   └─────────┘              └─────────┘
 *
 * HARDWARE IN EACH MODULE:
 * ------------------------
 *   1. DRIVE MOTOR (NEO)
 *      - Spins the wheel forward/backward
 *      - Controls how fast the robot moves
 *
 *   2. AZIMUTH MOTOR (NEO)
 *      - Rotates the entire wheel assembly
 *      - Points the wheel in the desired direction
 *      - Also called "steering" or "turn" motor
 *
 *   3. CANCoder (Absolute Encoder)
 *      - Tells us the wheel's actual angle
 *      - "Absolute" means it remembers position after power off
 *      - Used to reset the relative encoder on startup
 *
 * WHY TWO ENCODERS?
 * -----------------
 * We use the NEO's built-in encoder (relative) for control because it
 * updates faster. But we need the CANCoder (absolute) to know where
 * "zero" is when the robot turns on.
 *
 *   Startup sequence:
 *   1. Read CANCoder to get absolute angle
 *   2. Set relative encoder to match
 *   3. Use relative encoder for all control
 *
 * ENCODER OFFSET:
 * ---------------
 * When you install a CANCoder, it probably won't read 0° when the wheel
 * is pointing forward. The OFFSET corrects for this:
 *
 *   True Forward: 0°
 *   CANCoder reads: 47°
 *   Offset: -47°
 *   After offset: 47° + (-47°) = 0° ✓
 *
 * To calibrate: Point wheel forward manually, read CANCoder value,
 * that's your offset.
 *
 * STATE OPTIMIZATION:
 * -------------------
 * Instead of rotating 180° to go backward, we can reverse the drive
 * motor and only rotate 90°. This is called "optimization."
 *
 *   Without optimization:    With optimization:
 *   Rotate 180°, drive +     Rotate 0°, drive -
 *   ◇ → ↻↻↻ → ◇              ◇ (just reverse motor)
 *
 * HOW TO MODIFY:
 * --------------
 * - Change motor IDs: Edit SwerveConstants (different for each module)
 * - Tune turning PID: SwerveConstants.AZIMUTH_kP, kI, kD
 * - Tune drive PID: SwerveConstants.DRIVE_kP, kI, kD
 * - Change wheel size: SwerveConstants.WHEEL_CIRCUMFERENCE
 * - Recalibrate offsets: Update the offset for each module
 *
 * QUICK REFERENCE:
 * ----------------
 * → Get wheel angle: module.getAngle()
 * → Get wheel speed: module.getVelocity()
 * → Get position: module.getModulePosition()
 * → Set desired state: module.setDesiredState(state, openLoop)
 * → Reset encoder: module.resetToAbsolute()
 *
 * ========================================================================
 */

// CTRE Phoenix 6 - CANCoder absolute encoder
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// REV Robotics SparkMax - Motor controllers and encoders
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

// WPILib math and kinematics
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Our constants
import frc.robot.Constants.SwerveConstants;

/**
 * ========================================================================
 * SWERVE MODULE CLASS
 * ========================================================================
 *
 * Represents a single swerve module with independent drive and steering.
 * Each robot has 4 of these modules.
 *
 * [WHAT IS A MODULE?]
 *
 *        ┌───────┐
 *        │       │
 *   ═════╪═══════╪═════  ← Wheel can spin (drive)
 *        │       │
 *        └───────┘
 *            ↑
 *            │
 *            ╧  ← Entire assembly can rotate (azimuth)
 *
 * The module controls:
 *   - Wheel SPEED (drive motor)
 *   - Wheel DIRECTION (azimuth motor)
 */
public class SwerveModule {

    // ========================================================================
    // IDENTIFICATION
    // ========================================================================

    /**
     * Which module this is (0=FL, 1=FR, 2=RL, 3=RR).
     * Used for logging and debugging.
     */
    private final int moduleNumber;

    // ========================================================================
    // MOTORS - The muscle of the module
    // ========================================================================

    /**
     * DRIVE MOTOR - Controls wheel speed.
     *
     * [PHYSICAL DESCRIPTION]
     * This motor spins the wheel forward/backward. It's usually connected
     * through a gearbox to increase torque.
     *
     * [NEO MOTOR]
     * REV Robotics NEO is a brushless motor with:
     *   - Built-in encoder
     *   - High power density
     *   - Controlled by SparkMAX
     */
    private final SparkMax driveMotor;

    /**
     * AZIMUTH MOTOR - Controls wheel direction.
     *
     * [PHYSICAL DESCRIPTION]
     * This motor rotates the entire wheel assembly. Also called
     * "steering" or "turn" motor.
     *
     * [WHY BRAKE MODE?]
     * We want the wheel to stay pointed where we left it, so we use
     * brake mode (motor resists movement when not powered).
     */
    private final SparkMax azimuthMotor;

    // ========================================================================
    // ENCODERS - Position/velocity feedback
    // ========================================================================

    /**
     * Drive motor's built-in encoder.
     *
     * [WHAT IT MEASURES]
     * - Position: How far the wheel has traveled (meters)
     * - Velocity: How fast the wheel is spinning (m/s)
     *
     * We use conversion factors to convert motor rotations to
     * real-world units (accounts for gear ratio and wheel size).
     */
    private final RelativeEncoder driveEncoder;

    /**
     * Azimuth motor's built-in encoder.
     *
     * [WHAT IT MEASURES]
     * - Position: What angle the wheel is pointing (degrees)
     * - Velocity: How fast the wheel is rotating (deg/s)
     *
     * This is a "relative" encoder - it only tracks changes from
     * where it started. We reset it to the absolute value on startup.
     */
    private final RelativeEncoder azimuthEncoder;

    /**
     * CANCoder - Absolute position sensor on the azimuth.
     *
     * [WHY WE NEED THIS]
     * The NEO's encoder loses its position when powered off.
     * The CANCoder remembers the absolute angle forever.
     *
     * [HOW WE USE IT]
     * On startup, we read the CANCoder and set the NEO encoder
     * to match. Then we use the NEO encoder for control
     * (it updates faster).
     */
    private final CANcoder canCoder;

    // ========================================================================
    // CONTROLLERS - PID control
    // ========================================================================

    /**
     * PID controller for drive motor velocity.
     *
     * [WHAT IT DOES]
     * Takes a target velocity (m/s) and calculates motor output
     * to achieve that speed.
     *
     * Works with feedforward for better performance.
     */
    private final SparkClosedLoopController driveController;

    /**
     * PID controller for azimuth motor position.
     *
     * [WHAT IT DOES]
     * Takes a target angle (degrees) and rotates the wheel
     * to that position.
     *
     * Uses position wrapping (360° = 0°) for continuous rotation.
     */
    private final SparkClosedLoopController azimuthController;

    // ========================================================================
    // FEEDFORWARD - Physics-based control
    // ========================================================================

    /**
     * Feedforward calculator for drive motor.
     *
     * [WHAT IS FEEDFORWARD?]
     * PID alone reacts to error - it needs to see a mistake before
     * correcting. Feedforward predicts what power is needed.
     *
     * [THE CONSTANTS]
     *   - kS: Static friction (minimum power to start moving)
     *   - kV: Velocity constant (power per unit speed)
     *   - kA: Acceleration constant (power per unit acceleration)
     *
     * These are found through SysId characterization or manual tuning.
     */
    private final SimpleMotorFeedforward driveFeedforward;

    // ========================================================================
    // CALIBRATION
    // ========================================================================

    /**
     * Offset to correct CANCoder reading to "true forward."
     *
     * [WHY THIS EXISTS]
     * When the CANCoder is installed, it probably doesn't read 0°
     * when the wheel is pointing forward. This offset corrects that.
     *
     * [HOW TO CALIBRATE]
     * 1. Point the wheel straight forward (manually)
     * 2. Read the CANCoder value
     * 3. That value is your offset
     */
    private final Rotation2d encoderOffset;

    // ========================================================================
    // STATE TRACKING
    // ========================================================================

    /**
     * The last state we commanded (for optimization).
     * Used to track what we last asked for.
     */
    private SwerveModuleState lastState = new SwerveModuleState();

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new SwerveModule.
     *
     * [WHAT HAPPENS AT CREATION]
     * 1. Initialize all motor controllers and sensors
     * 2. Configure motor settings (PID, current limits, etc.)
     * 3. Configure CANCoder
     * 4. Reset azimuth encoder to absolute position
     *
     * @param moduleNumber The module number (0=FL, 1=FR, 2=RL, 3=RR)
     * @param driveMotorId CAN ID of the drive motor SparkMAX
     * @param azimuthMotorId CAN ID of the azimuth motor SparkMAX
     * @param canCoderId CAN ID of the CANCoder (absolute encoder)
     * @param encoderOffset Offset to make CANCoder read 0° when forward (degrees)
     */
    public SwerveModule(int moduleNumber, int driveMotorId, int azimuthMotorId,
                        int canCoderId, double encoderOffset) {
        this.moduleNumber = moduleNumber;
        this.encoderOffset = Rotation2d.fromDegrees(encoderOffset);

        // ----------------------------------------------------------------
        // DRIVE MOTOR SETUP
        // ----------------------------------------------------------------
        // Create SparkMAX, kBrushless because NEO is a brushless motor
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);

        // Get the built-in encoder and PID controller
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();

        // ----------------------------------------------------------------
        // AZIMUTH MOTOR SETUP
        // ----------------------------------------------------------------
        azimuthMotor = new SparkMax(azimuthMotorId, MotorType.kBrushless);
        azimuthEncoder = azimuthMotor.getEncoder();
        azimuthController = azimuthMotor.getClosedLoopController();

        // ----------------------------------------------------------------
        // CANCODER SETUP
        // ----------------------------------------------------------------
        canCoder = new CANcoder(canCoderId);

        // ----------------------------------------------------------------
        // FEEDFORWARD SETUP
        // ----------------------------------------------------------------
        // SimpleMotorFeedforward predicts motor power needed for a speed
        // Constants come from SysId characterization or manual tuning
        driveFeedforward = new SimpleMotorFeedforward(
            SwerveConstants.DRIVE_kS,  // Static friction (voltage to overcome friction)
            SwerveConstants.DRIVE_kV,  // Velocity constant (voltage per m/s)
            SwerveConstants.DRIVE_kA   // Acceleration constant (voltage per m/s²)
        );

        // ----------------------------------------------------------------
        // FINAL SETUP
        // ----------------------------------------------------------------
        configureMotors();     // Set up motor parameters (PID, current limits)
        configureCANCoder();   // Set up absolute encoder
        resetToAbsolute();     // Sync relative encoder to absolute position
    }

    // ========================================================================
    // CONFIGURATION METHODS
    // ========================================================================

    /**
     * Configure motor controllers with appropriate settings.
     *
     * [WHAT THIS DOES]
     * Sets up both motors with:
     *   - Current limits (protect motors from overheating)
     *   - Ramp rates (smooth acceleration)
     *   - PID gains (control accuracy)
     *   - Encoder conversion factors (real-world units)
     *
     * [CONVERSION FACTORS EXPLAINED]
     * The NEO encoder counts motor rotations. We need real units:
     *
     *   Drive: rotations → meters traveled
     *   Formula: circumference / gear_ratio
     *   If gear ratio is 6.75:1 and wheel is 0.1m circumference:
     *     1 motor rotation = 0.1 / 6.75 = 0.0148m
     *
     *   Azimuth: rotations → degrees rotated
     *   Formula: 360 / gear_ratio
     *   If gear ratio is 21.43:1:
     *     1 motor rotation = 360 / 21.43 = 16.8 degrees
     */
    private void configureMotors() {
        // ================================================================
        // DRIVE MOTOR CONFIGURATION
        // ================================================================
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
            .idleMode(IdleMode.kCoast)  // Coast when not powered (easier to push)
            .smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT)  // Prevent motor damage
            .openLoopRampRate(SwerveConstants.DRIVE_OPEN_LOOP_RAMP)  // Smooth teleop
            .closedLoopRampRate(SwerveConstants.DRIVE_CLOSED_LOOP_RAMP);  // Smooth auto

        // ----------------------------------------------------------------
        // ENCODER CONVERSION: Motor rotations → Meters
        // ----------------------------------------------------------------
        // Position: How far the robot has traveled (meters)
        double drivePositionFactor = SwerveConstants.WHEEL_CIRCUMFERENCE / SwerveConstants.DRIVE_GEAR_RATIO;

        // Velocity: How fast the robot is moving (meters/second)
        // NEO reports RPM, so divide by 60 to get per-second
        double driveVelocityFactor = drivePositionFactor / 60.0;

        driveConfig.encoder
            .positionConversionFactor(drivePositionFactor)   // Now getPosition() returns meters
            .velocityConversionFactor(driveVelocityFactor);  // Now getVelocity() returns m/s

        // ----------------------------------------------------------------
        // PID GAINS for velocity control
        // ----------------------------------------------------------------
        driveConfig.closedLoop
            .p(SwerveConstants.DRIVE_kP)  // Proportional: main correction
            .i(SwerveConstants.DRIVE_kI)  // Integral: steady-state error (usually 0)
            .d(SwerveConstants.DRIVE_kD); // Derivative: damping (reduces overshoot)

        // Apply configuration to motor
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ================================================================
        // AZIMUTH MOTOR CONFIGURATION
        // ================================================================
        SparkMaxConfig azimuthConfig = new SparkMaxConfig();

        azimuthConfig
            .idleMode(IdleMode.kBrake)  // BRAKE mode: hold position when not powered
            .smartCurrentLimit(SwerveConstants.AZIMUTH_CURRENT_LIMIT)
            .inverted(true);  // May need to flip depending on gearbox setup

        // ----------------------------------------------------------------
        // ENCODER CONVERSION: Motor rotations → Degrees
        // ----------------------------------------------------------------
        double azimuthPositionFactor = 360.0 / SwerveConstants.AZIMUTH_GEAR_RATIO;
        double azimuthVelocityFactor = azimuthPositionFactor / 60.0;

        azimuthConfig.encoder
            .positionConversionFactor(azimuthPositionFactor)   // getPosition() returns degrees
            .velocityConversionFactor(azimuthVelocityFactor);  // getVelocity() returns deg/s

        // ----------------------------------------------------------------
        // PID GAINS with POSITION WRAPPING
        // ----------------------------------------------------------------
        // Position wrapping makes 0° and 360° the same point
        // This is essential for continuous rotation!
        // Without it, going from 350° to 10° would go the long way (340° rotation)
        // With it, we go the short way (20° rotation)
        azimuthConfig.closedLoop
            .p(SwerveConstants.AZIMUTH_kP)
            .i(SwerveConstants.AZIMUTH_kI)
            .d(SwerveConstants.AZIMUTH_kD)
            .positionWrappingEnabled(true)   // Enable wraparound
            .positionWrappingMinInput(0)     // Minimum is 0°
            .positionWrappingMaxInput(360);  // Maximum is 360° (wraps to 0°)

        azimuthMotor.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configure the CANCoder for absolute position sensing.
     *
     * [WHAT THIS DOES]
     * Sets the CANCoder to report positive values for counter-clockwise rotation.
     * This matches the robot coordinate system (CCW positive).
     */
    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // CCW positive matches standard math convention
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoder.getConfigurator().apply(config);
    }

    /**
     * Reset the azimuth encoder to the absolute position from CANCoder.
     *
     * [WHEN TO CALL]
     * - Automatically called in constructor
     * - Can call manually if encoder gets out of sync
     *
     * [WHAT THIS DOES]
     * Reads the CANCoder's absolute angle and sets the NEO encoder
     * to match. Now both encoders agree on the current position.
     */
    public void resetToAbsolute() {
        // Get the absolute angle (already offset-corrected)
        double absolutePosition = getAbsoluteAngle().getDegrees();

        // Set the relative encoder to match
        azimuthEncoder.setPosition(absolutePosition);
    }

    // ========================================================================
    // GETTER METHODS - Read current state
    // ========================================================================

    /**
     * Get the absolute angle from the CANCoder, accounting for offset.
     *
     * [WHEN TO USE THIS]
     * Only use this for resetting the relative encoder.
     * For normal operation, use getAngle() (relative encoder is faster).
     *
     * [OFFSET EXPLAINED]
     * The raw CANCoder reading minus the offset gives "true forward = 0°"
     *
     * @return The absolute angle as a Rotation2d (0° = forward)
     */
    public Rotation2d getAbsoluteAngle() {
        // CANCoder returns 0-1 rotations, multiply by 360 for degrees
        double angle = canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;

        // Subtract offset to get corrected angle
        return Rotation2d.fromDegrees(angle).minus(encoderOffset);
    }

    /**
     * Get the current angle of the module from the integrated encoder.
     *
     * [THIS IS THE MAIN ANGLE GETTER]
     * Use this for all normal operations. It's faster than CANCoder
     * and is already synced to the correct value.
     *
     * @return The current angle as a Rotation2d (0° = forward)
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(azimuthEncoder.getPosition());
    }

    /**
     * Get the current velocity of the drive motor.
     *
     * @return The velocity in meters per second (positive = forward)
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Get the current position of the drive motor.
     *
     * [WHAT THIS RETURNS]
     * Total distance the wheel has traveled since startup (or last reset).
     * Used for odometry to track robot position on the field.
     *
     * @return The position in meters
     */
    public double getPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Get the current state of the swerve module.
     *
     * [WHAT IS A STATE?]
     * SwerveModuleState contains:
     *   - speedMetersPerSecond: How fast the wheel is spinning
     *   - angle: Which direction the wheel is pointing
     *
     * Think of it as a "snapshot" of what the module is doing right now.
     *
     * @return The current SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Get the current position of the swerve module.
     *
     * [WHAT IS A POSITION?]
     * SwerveModulePosition contains:
     *   - distanceMeters: How far the wheel has traveled (total)
     *   - angle: Which direction the wheel is pointing
     *
     * Used by odometry to calculate where the robot is on the field.
     * The difference from STATE is that position tracks TOTAL distance,
     * while state tracks current SPEED.
     *
     * @return The current SwerveModulePosition
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    // ========================================================================
    // CONTROL METHODS - Set desired state
    // ========================================================================

    /**
     * Set the desired state of the swerve module.
     *
     * [THIS IS THE MAIN CONTROL METHOD]
     * Called by SwerveDrive to tell each module what to do.
     *
     * [OPEN LOOP vs CLOSED LOOP]
     *
     *   OPEN LOOP (openLoop = true):
     *   - Just sets motor power as a percentage
     *   - Power = desiredSpeed / maxSpeed
     *   - Used in teleop (feels more responsive)
     *
     *   CLOSED LOOP (openLoop = false):
     *   - Uses PID + feedforward for precise velocity
     *   - Measures actual speed and adjusts
     *   - Used in autonomous (more accurate)
     *
     * [STATE OPTIMIZATION]
     * The optimize() call is KEY for swerve efficiency.
     * If we need to go backward, instead of:
     *   - Rotating 180° and driving forward
     * We do:
     *   - Keep pointing same direction, drive backward
     *
     * This means we NEVER rotate more than 90°!
     *
     * @param desiredState The desired state (speed and angle)
     * @param openLoop True for teleop (percentage control), false for auto (PID control)
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {

        // ================================================================
        // OPTIMIZATION - Smart angle selection
        // ================================================================
        // This is the magic that makes swerve efficient!
        // If target is more than 90° away, flip the angle and negate speed
        //
        // Example:
        //   Current: 0°, Target: 170°
        //   Without optimization: rotate 170°
        //   With optimization: rotate -10°, drive backward
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        // ================================================================
        // AZIMUTH CONTROL - Point the wheel
        // ================================================================
        // Tell the PID controller what angle we want
        // Position wrapping handles the 0°/360° boundary
        azimuthController.setReference(
            desiredState.angle.getDegrees(),
            SparkMax.ControlType.kPosition  // Position control mode
        );

        // ================================================================
        // DRIVE CONTROL - Spin the wheel
        // ================================================================
        if (openLoop) {
            // --------------------------------------------------------
            // OPEN LOOP - Simple percentage output
            // --------------------------------------------------------
            // Convert desired speed to a -1 to 1 percentage
            // If max speed is 4 m/s and we want 2 m/s: 2/4 = 0.5 = 50% power
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            driveMotor.set(percentOutput);

        } else {
            // --------------------------------------------------------
            // CLOSED LOOP - PID with feedforward
            // --------------------------------------------------------
            // Calculate feedforward: how much power we EXPECT to need
            double feedforward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

            // Use PID to get to exact velocity, with feedforward as base power
            driveController.setReference(
                desiredState.speedMetersPerSecond,    // Target velocity
                SparkMax.ControlType.kVelocity,       // Velocity control mode
                0,                                     // PID slot 0
                feedforward                            // Add feedforward to output
            );
        }

        // Remember what we commanded (for debugging/optimization)
        lastState = desiredState;
    }

    /**
     * Stop the module by setting both motors to 0.
     *
     * [WHEN TO USE]
     * Emergency stop or when robot is disabled.
     * For normal stopping, set desired state to 0 velocity instead.
     */
    public void stop() {
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    /**
     * Get the module number.
     *
     * @return The module number (0=FL, 1=FR, 2=RL, 3=RR)
     */
    public int getModuleNumber() {
        return moduleNumber;
    }

}  // End of SwerveModule class
