package frc.robot.subsystems;

/*
 * ========================================================================
 * CORAL ARM SUBSYSTEM - Arm Rotation for Scoring
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls the "arm" mechanism that rotates the claw/intake to different
 * angles for scoring CORAL at different levels on the REEF.
 *
 * Think of it like your arm - the elevator is your shoulder (moves up/down),
 * and this arm rotates at the "elbow" to angle the "hand" (claw) correctly.
 *
 * PHYSICAL HARDWARE:
 * ------------------
 *   - NEO Vortex motor (CAN ID 12)
 *     └─ NEO Vortex is a newer, more powerful brushless motor from REV
 *     └─ Uses SparkFlex controller (not SparkMax)
 *   - Built-in absolute encoder on SparkFlex
 *     └─ Knows exact angle even after power cycle!
 *
 * WHY ABSOLUTE ENCODER?
 * ---------------------
 * A relative encoder only counts rotations from startup - it doesn't know
 * the actual angle when powered on. An ABSOLUTE encoder always knows the
 * real angle, so we don't need to "home" the arm like we do the elevator.
 *
 * ANGLE CONVENTION:
 * -----------------
 *   - 0° = Horizontal, pointing forward (toward front of robot)
 *   - Positive angles = Counter-clockwise (arm rotates up)
 *   - Negative angles = Clockwise (arm rotates down)
 *
 *   Side view of robot:
 *
 *        +angle (up)
 *             ↑
 *             │
 *    ─────────┼─────────→ 0° (horizontal)
 *             │
 *             ↓
 *        -angle (down)
 *
 * KEY CONCEPTS:
 * -------------
 *
 * 1. ENCODER OFFSET - Calibration value
 *    The physical encoder might read 45° when the arm is actually at 0°.
 *    The offset corrects for this: actual_angle = encoder_reading - offset
 *
 * 2. ARM FEEDFORWARD - Compensates for gravity
 *    Unlike an elevator (constant gravity), an arm experiences different
 *    gravity depending on angle. Horizontal arm = max gravity effect.
 *    Vertical arm = no gravity effect on rotation.
 *
 *    The kG term in feedforward accounts for this using cos(angle).
 *
 * HOW TO MODIFY:
 * --------------
 * - Change angles: Constants.java → CoralArmConstants section
 * - Tune PID: Constants.java → CoralArmConstants.kP
 * - Adjust limits: CoralArmConstants.MINIMUM_ANGLE / MAXIMUM_ANGLE
 *
 * QUICK REFERENCE:
 * ----------------
 * → Set arm angle: coralArm.setAngle(Rotation2d.fromDegrees(-35))
 * → Get current angle: coralArm.getAngle()
 * → Check if at target: coralArm.atGoalRotation()
 *
 * ========================================================================
 */

// REV Robotics SparkFlex motor controller imports
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

// WPILib imports
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralArmConstants;

// Unit types for SysId
import static edu.wpi.first.units.Units.*;

/**
 * ========================================================================
 * CORAL ARM SUBSYSTEM
 * ========================================================================
 *
 * Controls the rotating arm mechanism that positions the claw at the
 * correct angle for scoring. Works with the Elevator to reach all
 * scoring positions (L1-L4) on the reef.
 *
 * Uses a NEO Vortex motor with a SparkFlex controller and built-in
 * absolute encoder for precise angle control.
 *
 * @see Constants.CoralArmConstants for tunable values
 * @see Elevator for the vertical positioning subsystem
 */
public class CoralArm extends SubsystemBase {

    // ========================================================================
    // HARDWARE - Motor and encoder
    // ========================================================================

    /**
     * SparkFlex motor controller with NEO Vortex motor.
     *
     * [DIFFERENCE FROM SPARKMAX]
     * SparkFlex is designed for the NEO Vortex motor (more powerful than NEO).
     * It has a built-in absolute encoder port, which we use for precise positioning.
     */
    private final SparkFlex motor;

    /**
     * Absolute encoder - Knows the exact arm angle even after power off.
     *
     * [WHY ABSOLUTE?]
     * Unlike the elevator's relative encoder (counts from zero each startup),
     * this encoder always knows the real angle. No homing needed!
     */
    private final AbsoluteEncoder absoluteEncoder;

    /**
     * PID controller running on the SparkFlex (1kHz updates).
     */
    private final SparkClosedLoopController controller;

    // ========================================================================
    // CONTROL - Feedforward and calibration
    // ========================================================================

    /**
     * Arm feedforward - Predicts power needed based on physics.
     *
     * [ARM vs ELEVATOR FEEDFORWARD]
     * Elevator: kG is constant (gravity is always the same)
     * Arm: kG varies with cos(angle) because gravity's effect depends on arm angle
     *
     *   Horizontal arm:     Vertical arm:
     *   Maximum gravity     Zero gravity effect
     *   effect on rotation  on rotation
     *
     *        ────○         │
     *            │        ─○─
     *            ↓         │
     *         Gravity
     */
    private final ArmFeedforward feedforward;

    /**
     * Encoder offset - Calibration value for the real robot.
     *
     * When the arm is physically at 0°, the encoder might read some other value.
     * This offset corrects for that misalignment.
     *
     * Set to 0 in simulation (no physical encoder to offset).
     */
    private final double encoderOffset;

    // ========================================================================
    // VISUALIZATION - Dashboard display
    // ========================================================================

    /**
     * Mechanism2d for visualizing arm position in Shuffleboard.
     */
    private final Mechanism2d mechanism;
    private final MechanismLigament2d armLigament;

    /**
     * SysId routine for motor characterization.
     * Used to find optimal feedforward values.
     */
    private final SysIdRoutine sysIdRoutine;

    // ========================================================================
    // STATE - Current goal
    // ========================================================================

    /**
     * Target angle we're trying to reach.
     * null = no target set
     */
    private Rotation2d goalRotation = null;

    // ========================================================================
    // CONSTRUCTOR - Initialize hardware and control systems
    // ========================================================================

    /**
     * Creates a new CoralArm subsystem.
     *
     * Sets up:
     *   1. Motor and encoder
     *   2. Feedforward calculator
     *   3. Motor configuration (PID, limits)
     *   4. Dashboard visualization
     *   5. SysId for tuning
     */
    public CoralArm() {
        // ----------------------------------------------------------------
        // STEP 1: Create motor and get encoder/controller references
        // ----------------------------------------------------------------
        // SparkFlex for NEO Vortex motor
        motor = new SparkFlex(CoralArmConstants.MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = motor.getAbsoluteEncoder();  // Built-in absolute encoder
        controller = motor.getClosedLoopController();

        // ----------------------------------------------------------------
        // STEP 2: Create feedforward calculator
        // ----------------------------------------------------------------
        // ArmFeedforward uses different physics than ElevatorFeedforward
        //   kS = Static friction
        //   kG = Gravity (varies with cos(angle) for arms!)
        //   kV = Velocity coefficient
        //   kA = Acceleration coefficient
        feedforward = new ArmFeedforward(
            CoralArmConstants.FEEDFORWARD_CONSTANTS[0],  // kS
            CoralArmConstants.FEEDFORWARD_CONSTANTS[1],  // kG
            CoralArmConstants.FEEDFORWARD_CONSTANTS[2],  // kV
            CoralArmConstants.FEEDFORWARD_CONSTANTS[3]   // kA
        );

        // ----------------------------------------------------------------
        // STEP 3: Set encoder offset
        // ----------------------------------------------------------------
        // On the real robot, we need to offset the encoder reading
        // In simulation, there's no physical encoder, so offset = 0
        encoderOffset = RobotBase.isReal() ? CoralArmConstants.ENCODER_OFFSET : 0;

        // ----------------------------------------------------------------
        // STEP 4: Configure motor settings
        // ----------------------------------------------------------------
        configureMotor();

        // Sync the relative encoder (inside motor) with absolute encoder
        // This helps when using relative encoder for velocity readings
        motor.getEncoder().setPosition(absoluteEncoder.getPosition());

        // ----------------------------------------------------------------
        // STEP 5: Setup dashboard visualization
        // ----------------------------------------------------------------
        // Mechanism2d shows the arm rotating in Shuffleboard
        mechanism = new Mechanism2d(5, 5);  // Canvas size
        MechanismRoot2d root = mechanism.getRoot("armPivot", 1, 2.5);  // Pivot point
        armLigament = root.append(new MechanismLigament2d("arm", 3, getAngle().getDegrees()));
        SmartDashboard.putData("Arm Mechanism", mechanism);

        // ----------------------------------------------------------------
        // STEP 6: Setup SysId for motor characterization
        // ----------------------------------------------------------------
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds.of(1)),  // Slow ramp for arms (safer)
                Volts.of(3),                        // Lower voltage for safety
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                voltage -> setVoltage(voltage.in(Volts)),
                log -> {
                    // Log angular position/velocity (not linear like elevator)
                    log.motor("arm-pivot")
                        .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
                        .angularPosition(Degrees.of(getAngle().getDegrees()))
                        .angularVelocity(DegreesPerSecond.of(absoluteEncoder.getVelocity()));
                },
                this
            )
        );
    }

    // ========================================================================
    // MOTOR CONFIGURATION
    // ========================================================================

    /**
     * Configure the motor with PID, encoder conversion, and soft limits.
     */
    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        // Brake mode so arm doesn't fall when power is cut
        config.idleMode(IdleMode.kBrake);

        // ----------------------------------------------------------------
        // ABSOLUTE ENCODER CONFIGURATION
        // ----------------------------------------------------------------
        // The absolute encoder rotates once per arm rotation (no gearing)
        // Convert: rotations → degrees
        config.absoluteEncoder
            .positionConversionFactor(360)              // 1 rotation = 360°
            .velocityConversionFactor(360.0 / 60.0);    // RPM → degrees/second

        // ----------------------------------------------------------------
        // RELATIVE ENCODER CONFIGURATION
        // ----------------------------------------------------------------
        // The motor encoder is geared down, so we need to account for that
        // Motor rotates GEAR_RATIO times for 1 arm rotation
        config.encoder
            .positionConversionFactor(360.0 / CoralArmConstants.GEAR_RATIO)
            .velocityConversionFactor(360.0 / CoralArmConstants.GEAR_RATIO / 60.0);

        // ----------------------------------------------------------------
        // PID CONFIGURATION
        // ----------------------------------------------------------------
        // Use the absolute encoder for feedback (most accurate for position)
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)  // Use absolute encoder
            .p(CoralArmConstants.kP)  // Proportional gain
            .i(0)                      // No integral (not needed for arm)
            .d(0)                      // No derivative (can add if needed)
            .outputRange(-1, 1);       // Full power range

        // ----------------------------------------------------------------
        // SOFT LIMITS
        // ----------------------------------------------------------------
        // Prevent the arm from rotating past safe angles
        // NOTE: We add encoderOffset because the motor sees raw encoder values
        config.softLimit
            .forwardSoftLimit(CoralArmConstants.MAXIMUM_ANGLE.getDegrees() + encoderOffset)
            .reverseSoftLimit(CoralArmConstants.MINIMUM_ANGLE.getDegrees() + encoderOffset)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        // Apply configuration to motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ========================================================================
    // PERIODIC - Runs every loop
    // ========================================================================

    @Override
    public void periodic() {
        // Update visualization
        armLigament.setAngle(getAngle().getDegrees());

        // Log data to SmartDashboard
        SmartDashboard.putNumber("CoralArm/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("CoralArm/Velocity", absoluteEncoder.getVelocity());
        SmartDashboard.putBoolean("CoralArm/AtGoal", atGoalRotation());
        SmartDashboard.putString("CoralArm/Command", getCurrentCommandName());

        if (goalRotation != null) {
            SmartDashboard.putNumber("CoralArm/GoalAngle", goalRotation.getDegrees());
        }
    }

    // ========================================================================
    // SETTER METHODS - Control the arm
    // ========================================================================

    /**
     * Set the arm to a specific angle.
     *
     * @param angle The target angle as Rotation2d
     *              0° = horizontal (forward)
     *              Positive = counter-clockwise (up)
     *              Negative = clockwise (down)
     *
     * EXAMPLE:
     *   setAngle(Rotation2d.fromDegrees(-35));  // Angle down for scoring
     *   setAngle(new Rotation2d());             // Horizontal (0°)
     */
    public void setAngle(Rotation2d angle) {
        goalRotation = angle;
        // Add offset because motor controller uses raw encoder values
        controller.setReference(
            angle.getDegrees() + encoderOffset,
            ControlType.kPosition
        );
    }

    /**
     * Set the motor duty cycle directly (open-loop control).
     *
     * @param output The duty cycle (-1 to 1)
     */
    public void setDutyCycle(double output) {
        motor.set(output);
    }

    /**
     * Set the motor voltage directly.
     *
     * @param volts The voltage to apply (-12 to +12)
     */
    public void setVoltage(double volts) {
        controller.setReference(volts, ControlType.kVoltage);
    }

    // ========================================================================
    // GETTER METHODS - Read arm state
    // ========================================================================

    /**
     * Check if the arm is at a specific rotation (within tolerance).
     *
     * @param rotation The rotation to check against
     * @return true if within ARM_TOLERANCE of the target
     */
    public boolean atRotation(Rotation2d rotation) {
        // Compare in radians for precision
        double diff = Math.abs(getAngle().getRadians() - rotation.getRadians());
        return diff < CoralArmConstants.ARM_TOLERANCE.getRadians();
    }

    /**
     * Check if the arm is at the goal rotation.
     *
     * @return true if at the last target angle set via setAngle()
     */
    public boolean atGoalRotation() {
        return goalRotation != null && atRotation(goalRotation);
    }

    /**
     * Get the current angle of the arm.
     *
     * @return The current angle as a Rotation2d (offset-corrected)
     */
    public Rotation2d getAngle() {
        // Subtract offset to get the "real" angle
        double degrees = absoluteEncoder.getPosition() - encoderOffset;
        return Rotation2d.fromDegrees(degrees);
    }

    /**
     * Get the name of the current command.
     *
     * @return The command name or empty string if none
     */
    public String getCurrentCommandName() {
        Command current = getCurrentCommand();
        return current != null ? current.getName() : "";
    }

    // ========================================================================
    // COMMANDS - Actions for the arm
    // ========================================================================

    /**
     * Create a command to set the arm to a specific angle.
     *
     * The command runs until the arm reaches the target angle,
     * then ends automatically.
     *
     * @param angle The target angle
     * @return A command that moves the arm to the angle
     *
     * EXAMPLE:
     *   someButton.onTrue(coralArm.setAngleCommand(Rotation2d.fromDegrees(-35)));
     */
    public Command setAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), this)  // Continuously set angle
            .until(this::atGoalRotation)                  // Stop when reached
            .withName("SetAngle(" + angle.getDegrees() + ")");
    }

    // ========================================================================
    // SYSID COMMANDS - For tuning
    // ========================================================================

    /**
     * SysId quasistatic test (slowly increase voltage).
     *
     * @param direction Forward or reverse
     * @return The test command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * SysId dynamic test (sudden voltage step).
     *
     * @param direction Forward or reverse
     * @return The test command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}  // End of CoralArm class
