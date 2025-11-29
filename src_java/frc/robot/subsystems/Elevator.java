package frc.robot.subsystems;

/*
 * ========================================================================
 * ELEVATOR SUBSYSTEM - Vertical Lift Mechanism
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls the elevator (vertical lift) that raises and lowers game pieces
 * to different scoring heights. Think of it like an elevator in a building -
 * it moves the robot's "arm" up and down to reach different levels.
 *
 * PHYSICAL HARDWARE:
 * ------------------
 *   - 2 NEO motors (CAN IDs 9 and 10) working together
 *     └─ One is the "leader" (makes decisions)
 *     └─ One is the "follower" (copies the leader)
 *   - Hall Effect sensor (DIO 0) - detects when elevator hits bottom
 *   - Chain/belt drive system with sprockets
 *
 * WHY TWO MOTORS?
 * ---------------
 * Lifting things takes a lot of force! Two motors are stronger than one.
 * The "leader/follower" pattern means we only need to control ONE motor -
 * the follower automatically copies whatever the leader does.
 *
 * KEY CONCEPTS EXPLAINED:
 * -----------------------
 *
 * 1. ENCODER - Measures how far the elevator has moved
 *    Think of it like an odometer in a car - counts rotations and
 *    converts them to distance (meters).
 *
 * 2. PID CONTROL - Makes the elevator go to exact positions smoothly
 *    P = Proportional: "How far away am I?" → Push harder if far away
 *    I = Integral: "Have I been stuck?" → Push harder if stuck for a while
 *    D = Derivative: "Am I going too fast?" → Slow down as you approach
 *
 * 3. FEEDFORWARD - Counteracts gravity
 *    The elevator is heavy! Feedforward adds extra power just to hold it
 *    in place, so the PID doesn't have to work as hard.
 *
 * 4. MOTION PROFILE (TrapezoidProfile) - Smooth acceleration/deceleration
 *    Instead of going 0 → full speed instantly (jerky!), the profile
 *    gradually speeds up, maintains speed, then gradually slows down.
 *    This prevents the robot from tipping and reduces wear on parts.
 *
 *    Speed
 *      ^    ___________
 *      |   /           \      ← Trapezoidal shape
 *      |  /             \
 *      | /               \
 *      +------------------→ Time
 *
 * 5. SOFT LIMITS - Software safety stops
 *    Prevents the elevator from going too high or too low and breaking
 *    itself. Like invisible walls the motor won't go past.
 *
 * 6. HOMING - Finding the "zero" position
 *    The encoder counts relative rotations, not absolute position.
 *    Homing moves down until the limit switch triggers, then sets that
 *    position as the "zero" reference point.
 *
 * HOW TO MODIFY:
 * --------------
 * - Change heights: Edit Constants.java → ElevatorConstants section
 * - Change speed: Edit MAX_VELOCITY and MAX_ACCELERATION in Constants.java
 * - Change PID: Edit POSITION_kP, POSITION_kD, VELOCITY_kP in Constants.java
 *
 * QUICK REFERENCE - Common Tasks:
 * -------------------------------
 * → Move to height: elevator.setHeightCommand(heightInMeters)
 * → Get current height: elevator.getCarriageHeight()
 * → Check if at target: elevator.atGoalHeight()
 * → Home the elevator: elevator.homeElevatorCommand()
 *
 * ========================================================================
 */

import java.util.Set;

// REV Robotics SparkMax motor controller imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// WPILib imports for math, sensors, and commands
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

// Static import for unit types (Volts, Meters, Seconds, etc.)
import static edu.wpi.first.units.Units.*;

/**
 * ========================================================================
 * ELEVATOR SUBSYSTEM
 * ========================================================================
 *
 * Controls the vertical lift mechanism using two NEO motors in a
 * leader/follower configuration. The elevator moves the arm to different
 * scoring heights (L1-L4) for placing game pieces on the reef.
 *
 * This is a SUBSYSTEM - it represents a physical mechanism on the robot.
 * Subsystems have:
 *   - Hardware (motors, sensors)
 *   - State (current position, target position)
 *   - Methods to control the hardware
 *   - Commands that use those methods
 *
 * @see Constants.ElevatorConstants for tunable values
 */
public class Elevator extends SubsystemBase {

    // ========================================================================
    // HARDWARE - Physical components connected to this subsystem
    // ========================================================================

    /**
     * Leader motor - This motor receives commands and the follower copies it.
     * Connected to the LEFT side of the elevator.
     *
     * [HARDWARE] SparkMax motor controller with NEO brushless motor
     * [CAN ID] Defined in Constants.ElevatorConstants.LEFT_MOTOR_ID (default: 9)
     */
    private final SparkMax leader;

    /**
     * Follower motor - Automatically mirrors everything the leader does.
     * Connected to the RIGHT side of the elevator.
     *
     * [HARDWARE] SparkMax motor controller with NEO brushless motor
     * [CAN ID] Defined in Constants.ElevatorConstants.RIGHT_MOTOR_ID (default: 10)
     *
     * [WHY FOLLOWER?] Using follower mode means:
     *   - We only send commands to ONE motor
     *   - The other motor copies automatically
     *   - This keeps both motors perfectly synchronized
     *   - Less CAN bus traffic (fewer messages sent)
     */
    private final SparkMax follower;

    // ========================================================================
    // SENSORS & CONTROLLERS - How we measure and control position
    // ========================================================================

    /**
     * Encoder - Measures the elevator's position and velocity.
     *
     * [HOW IT WORKS]
     * The encoder counts motor rotations. We multiply by a "conversion factor"
     * to convert rotations → meters (the actual height).
     *
     * Conversion factor = Sprocket circumference / Gear ratio
     *   - If sprocket turns once, elevator moves (diameter × π) meters
     *   - If gear ratio is 5:1, motor turns 5 times for 1 sprocket turn
     */
    private final RelativeEncoder encoder;

    /**
     * Closed-loop controller - Runs PID calculations on the SparkMax itself.
     *
     * [WHY ON THE MOTOR?]
     * Running PID on the SparkMax (not the roboRIO) means:
     *   - Faster updates (1kHz vs 50Hz on roboRIO)
     *   - More accurate position control
     *   - Less load on the main robot processor
     */
    private final SparkClosedLoopController controller;

    /**
     * Limit switch - Detects when the elevator reaches the bottom.
     *
     * [HARDWARE] Hall Effect sensor (magnetic sensor)
     * [DIO PORT] Defined in Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_ID
     *
     * [HOW IT WORKS]
     * A magnet on the elevator carriage passes by the sensor when at the bottom.
     * The sensor detects the magnetic field and outputs a signal.
     *
     * [WHY WE NEED IT]
     * The encoder only counts RELATIVE rotations - it doesn't know the
     * absolute position. We use the limit switch to "home" the elevator:
     *   1. Move down until limit switch triggers
     *   2. Reset encoder to the known height at that position
     *   3. Now the encoder reading matches real-world position!
     */
    private final DigitalInput limitSwitch;

    // ========================================================================
    // CONTROL SYSTEMS - Math that makes movement smooth
    // ========================================================================

    /**
     * Feedforward controller - Predicts how much power the elevator needs.
     *
     * [WHAT IS FEEDFORWARD?]
     * PID is "reactive" - it sees an error and reacts to fix it.
     * Feedforward is "proactive" - it predicts how much power is needed.
     *
     * For an elevator, feedforward handles:
     *   - kS: Static friction (minimum power to start moving)
     *   - kG: Gravity (constant power to hold position against gravity)
     *   - kV: Velocity (more power needed to go faster)
     *   - kA: Acceleration (extra power to speed up/slow down)
     *
     * [ANALOGY]
     * Imagine holding a heavy box. Feedforward is like knowing "I need to
     * push up with X pounds just to keep it still" before you even try.
     */
    private final ElevatorFeedforward feedforward;

    /**
     * Motion profile - Plans a smooth path from current position to target.
     *
     * [WHAT IS A TRAPEZOID PROFILE?]
     * Instead of instantly trying to reach a position (jerky!), the profile
     * creates a smooth motion plan:
     *
     *   1. ACCELERATE - Gradually speed up (don't jerk the robot)
     *   2. CRUISE - Travel at max speed
     *   3. DECELERATE - Gradually slow down (don't overshoot)
     *
     * The velocity graph looks like a trapezoid:
     *
     *   Velocity
     *     ^     ____________
     *     |    /            \
     *     |   /   CRUISE     \
     *     |  / ACCEL    DECEL \
     *     | /                  \
     *     +---------------------→ Time
     *
     * [WHY IT MATTERS]
     * Smooth motion = less stress on parts = robot doesn't tip over
     */
    private final TrapezoidProfile profile;

    // ========================================================================
    // VISUALIZATION - Tools for debugging and testing
    // ========================================================================

    /**
     * Mechanism2d - Visual representation of the elevator in Shuffleboard.
     *
     * [WHAT IS THIS?]
     * Mechanism2d creates a live animation of the elevator in the dashboard.
     * You can see the carriage move up and down in real-time!
     *
     * [HOW TO VIEW]
     * In Shuffleboard: Drag "Elevator Mechanism" widget to a tab
     */
    private final Mechanism2d mechanism;
    private final MechanismLigament2d elevatorLigament;

    /**
     * SysId routine - Tool for automatically tuning feedforward values.
     *
     * [WHAT IS SYSID?]
     * System Identification (SysId) is a WPILib tool that:
     *   1. Runs the motor at different speeds
     *   2. Records voltage, position, and velocity
     *   3. Calculates the optimal feedforward constants (kS, kG, kV, kA)
     *
     * [WHEN TO USE]
     * Run SysId when:
     *   - Setting up a new mechanism
     *   - After changing gearing or motors
     *   - If the elevator isn't moving smoothly
     *
     * [HOW TO RUN]
     * Use the SysId commands: sysIdQuasistatic() and sysIdDynamic()
     */
    private final SysIdRoutine sysIdRoutine;

    // ========================================================================
    // STATE TRACKING - Remember what we're trying to do
    // ========================================================================

    /**
     * Target height we're trying to reach (in METERS).
     * null = no target set
     */
    private Double goalHeight = null;

    /**
     * Target velocity we're trying to maintain (in METERS PER SECOND).
     * null = no velocity target set
     */
    private Double goalVelocity = null;

    /**
     * Human-readable name of the current scoring level (e.g., "L3").
     * Used for dashboard display.
     */
    private String goalLevelName = null;

    // ========================================================================
    // CONSTRUCTOR - Initialize all hardware and control systems
    // ========================================================================

    /**
     * Creates a new Elevator subsystem.
     *
     * This constructor:
     *   1. Creates motor controller objects
     *   2. Sets up the encoder and PID controller
     *   3. Initializes the limit switch
     *   4. Creates the feedforward calculator
     *   5. Sets up motion profiling
     *   6. Configures motor settings (current limits, PID, etc.)
     *   7. Sets up dashboard visualization
     *   8. Prepares SysId for tuning
     */
    public Elevator() {
        // ----------------------------------------------------------------
        // STEP 1: Create motor objects
        // ----------------------------------------------------------------
        // SparkMax constructor takes:
        //   - CAN ID: The unique address of this motor on the CAN bus
        //   - MotorType: kBrushless for NEO motors, kBrushed for CIM/775pro
        leader = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        follower = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        // Get references to the built-in encoder and PID controller
        // These are inside the SparkMax - we're just getting handles to them
        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();

        // ----------------------------------------------------------------
        // STEP 2: Create limit switch object
        // ----------------------------------------------------------------
        // DigitalInput reads a simple on/off signal from a sensor
        // The number is the DIO (Digital I/O) port on the roboRIO
        limitSwitch = new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_ID);

        // ----------------------------------------------------------------
        // STEP 3: Create feedforward calculator
        // ----------------------------------------------------------------
        // The feedforward constants come from SysId characterization
        //   kS = Static friction (volts to overcome friction)
        //   kG = Gravity compensation (volts to hold against gravity)
        //   kV = Velocity coefficient (volts per meter/second)
        //   kA = Acceleration coefficient (volts per meter/second²)
        feedforward = new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD_CONSTANTS[0],  // kS
            ElevatorConstants.FEEDFORWARD_CONSTANTS[1],  // kG
            ElevatorConstants.FEEDFORWARD_CONSTANTS[2],  // kV
            ElevatorConstants.FEEDFORWARD_CONSTANTS[3]   // kA
        );

        // ----------------------------------------------------------------
        // STEP 4: Create motion profile
        // ----------------------------------------------------------------
        // TrapezoidProfile plans smooth acceleration/deceleration
        // Constraints define the limits:
        //   - MAX_VELOCITY: Fastest the elevator can move (m/s)
        //   - MAX_ACCELERATION: Fastest it can speed up/slow down (m/s²)
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY,
                ElevatorConstants.MAX_ACCELERATION
            )
        );

        // ----------------------------------------------------------------
        // STEP 5: Configure motors (PID, current limits, etc.)
        // ----------------------------------------------------------------
        configureMotors();

        // Set initial encoder position to the minimum height
        // (This assumes the elevator starts at the bottom)
        reset(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT);

        // ----------------------------------------------------------------
        // STEP 6: Setup mechanism visualization for dashboard
        // ----------------------------------------------------------------
        // Mechanism2d creates a visual widget in Shuffleboard
        // Parameters: width=3, height=4 (in arbitrary units for the canvas)
        mechanism = new Mechanism2d(3, 4);

        // Create a "root" point where the elevator base is
        MechanismRoot2d root = mechanism.getRoot("elevator", 2, 0);

        // Create a line ("ligament") that represents the elevator carriage
        // It will move up/down to show current height
        elevatorLigament = root.append(new MechanismLigament2d("carriage", getCarriageHeight(), 90));

        // Publish to SmartDashboard so we can see it in Shuffleboard
        SmartDashboard.putData("Elevator Mechanism", mechanism);

        // ----------------------------------------------------------------
        // STEP 7: Setup SysId routine for motor characterization
        // ----------------------------------------------------------------
        // SysId helps us find the optimal feedforward values
        // Config parameters:
        //   - Ramp rate: 0.3 volts per second (how fast to increase voltage)
        //   - Step voltage: 2 volts (voltage for dynamic tests)
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.3).per(Seconds.of(1)),  // Ramp rate for quasistatic test
                Volts.of(2),                       // Step voltage for dynamic test
                null,                              // Use default timeout
                null                               // No state callback
            ),
            new SysIdRoutine.Mechanism(
                // Function to apply voltage to the motor
                voltage -> setVoltage(voltage.in(Volts)),

                // Function to log data (position, velocity, voltage)
                log -> {
                    log.motor("elevator")
                        .voltage(Volts.of(leader.getAppliedOutput() * leader.getBusVoltage()))
                        .linearPosition(Meters.of(encoder.getPosition()))
                        .linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
                },

                // The subsystem this routine belongs to
                this
            )
        );
    }

    // ========================================================================
    // MOTOR CONFIGURATION - Set up PID, limits, and encoder conversion
    // ========================================================================

    /**
     * Configure both motors with appropriate settings.
     *
     * This method sets up:
     *   - Encoder conversion factors (rotations → meters)
     *   - PID gains for position and velocity control
     *   - Current limits to prevent motor damage
     *   - Soft limits to prevent mechanical damage
     *   - Leader/follower relationship
     *
     * [WHEN IS THIS CALLED?]
     * Only once, during the constructor. The settings are saved to the
     * SparkMax's flash memory and persist even after power cycles.
     */
    private void configureMotors() {

        // ================================================================
        // ENCODER CONVERSION FACTORS
        // ================================================================
        // These convert motor rotations into real-world units (meters)
        //
        // [THE MATH]
        // For a chain/belt elevator:
        //   - Motor rotates → Sprocket rotates → Chain moves → Elevator moves
        //   - If gear ratio is 5:1, motor spins 5 times for 1 sprocket rotation
        //   - Each sprocket rotation moves the chain by (diameter × π) meters
        //   - The "2" multiplier is because the chain system doubles the travel
        //     (cascade elevator or specific rigging configuration)
        //
        // FORMULA:
        //   positionFactor = 2 × (1/GEAR_RATIO) × (SPROCKET_DIAMETER × π)
        //
        // EXAMPLE:
        //   Gear ratio = 5:1, Sprocket diameter = 0.04m
        //   positionFactor = 2 × (1/5) × (0.04 × 3.14159) = 0.0503 meters/rotation
        //   So if motor rotates 100 times, elevator moves 5.03 meters
        double positionFactor = 2 * (1.0 / ElevatorConstants.GEAR_RATIO)
            * (ElevatorConstants.SPROCKET_PITCH_DIAMETER * Math.PI);

        // Velocity factor: The encoder reports velocity in RPM (rotations per minute)
        // We want meters per second, so divide by 60 to convert minutes → seconds
        double velocityFactor = positionFactor / 60.0;

        // ================================================================
        // LEADER MOTOR CONFIGURATION
        // ================================================================
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        // Basic motor settings
        leaderConfig
            .idleMode(IdleMode.kBrake)      // BRAKE mode: Motor resists movement when stopped
                                             // (Prevents elevator from falling when power is cut!)
            .smartCurrentLimit(60)           // Limit current to 60 amps to prevent motor damage
            .inverted(ElevatorConstants.INVERT_LEFT_MOTOR);  // Flip direction if needed

        // Configure encoder to output in meters instead of rotations
        leaderConfig.encoder
            .positionConversionFactor(positionFactor)   // Rotations → Meters
            .velocityConversionFactor(velocityFactor);  // RPM → Meters per second

        // ================================================================
        // PID CONFIGURATION - Position Control (Slot 0)
        // ================================================================
        // Slot 0 is used for POSITION control (go to exact height)
        //
        // [PID EXPLAINED]
        // P (Proportional): How hard to push based on distance from target
        //   - Higher P = More aggressive, but may overshoot
        //   - Lower P = More gentle, but may never reach target
        //
        // D (Derivative): Slows down as approaching target (damping)
        //   - Higher D = More damping, prevents overshoot
        //   - Lower D = Less damping, may oscillate
        //
        // [OUTPUT RANGE]
        // Limits how much power the PID can output
        //   - -0.5 to MAX_OUT_UP means it can push up at full power
        //   - but only pull down at half power (gravity helps!)
        leaderConfig.closedLoop
            .p(ElevatorConstants.POSITION_kP, ClosedLoopSlot.kSlot0)
            .d(ElevatorConstants.POSITION_kD, ClosedLoopSlot.kSlot0)
            .outputRange(-0.5, ElevatorConstants.MAX_OUT_UP, ClosedLoopSlot.kSlot0);

        // ================================================================
        // PID CONFIGURATION - Velocity Control (Slot 1)
        // ================================================================
        // Slot 1 is used for VELOCITY control (maintain speed)
        // Used with Smart Motion for smooth profiled movement
        leaderConfig.closedLoop
            .p(ElevatorConstants.VELOCITY_kP, ClosedLoopSlot.kSlot1)
            .outputRange(ElevatorConstants.MAX_OUT_DOWN, ElevatorConstants.MAX_OUT_UP, ClosedLoopSlot.kSlot1);

        // ================================================================
        // SMART MOTION - Built-in motion profiling on the SparkMax
        // ================================================================
        // Smart Motion creates smooth trapezoidal velocity profiles
        // The SparkMax calculates the profile internally (faster than roboRIO)
        leaderConfig.closedLoop.smartMotion
            .maxVelocity(ElevatorConstants.MAX_VELOCITY, ClosedLoopSlot.kSlot1)
            .minOutputVelocity(ElevatorConstants.MAX_VELOCITY / 2, ClosedLoopSlot.kSlot1)
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(ElevatorConstants.HEIGHT_TOLERANCE, ClosedLoopSlot.kSlot1);

        // ================================================================
        // SOFT LIMITS - Software safety boundaries
        // ================================================================
        // Soft limits prevent the motor from driving past certain positions
        // Even if code tells it to go past, the SparkMax will refuse
        //
        // [WHY SOFT LIMITS?]
        // - Prevents elevator from crashing into top/bottom of frame
        // - Protects mechanical components
        // - Works even if our code has bugs
        //
        // NOTE: We disable these during homing (to find the true bottom)
        leaderConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT)  // Upper limit
            .reverseSoftLimit(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT)  // Lower limit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        // Apply configuration to the leader motor
        // ResetMode.kResetSafeParameters = Clear any existing settings first
        // PersistMode.kPersistParameters = Save to flash memory (survives reboot)
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ================================================================
        // FOLLOWER MOTOR CONFIGURATION
        // ================================================================
        // The follower just copies the leader - much simpler config!
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .idleMode(IdleMode.kBrake)       // Also brake mode for safety
            .smartCurrentLimit(60)            // Same current limit
            .follow(ElevatorConstants.LEFT_MOTOR_ID, ElevatorConstants.INVERT_RIGHT_MOTOR);
            // .follow() tells this motor to copy the leader
            // Second parameter: true = spin opposite direction (for opposite-mounted motors)

        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ========================================================================
    // PERIODIC - Runs every robot loop (every 20ms / 50 times per second)
    // ========================================================================

    /**
     * Called periodically by the scheduler (every 20ms).
     *
     * [WHAT IS PERIODIC?]
     * This method is automatically called by the robot framework in a loop.
     * It's the heartbeat of the subsystem - use it for:
     *   - Updating dashboard values
     *   - Checking sensor states
     *   - Running continuous monitoring
     *
     * [IMPORTANT]
     * Don't put slow code here! This runs 50 times per second.
     * Slow code will make the whole robot laggy.
     */
    @Override
    public void periodic() {
        // Update the visual representation in Shuffleboard
        // The "ligament" length changes to show current height
        elevatorLigament.setLength(getCarriageHeight());

        // ----------------------------------------------------------------
        // LOG DATA TO DASHBOARD
        // ----------------------------------------------------------------
        // These values appear in Shuffleboard under "Elevator/..."
        // Use them for debugging and monitoring during testing
        //
        // [HOW TO VIEW]
        // In Shuffleboard: Look for NetworkTables → SmartDashboard → Elevator

        SmartDashboard.putNumber("Elevator/Height", getCarriageHeight());         // Current height in meters
        SmartDashboard.putNumber("Elevator/Velocity", encoder.getVelocity());     // Current speed in m/s
        SmartDashboard.putBoolean("Elevator/LimitSwitch", lowerLimit());          // Is at bottom?
        SmartDashboard.putNumber("Elevator/Voltage", leader.getAppliedOutput() * leader.getBusVoltage());  // Power being used
        SmartDashboard.putBoolean("Elevator/AtGoal", atGoalHeight());             // Have we reached target?

        // Only show goal info if we have a target
        if (goalHeight != null) {
            SmartDashboard.putNumber("Elevator/GoalHeight", goalHeight);          // Where we're trying to go
            SmartDashboard.putNumber("Elevator/HeightError", heightError());      // How far from target
        }
    }

    // ========================================================================
    // SETTER METHODS - Control the elevator
    // ========================================================================

    /**
     * Reset the encoder to a specific height.
     *
     * [WHEN TO USE]
     * Call this after homing to tell the encoder "this is your new zero reference"
     *
     * @param height The height to set the encoder to (in METERS)
     *
     * EXAMPLE:
     *   reset(0.775);  // Encoder now reads 0.775m at current physical position
     */
    public void reset(double height) {
        encoder.setPosition(height);
    }

    /**
     * Set the target height using position control (PID Slot 0).
     *
     * This tells the elevator "go to this height and stay there."
     * The PID controller handles all the motor power calculations.
     *
     * @param height The target height in METERS
     *
     * EXAMPLE:
     *   setHeight(1.0);  // Go to 1 meter height
     *   setHeight(Units.inchesToMeters(30));  // Go to 30 inches
     *
     * NOTE: This method is used for precise positioning.
     * The elevator will hold this position until told otherwise.
     */
    public void setHeight(double height) {
        goalHeight = height;  // Remember our target
        controller.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // ^ Tell the SparkMax's built-in PID to go to this position
        //   Using Slot 0 = position control PID gains
    }

    /**
     * Set the target velocity using velocity control (PID Slot 1).
     *
     * This tells the elevator "move at this speed" rather than
     * "go to this position." Useful for manual joystick control.
     *
     * @param velocity The target velocity in METERS PER SECOND
     *                 Positive = up, Negative = down
     *
     * EXAMPLE:
     *   setVelocity(0.5);   // Move up at 0.5 m/s
     *   setVelocity(-0.3);  // Move down at 0.3 m/s
     *   setVelocity(0);     // Stop
     */
    public void setVelocity(double velocity) {
        goalVelocity = velocity;
        // The "1.0 / 473.0" is an arbitrary feedforward value
        // It helps overcome static friction when starting to move
        controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1, 1.0 / 473.0);
    }

    /**
     * Set the motor duty cycle directly (open-loop control).
     *
     * [WHAT IS DUTY CYCLE?]
     * Duty cycle is the percentage of power sent to the motor.
     *   - 1.0 = Full power forward (up)
     *   - 0.0 = No power (coast or brake depending on idle mode)
     *   - -1.0 = Full power reverse (down)
     *
     * [WHEN TO USE]
     * Usually for simple manual control or testing.
     * Position/velocity control is better for precise movements.
     *
     * @param output The duty cycle from -1.0 to 1.0
     *
     * [WARNING] No safety checks! Can damage mechanism if misused.
     */
    public void setDutyCycle(double output) {
        leader.set(output);  // Follower automatically copies
    }

    /**
     * Set the motor voltage directly.
     *
     * [WHY USE VOLTAGE?]
     * Duty cycle depends on battery voltage - when battery is low,
     * the same duty cycle gives less power. Voltage control compensates
     * for this, giving consistent behavior regardless of battery level.
     *
     * @param volts The voltage to apply (typically -12 to +12)
     *
     * [WHEN TO USE]
     * - During SysId characterization
     * - For feedforward calculations
     * - When you need consistent force regardless of battery
     */
    public void setVoltage(double volts) {
        controller.setReference(volts, ControlType.kVoltage);
    }

    /**
     * Enable or disable soft limits.
     *
     * [WHAT ARE SOFT LIMITS?]
     * Software-enforced boundaries that prevent the motor from
     * driving past certain positions. The SparkMax ignores commands
     * that would push past the limit.
     *
     * [WHEN TO DISABLE]
     * Only during homing! We need to move past the soft limit
     * to find the physical limit switch.
     *
     * @param enable true = limits active (safe), false = limits disabled (careful!)
     */
    public void enableSoftLimits(boolean enable) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.softLimit
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimitEnabled(enable);
        // kNoResetSafeParameters = Don't reset other settings
        // kNoPersistParameters = Don't save to flash (temporary change)
        leader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // ========================================================================
    // GETTER METHODS - Read elevator state
    // ========================================================================

    /**
     * Get the current carriage height.
     *
     * @return The carriage height in METERS
     *
     * EXAMPLE:
     *   double height = elevator.getCarriageHeight();
     *   if (height > 1.0) {
     *       System.out.println("Elevator is above 1 meter!");
     *   }
     */
    public double getCarriageHeight() {
        return encoder.getPosition();  // Already converted to meters by conversion factor
    }

    /**
     * Get the current carriage height in inches.
     *
     * Convenience method for those who prefer imperial units.
     * Internally, everything uses meters, but inches are easier to
     * visualize for many people.
     *
     * @return The carriage height in INCHES
     *
     * EXAMPLE:
     *   System.out.println("Height: " + elevator.getCarriageHeightInches() + " inches");
     */
    public double getCarriageHeightInches() {
        // 1 meter = 39.37 inches
        return getCarriageHeight() * 39.37;
    }

    /**
     * Check if the lower limit switch is triggered.
     *
     * [HARDWARE NOTE]
     * Hall Effect sensors return FALSE when the magnet is detected (normally high).
     * So we invert the reading: !false = true = limit triggered
     *
     * @return true if the elevator is at the bottom (magnet detected)
     */
    public boolean lowerLimit() {
        // Hall Effect sensor is "active low" - outputs false when magnet present
        return !limitSwitch.get();
    }

    // ========================================================================
    // STATE CHECK METHODS - Are we where we want to be?
    // ========================================================================

    /**
     * Check if the elevator is at a specific height (within tolerance).
     *
     * [WHY TOLERANCE?]
     * Motors can't hit EXACT positions - there's always tiny errors.
     * We say "close enough" if within HEIGHT_TOLERANCE (default: 0.01m = 1cm)
     *
     * @param height The target height to check against (in METERS)
     * @return true if current height is within tolerance of target
     *
     * EXAMPLE:
     *   if (elevator.atHeight(1.5)) {
     *       System.out.println("Reached 1.5 meters!");
     *   }
     */
    public boolean atHeight(double height) {
        // In simulation, always return true (simulation doesn't have real physics)
        if (RobotBase.isSimulation()) {
            return true;
        }
        // Check if |current - target| < tolerance
        return Math.abs(getCarriageHeight() - height) < ElevatorConstants.HEIGHT_TOLERANCE;
    }

    /**
     * Check if the elevator is at the GOAL height (the last setHeight target).
     *
     * [DIFFERENCE FROM atHeight()]
     * - atHeight(x) checks against any arbitrary height x
     * - atGoalHeight() checks against the last target we set
     *
     * @return true if at the goal height within tolerance
     *
     * COMMON USE:
     *   // In a command's isFinished() method:
     *   return elevator.atGoalHeight();
     */
    public boolean atGoalHeight() {
        // Make sure we actually have a goal set (not null)
        return goalHeight != null && atHeight(goalHeight);
    }

    /**
     * Get the height error from the goal (how far from target).
     *
     * Positive error = below target (need to go UP)
     * Negative error = above target (need to go DOWN)
     *
     * @return The height error in METERS (goal - current)
     *
     * USEFUL FOR:
     *   - Debugging PID tuning
     *   - Dashboard displays
     *   - Logging
     */
    public double heightError() {
        return goalHeight != null ? goalHeight - getCarriageHeight() : 0;
    }

    /**
     * Get the velocity error from the goal (how far from target speed).
     *
     * @return The velocity error in METERS PER SECOND
     */
    public double velocityError() {
        return goalVelocity != null ? goalVelocity - encoder.getVelocity() : 0;
    }

    /**
     * Get the name of the command currently controlling this subsystem.
     *
     * [WHY IS THIS USEFUL?]
     * For debugging - see what command is running on the dashboard
     *
     * @return The command name, or empty string if no command running
     */
    public String getCurrentCommandName() {
        Command current = getCurrentCommand();  // From SubsystemBase
        return current != null ? current.getName() : "";
    }

    // ========================================================================
    // COMMANDS - Actions the elevator can perform
    // ========================================================================
    /*
     * WHAT ARE COMMANDS?
     * ------------------
     * Commands are actions that use subsystems. In WPILib's Command-based
     * framework, commands are the "verbs" (actions) and subsystems are the
     * "nouns" (things that act).
     *
     * Commands have a lifecycle:
     *   1. initialize() - Called once when command starts
     *   2. execute() - Called repeatedly while command runs (every 20ms)
     *   3. isFinished() - Checked to see if command should end
     *   4. end() - Called once when command ends
     *
     * WHY COMMANDS?
     * - Reusable: Create once, use many times
     * - Composable: Combine commands (sequence, parallel, etc.)
     * - Safe: Scheduler ensures only one command uses a subsystem at a time
     */

    /**
     * Create a command that moves the elevator to a specific height.
     *
     * This is a FACTORY METHOD - it creates and returns a new Command object.
     *
     * [HOW IT WORKS]
     * 1. Creates a profiled movement command (smooth acceleration)
     * 2. After profile completes, switches to position hold
     *
     * @param height The target height in METERS
     * @return A Command that moves the elevator to the target height
     *
     * EXAMPLE USAGE:
     *   // In RobotContainer button bindings:
     *   someButton.onTrue(elevator.setHeightCommand(1.5));
     *
     *   // In an auto routine:
     *   addCommands(
     *       elevator.setHeightCommand(ScoringConstants.L3_HEIGHT)
     *   );
     */
    public Command setHeightCommand(double height) {
        return new SetProfiledHeightCommand(height, this)
            // After the profile finishes, hold the position with regular PID
            .andThen(Commands.runOnce(() -> setHeight(height), this));
    }

    /**
     * Create a command that homes the elevator using the limit switch.
     *
     * [WHAT IS HOMING?]
     * The encoder only counts relative rotations - it doesn't know the
     * absolute position when the robot turns on. Homing establishes
     * a known reference point.
     *
     * [THE HOMING SEQUENCE]
     * 1. Disable soft limits (we need to move past them)
     * 2. If already at limit, move UP first (can't go down if already there!)
     * 3. Wait a moment
     * 4. Move DOWN slowly until limit switch triggers
     * 5. Reset encoder to the known height at that position
     * 6. Stop the motor
     * 7. Re-enable soft limits
     *
     * @return A Command that homes the elevator
     *
     * WHEN TO USE:
     *   - At the start of a match
     *   - If the elevator seems "off" (wrong positions)
     *   - After any mechanical adjustment
     */
    public Command homeElevatorCommand() {
        return Commands.sequence(
            // STEP 1: Disable soft limits so we can move freely
            Commands.runOnce(() -> enableSoftLimits(false)),

            // STEP 2: If we're already at the limit, move up first
            // .onlyWhile() runs ONLY while the condition is true
            Commands.run(() -> setVoltage(1.5)).onlyWhile(this::lowerLimit),

            // STEP 3: Wait a moment to ensure we've moved away
            Commands.waitSeconds(0.2),

            // STEP 4: Move down until the limit switch triggers
            // .until() runs UNTIL the condition becomes true
            Commands.run(() -> setVoltage(-1)).until(this::lowerLimit),

            // STEP 5: We're at the limit! Reset the encoder to known height
            Commands.runOnce(() -> reset(ElevatorConstants.LIMIT_SWITCH_HEIGHT)),

            // STEP 6: Stop the motors (0 volts)
            Commands.runOnce(() -> setVoltage(0)),

            // STEP 7: Re-enable soft limits for safety
            Commands.runOnce(() -> enableSoftLimits(true))
        ).withName("Home Elevator");  // Give command a name for dashboard
    }

    // ========================================================================
    // SYSID COMMANDS - For motor characterization (advanced tuning)
    // ========================================================================
    /*
     * [WHAT IS SYSID?]
     * System Identification (SysId) is a tool that helps you find the best
     * feedforward values for your mechanism. It works by:
     *   1. Running the motor at various power levels
     *   2. Recording position, velocity, and voltage
     *   3. Analyzing the data to calculate kS, kG, kV, kA
     *
     * [TWO TEST TYPES]
     * - QUASISTATIC: Slowly ramps voltage (finds static friction and velocity)
     * - DYNAMIC: Applies sudden voltage (finds acceleration characteristics)
     *
     * [HOW TO USE]
     * 1. Run all 4 tests: quasistatic forward, quasistatic reverse,
     *                     dynamic forward, dynamic reverse
     * 2. Export the data log
     * 3. Use WPILib's SysId tool to analyze
     * 4. Update feedforward constants in Constants.java
     */

    /**
     * Create a SysId quasistatic test command.
     *
     * QUASISTATIC = Slowly increase voltage over time
     * Used to find kS (static friction) and kV (velocity coefficient)
     *
     * @param direction SysIdRoutine.Direction.kForward or kReverse
     * @return Command to run the quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Create a SysId dynamic test command.
     *
     * DYNAMIC = Apply sudden step voltage
     * Used to find kA (acceleration coefficient)
     *
     * @param direction SysIdRoutine.Direction.kForward or kReverse
     * @return Command to run the dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // ========================================================================
    // INNER COMMAND CLASS - SetProfiledHeightCommand
    // ========================================================================
    /*
     * WHY AN INNER CLASS?
     * -------------------
     * This command needs tight integration with the Elevator subsystem.
     * Making it an inner class means:
     *   - It can access private fields of Elevator (like goalHeight)
     *   - It's clearly associated with Elevator
     *   - All elevator-related code stays in one file
     *
     * WHAT THIS COMMAND DOES:
     * -----------------------
     * Moves the elevator to a target height using a TRAPEZOIDAL MOTION PROFILE.
     * This ensures smooth acceleration and deceleration, preventing the robot
     * from jerking around.
     *
     * THE MOTION PROFILE:
     *
     *   Velocity
     *     ^     ____________     ← Max velocity
     *     |    /            \
     *     |   /   CRUISE     \    ← Travels at constant speed
     *     |  / ACCEL    DECEL \   ← Slows down as approaching target
     *     | /                  \
     *     +---------------------→ Time
     *       Start            End
     *
     * HOW IT WORKS:
     * 1. initialize(): Record current position, set up profile
     * 2. execute(): Calculate next position from profile, command elevator
     * 3. isFinished(): Check if profile is done AND elevator reached target
     * 4. end(): Clean up (stop timer)
     */

    /**
     * Command that moves the elevator to a height using a trapezoidal profile.
     *
     * A trapezoidal profile smoothly accelerates, cruises, then decelerates
     * to reach the target position without jerking or overshooting.
     */
    public static class SetProfiledHeightCommand extends Command {

        // The target height we want to reach (in METERS)
        private final double goalPosition;

        // Reference to the elevator subsystem we're controlling
        private final Elevator elevator;

        // Motion profile calculator
        private final TrapezoidProfile profile;

        // Timer to track how long we've been running
        private final Timer timer = new Timer();

        // Current state: where we are in the profile (position + velocity)
        private TrapezoidProfile.State setpoint;

        // Goal state: where we want to end up (position + 0 velocity)
        private TrapezoidProfile.State goal;

        /**
         * Create a new profiled height command.
         *
         * @param height Target height in METERS
         * @param elevator The elevator subsystem to control
         */
        public SetProfiledHeightCommand(double height, Elevator elevator) {
            this.goalPosition = height;
            this.elevator = elevator;

            // Create a new profile with velocity and acceleration limits
            this.profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.MAX_VELOCITY,      // Max speed (m/s)
                    ElevatorConstants.MAX_ACCELERATION   // Max acceleration (m/s²)
                )
            );

            // Tell the scheduler this command uses the elevator
            // This prevents other commands from using it simultaneously
            addRequirements(elevator);

            // Give it a name for dashboard display
            setName("ProfiledHeightCommand");
        }

        /**
         * Called ONCE when the command starts.
         *
         * Sets up the motion profile starting from current position/velocity.
         */
        @Override
        public void initialize() {
            // SETPOINT = Current state (where we are NOW)
            // Start from current position and velocity
            setpoint = new TrapezoidProfile.State(
                elevator.getCarriageHeight(),      // Current position
                elevator.encoder.getVelocity()     // Current velocity
            );

            // GOAL = Target state (where we want to END UP)
            // Target position with 0 velocity (stopped at target)
            goal = new TrapezoidProfile.State(goalPosition, 0);

            // Tell the elevator what we're aiming for
            elevator.goalHeight = goalPosition;

            // Start the timer (used to check if profile is complete)
            timer.restart();
        }

        /**
         * Called REPEATEDLY while the command runs (every 20ms).
         *
         * Calculates the next step in the motion profile and
         * commands the elevator to that position.
         */
        @Override
        public void execute() {
            // Command the elevator to the current setpoint position
            elevator.setHeight(setpoint.position);

            // Calculate the NEXT setpoint for the profile
            // Parameters:
            //   0.02 = Time step (20ms = 0.02 seconds)
            //   setpoint = Where we are in the profile
            //   goal = Where we're trying to get to
            setpoint = profile.calculate(0.02, setpoint, goal);
        }

        /**
         * Called ONCE when the command ends (either finished or interrupted).
         *
         * @param interrupted true if the command was interrupted, false if it finished normally
         */
        @Override
        public void end(boolean interrupted) {
            timer.stop();
            // Note: We don't stop the elevator here because the parent command
            // (setHeightCommand) chains a position-hold command after this
        }

        /**
         * Determines when the command should end.
         *
         * @return true when the motion profile is complete AND
         *         the elevator has actually reached the target height
         *
         * [WHY BOTH CONDITIONS?]
         * - profile.isFinished(): The math says we should be there
         * - elevator.atGoalHeight(): The physical elevator actually arrived
         *
         * Sometimes the profile finishes but the elevator is still moving
         * (due to momentum, friction, etc.). We wait for both to be true.
         */
        @Override
        public boolean isFinished() {
            return profile.isFinished(timer.get()) && elevator.atGoalHeight();
        }

        /**
         * Returns the subsystems this command uses.
         *
         * This is important for the scheduler to prevent conflicts.
         * Two commands can't use the same subsystem at the same time.
         *
         * @return A set containing just the elevator subsystem
         */
        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(elevator);
        }
    }
}  // End of Elevator class
