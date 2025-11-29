package frc.robot.subsystems;

/*
 * ========================================================================
 * CLAW SUBSYSTEM - Game Piece Intake/Outtake
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls the "claw" mechanism that grabs and releases CORAL game pieces.
 * This is the end effector at the tip of the arm - it's what actually
 * picks up and places the game pieces.
 *
 * Think of it like a hand: The elevator is the shoulder, the arm is the
 * forearm/elbow, and the claw is the hand that grabs things.
 *
 * PHYSICAL HARDWARE:
 * ------------------
 *   - NEO 550 motor (CAN ID 11)
 *     └─ NEO 550 is a smaller, faster motor - perfect for intakes
 *     └─ Drives rollers/wheels that pull in or push out CORAL
 *   - Time-of-Flight (ToF) sensor (ID 1)
 *     └─ Measures distance using light (like a tiny radar)
 *     └─ Detects when a CORAL piece is inside the claw
 *
 * HOW INTAKE/OUTTAKE WORKS:
 * -------------------------
 *
 *   INTAKE (pulling in):         OUTTAKE (pushing out):
 *   Motor spins → (+)            Motor spins ← (-)
 *         ↓ ↓                          ↑ ↑
 *      ○═════○                      ○═════○
 *      │ CORAL│  → enters          │ CORAL│  → exits
 *      ○═════○                      ○═════○
 *
 * PIECE DETECTION:
 * ----------------
 * We detect if we have a CORAL piece by checking:
 *   1. Motor current is high (motor is working hard against piece)
 *   2. Wheel speed is low (piece is stopping the wheels)
 *   3. Motor has been running for at least 0.5 seconds (ignore startup)
 *
 * If all three are true, we probably have a piece!
 *
 * HOW TO MODIFY:
 * --------------
 * - Change intake/outtake speed: Edit the values in intake()/outtake() methods
 * - Change detection threshold: ClawConstants.THRESHOLD_CURRENT
 * - Change motor ID: ClawConstants.MOTOR_ID
 *
 * QUICK REFERENCE:
 * ----------------
 * → Start intake: claw.intake() or claw.intakeCommand()
 * → Start outtake: claw.outtake() or claw.outtakeCommand()
 * → Check for piece: claw.hasPossession()
 * → Stop motors: claw.stop()
 *
 * ========================================================================
 */

// Playing With Fusion Time-of-Flight sensor
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

// REV Robotics SparkMax imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// WPILib imports
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * ========================================================================
 * CLAW SUBSYSTEM
 * ========================================================================
 *
 * The claw is the "end effector" - the mechanism that physically
 * interacts with CORAL game pieces. Uses roller wheels to intake
 * and outtake pieces.
 *
 * This is one of the simpler subsystems - great for learning!
 *
 * @see Constants.ClawConstants for tunable values
 */
public class Claw extends SubsystemBase {

    // ========================================================================
    // HARDWARE - Motor and sensors
    // ========================================================================

    /**
     * SparkMax motor controller with NEO 550 motor.
     *
     * [NEO 550 vs NEO]
     * NEO 550 is smaller and lighter than the full NEO motor.
     * It's perfect for low-load applications like intakes where
     * you want speed more than torque.
     */
    private final SparkMax motor;

    /**
     * Encoder to measure wheel speed (for piece detection).
     */
    private final RelativeEncoder encoder;

    /**
     * Time-of-Flight distance sensor - Detects objects using light.
     *
     * [HOW IT WORKS]
     * Sends out a tiny pulse of light, measures how long it takes to
     * bounce back. Closer objects = faster return = shorter distance.
     *
     * [WHY WE HAVE IT]
     * Can detect when a CORAL piece is inside the claw.
     * (Currently using current-sensing instead, but ToF is available)
     */
    private final TimeOfFlight distanceSensor;

    // ========================================================================
    // STATE - Timers and simulation
    // ========================================================================

    /**
     * Timer for tracking how long the motor has been running.
     * Used to ignore piece detection during motor spin-up
     * (motor draws high current when starting, which could false-trigger).
     */
    private final Timer timer = new Timer();

    /**
     * Simulated piece possession flag (for testing in simulation).
     * In simulation, we can't read real sensor values, so we use this
     * flag to pretend we have/don't have a piece.
     */
    private boolean hasPieceSim = false;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new Claw subsystem.
     */
    public Claw() {
        // ----------------------------------------------------------------
        // MOTOR SETUP
        // ----------------------------------------------------------------
        motor = new SparkMax(ClawConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor settings
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)      // Brake mode (hold piece in place)
            .smartCurrentLimit(20)           // Low current limit - NEO 550 is small
            .inverted(true);                 // Flip direction if needed

        // Encoder conversion (rotations → degrees, RPM → deg/sec)
        config.encoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360.0 / 60.0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ----------------------------------------------------------------
        // DISTANCE SENSOR SETUP
        // ----------------------------------------------------------------
        // Time-of-Flight sensor from Playing With Fusion
        distanceSensor = new TimeOfFlight(ClawConstants.TOF_SENSOR_ID);

        // Set the "region of interest" - which pixels on the sensor to use
        // (8,8) to (12,12) is a small area in the center of the sensor
        distanceSensor.setRangeOfInterest(8, 8, 12, 12);

        // Short range mode for close-up detection, 500ms sample time
        distanceSensor.setRangingMode(RangingMode.Short, 500);
    }

    // ========================================================================
    // PERIODIC - Runs every loop
    // ========================================================================

    @Override
    public void periodic() {
        // Log useful values to the dashboard
        SmartDashboard.putBoolean("Claw/HasPossession", hasPossession());
        SmartDashboard.putNumber("Claw/MotorCurrent", motor.getOutputCurrent());
        SmartDashboard.putNumber("Claw/WheelVelocity", encoder.getVelocity());
        SmartDashboard.putNumber("Claw/TimeSinceSpinup", timer.get());
    }

    // ========================================================================
    // CONTROL METHODS - Run the intake
    // ========================================================================

    /**
     * Run the intake motors to pull CORAL into the claw.
     *
     * [HOW IT WORKS]
     * The rollers spin in a direction that pulls objects into the claw.
     * Power is set to 20% (0.2) - enough to grab without being aggressive.
     */
    public void intake() {
        motor.set(0.2);  // 20% power inward
    }

    /**
     * Run the intake motors to push CORAL out of the claw.
     *
     * [HOW IT WORKS]
     * The rollers spin in reverse to push objects out.
     * Power is set to 30% (-0.3) - slightly faster to ensure release.
     */
    public void outtake() {
        motor.set(-0.3);  // 30% power outward
    }

    /**
     * Stop the intake motors.
     */
    public void stop() {
        motor.set(0);
    }

    // ========================================================================
    // PIECE DETECTION - Check if we have a game piece
    // ========================================================================

    /**
     * Check if the claw is currently holding a CORAL piece.
     *
     * [DETECTION METHOD]
     * We use "current sensing" - when the piece is held against the
     * rollers, the motor stalls slightly, causing:
     *   1. Higher motor current (working harder)
     *   2. Lower wheel speed (piece blocking rotation)
     *
     * We also wait 0.5 seconds after starting to avoid false positives
     * from motor spin-up current spikes.
     *
     * @return true if we think we're holding a CORAL piece
     */
    public boolean hasPossession() {
        if (RobotBase.isReal()) {
            // On real robot, use sensor readings
            double motorCurrent = motor.getOutputCurrent();      // Amps
            double motorSpeed = Math.abs(encoder.getVelocity()); // Degrees/sec
            double timeSinceSpinup = timer.get();                // Seconds

            // All three conditions must be true:
            //   1. Current is high (motor working against something)
            //   2. Speed is low (wheels not spinning freely)
            //   3. Motor has been running for a bit (ignore startup)
            return (motorCurrent > ClawConstants.THRESHOLD_CURRENT)
                && (motorSpeed < 500.0)
                && (timeSinceSpinup > 0.5);
        } else {
            // In simulation, use manual flag
            return hasPieceSim;
        }
    }

    /**
     * Toggle the simulated piece possession (for testing in simulation).
     *
     * Call this to pretend we picked up or dropped a piece.
     */
    public void toggleSimPossession() {
        hasPieceSim = !hasPieceSim;
    }

    // ========================================================================
    // COMMANDS - Actions for the claw
    // ========================================================================

    /**
     * Create a command to intake CORAL.
     *
     * This command runs the intake continuously until interrupted.
     * It also restarts the timer (for piece detection timing).
     *
     * @return Command that runs intake
     *
     * TYPICAL USAGE:
     *   // Run intake while button is held
     *   someButton.whileTrue(claw.intakeCommand());
     */
    public Command intakeCommand() {
        return Commands.sequence(
            Commands.runOnce(timer::restart, this),  // Reset timer for detection
            Commands.run(this::intake, this),        // Run intake continuously
            Commands.runOnce(this::stop, this)       // Stop when interrupted
        ).withName("Intake");
    }

    /**
     * Create a command to outtake CORAL (release/score).
     *
     * This command runs the outtake continuously until interrupted.
     *
     * @return Command that runs outtake
     *
     * TYPICAL USAGE:
     *   // Run outtake while button is held
     *   someButton.whileTrue(claw.outtakeCommand());
     */
    public Command outtakeCommand() {
        return Commands.sequence(
            Commands.runOnce(timer::restart, this),  // Reset timer
            Commands.run(this::outtake, this),       // Run outtake continuously
            Commands.runOnce(this::stop, this)       // Stop when interrupted
        ).withName("Outtake");
    }
}  // End of Claw class
