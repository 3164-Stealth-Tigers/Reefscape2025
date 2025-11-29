package frc.robot.subsystems.swerve;

/*
 * ========================================================================
 * SWERVE DRIVE SUBSYSTEM - The Robot's Driving System
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Controls the robot's swerve drive - a special type of drivetrain where
 * each wheel can rotate independently AND steer independently. This allows
 * the robot to move in ANY direction while facing ANY direction!
 *
 * WHAT IS SWERVE DRIVE?
 * ---------------------
 * Traditional drivetrains (like a tank drive) can only go forward/backward
 * and turn by spinning the wheels at different speeds.
 *
 * Swerve drive is like having 4 shopping cart wheels that can each:
 *   1. SPIN (drive motor) - Move the robot
 *   2. ROTATE (azimuth motor) - Point in any direction
 *
 *   Traditional Tank Drive:        Swerve Drive:
 *   ┌─────────────┐                ┌─────────────┐
 *   │ ║         ║ │                │  ⟲       ⟲  │  ← Each wheel can
 *   │ ║   →→→   ║ │  Only forward  │      →      │    point any direction
 *   │ ║         ║ │                │  ⟲       ⟲  │
 *   └─────────────┘                └─────────────┘
 *
 * With swerve, you can:
 *   - Drive forward while facing sideways (strafing)
 *   - Spin in place while moving
 *   - Move diagonally
 *   - "Crab walk" sideways
 *
 * KEY CONCEPTS:
 * -------------
 *
 * 1. SWERVE MODULE - One wheel unit with 2 motors
 *    - Drive motor: Spins the wheel (makes it go)
 *    - Azimuth motor: Rotates the wheel direction (steering)
 *    - Absolute encoder: Knows wheel angle even after power off
 *
 * 2. KINEMATICS - Math that converts robot motion into wheel motions
 *    - Input: "Go forward at 2 m/s while rotating 1 rad/s"
 *    - Output: "FL wheel: 30°, 2.1 m/s | FR wheel: -30°, 2.1 m/s | ..."
 *
 * 3. FIELD-RELATIVE vs ROBOT-RELATIVE
 *    - Field-relative: "Forward" means toward the far end of the field
 *    - Robot-relative: "Forward" means wherever the robot is facing
 *
 *    Example: Robot is facing left, driver pushes "forward"
 *    - Field-relative: Robot moves toward far end (actually right from robot's view)
 *    - Robot-relative: Robot moves left (where it's facing)
 *
 * 4. POSE ESTIMATION - Tracking robot position on the field
 *    - Combines wheel odometry (distance traveled)
 *    - With gyro readings (rotation)
 *    - And optional vision measurements (AprilTags)
 *
 * 5. ODOMETRY - Calculating position from wheel movements
 *    - If wheels moved X distance at Y angle, robot moved to Z position
 *
 * MODULE LAYOUT:
 * --------------
 *       Front
 *   FL ─────── FR     FL = Front Left (index 0)
 *    │         │      FR = Front Right (index 1)
 *    │    ↑    │      RL = Rear Left (index 2)
 *    │  Robot  │      RR = Rear Right (index 3)
 *    │         │
 *   RL ─────── RR
 *        Back
 *
 * HOW TO MODIFY:
 * --------------
 * - Change max speed: Constants.java → SwerveConstants.MAX_SPEED
 * - Change wheel positions: SwerveConstants.WHEEL_BASE / TRACK_WIDTH
 * - Tune module angles: SwerveConstants.XX_ENCODER_OFFSET
 *
 * QUICK REFERENCE:
 * ----------------
 * → Drive the robot: swerve.drive(translation, rotation, fieldRelative, openLoop)
 * → Get current position: swerve.getPose()
 * → Reset gyro: swerve.resetYaw(angle)
 * → Lock wheels (X pattern): swerve.setX()
 *
 * ========================================================================
 */

import java.util.function.DoubleSupplier;

// CTRE library for Pigeon2 gyroscope
import com.ctre.phoenix6.hardware.Pigeon2;

// WPILib math classes for swerve calculations
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// NetworkTables for logging data
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

// WPILib utilities
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

/**
 * ========================================================================
 * SWERVE DRIVE SUBSYSTEM
 * ========================================================================
 *
 * Controls the robot's four-wheel swerve drivetrain. Each wheel (module)
 * can independently spin and steer, allowing omnidirectional movement.
 *
 * This is the most complex subsystem - it coordinates 8 motors
 * (2 per wheel × 4 wheels) plus a gyroscope to move the robot.
 *
 * @see SwerveModule for individual wheel control
 * @see Constants.SwerveConstants for tunable values
 */
public class SwerveDrive extends SubsystemBase {

    // ========================================================================
    // HARDWARE - The four swerve modules (wheels)
    // ========================================================================

    /**
     * Array of all four swerve modules.
     * Index order: [0]=FL, [1]=FR, [2]=RL, [3]=RR
     *
     * Each module contains:
     *   - A drive motor (spins the wheel)
     *   - An azimuth motor (steers the wheel)
     *   - An absolute encoder (tracks wheel angle)
     */
    private final SwerveModule[] modules;

    // ========================================================================
    // SENSORS - Gyroscope for rotation tracking
    // ========================================================================

    /**
     * Pigeon2 gyroscope - Measures the robot's rotation.
     *
     * [WHAT IS A GYRO?]
     * A gyroscope measures angular velocity and orientation.
     * It tells us which way the robot is facing.
     *
     * [WHY WE NEED IT]
     * For field-relative driving: When driver pushes "forward",
     * we need to know which way the robot is facing to calculate
     * which direction is actually "forward" on the field.
     *
     * [HARDWARE] CTRE Pigeon 2.0 (connected via CAN bus)
     */
    private final Pigeon2 gyro;

    // ========================================================================
    // MATH - Kinematics and pose estimation
    // ========================================================================

    /**
     * Kinematics - Converts desired robot motion into individual wheel motions.
     *
     * [WHAT KINEMATICS DOES]
     * Input: "Move robot at 2 m/s forward, 1 m/s left, rotating 0.5 rad/s"
     * Output: Speed and angle for each of the 4 wheels
     *
     * [THE MATH]
     * Each wheel's position relative to the robot center affects how it
     * needs to move. Outer wheels travel farther when turning.
     *
     *     FL ← needs to travel farther       FR
     *      │         ↺                        │
     *      │    (robot turns)                 │
     *     RL                                 RR
     */
    private final SwerveDriveKinematics kinematics;

    /**
     * Pose Estimator - Tracks the robot's position on the field.
     *
     * [WHAT IS POSE?]
     * Pose = Position (X, Y) + Rotation (angle)
     * It tells us exactly where the robot is on the field.
     *
     * [HOW IT WORKS]
     * Combines multiple sources of information:
     *   1. Wheel odometry - How far each wheel has traveled
     *   2. Gyroscope - Which way we're facing
     *   3. Vision (optional) - Camera sees AprilTags
     *
     * The estimator "fuses" these together for a best guess.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    // ========================================================================
    // VISUALIZATION - Dashboard displays
    // ========================================================================

    /**
     * Field2d - Displays robot position on a field diagram in Shuffleboard.
     *
     * [HOW TO VIEW]
     * In Shuffleboard: Look for "Field" widget, shows top-down field view
     * with robot icon showing current estimated position.
     */
    private final Field2d field;

    /**
     * NetworkTables publishers for AdvantageScope.
     *
     * [WHAT IS ADVANTAGESCOPE?]
     * A powerful data visualization tool for FRC.
     * These publishers send structured data that AdvantageScope
     * can display in 3D views, graphs, etc.
     */
    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<SwerveModuleState> statePublisher;

    // ========================================================================
    // STATE - Current driving mode
    // ========================================================================

    /**
     * Field-relative mode flag.
     *
     * true = Field-relative (default): "Forward" is toward the far end of field
     * false = Robot-relative: "Forward" is wherever the robot is facing
     *
     * [DRIVER PREFERENCE]
     * Most drivers prefer field-relative because:
     *   - Pushing forward ALWAYS moves toward the opponent's side
     *   - Doesn't matter which way robot is facing
     *   - More intuitive for strafing
     */
    private boolean fieldRelative = true;

    /**
     * Open-loop vs closed-loop control.
     *
     * Open-loop: Set power directly (faster response, less accurate)
     * Closed-loop: Use velocity PID (more accurate, slight delay)
     *
     * [WHEN TO USE WHICH]
     * - Teleop: Often open-loop for responsiveness
     * - Auto: Often closed-loop for precision
     */
    private boolean openLoop = SwerveConstants.DRIVE_OPEN_LOOP_RAMP > 0;

    // ========================================================================
    // CONSTRUCTOR - Initialize all swerve drive components
    // ========================================================================

    /**
     * Creates a new SwerveDrive subsystem.
     *
     * This constructor:
     *   1. Initializes the gyroscope
     *   2. Defines where each wheel is located on the robot
     *   3. Creates the kinematics object
     *   4. Creates all four swerve modules
     *   5. Sets up pose estimation
     *   6. Initializes visualization tools
     */
    public SwerveDrive() {
        // ----------------------------------------------------------------
        // STEP 1: Initialize the gyroscope
        // ----------------------------------------------------------------
        // The Pigeon2 needs to know its CAN ID
        gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
        gyro.reset();  // Set current heading as "0 degrees"

        // ----------------------------------------------------------------
        // STEP 2: Define where each module is located on the robot
        // ----------------------------------------------------------------
        // Positions are relative to the robot's center
        // X = forward/backward (positive = forward)
        // Y = left/right (positive = left)
        //
        // Think of it like a coordinate system:
        //
        //          +X (forward)
        //           ↑
        //           │
        //    +Y ←───┼───→ -Y
        //     (left)│    (right)
        //           │
        //           ↓
        //          -X (backward)
        //
        // WHEEL_BASE = front-to-back distance
        // TRACK_WIDTH = left-to-right distance
        Translation2d[] moduleLocations = {
            // Front Left: forward half, left half
            new Translation2d(SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),
            // Front Right: forward half, right half (negative Y)
            new Translation2d(SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2),
            // Rear Left: backward half (negative X), left half
            new Translation2d(-SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),
            // Rear Right: backward half, right half
            new Translation2d(-SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2)
        };

        // ----------------------------------------------------------------
        // STEP 3: Create the kinematics object
        // ----------------------------------------------------------------
        // Kinematics uses module positions to calculate how each wheel
        // should move for a given robot motion
        kinematics = new SwerveDriveKinematics(moduleLocations);

        // ----------------------------------------------------------------
        // STEP 4: Create all four swerve modules
        // ----------------------------------------------------------------
        // Each module needs:
        //   - Index (0-3 for FL, FR, RL, RR)
        //   - Drive motor CAN ID
        //   - Azimuth (steering) motor CAN ID
        //   - CANCoder ID (absolute encoder for wheel angle)
        //   - Encoder offset (calibration value - what angle is "straight")
        modules = new SwerveModule[] {
            // Front Left module
            new SwerveModule(0, SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_AZIMUTH_ID,
                           SwerveConstants.FL_CANCODER_ID, SwerveConstants.FL_ENCODER_OFFSET),
            // Front Right module
            new SwerveModule(1, SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_AZIMUTH_ID,
                           SwerveConstants.FR_CANCODER_ID, SwerveConstants.FR_ENCODER_OFFSET),
            // Rear Left module
            new SwerveModule(2, SwerveConstants.RL_DRIVE_ID, SwerveConstants.RL_AZIMUTH_ID,
                           SwerveConstants.RL_CANCODER_ID, SwerveConstants.RL_ENCODER_OFFSET),
            // Rear Right module
            new SwerveModule(3, SwerveConstants.RR_DRIVE_ID, SwerveConstants.RR_AZIMUTH_ID,
                           SwerveConstants.RR_CANCODER_ID, SwerveConstants.RR_ENCODER_OFFSET)
        };

        // ----------------------------------------------------------------
        // STEP 5: Create the pose estimator
        // ----------------------------------------------------------------
        // The pose estimator tracks our position on the field
        // It needs: kinematics, initial gyro angle, initial wheel positions, initial pose
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),              // Current gyro reading
            getModulePositions(),  // Current wheel positions
            new Pose2d()           // Start at origin (0, 0) facing forward
        );

        // ----------------------------------------------------------------
        // STEP 6: Initialize visualization tools
        // ----------------------------------------------------------------
        // Field2d shows robot position in Shuffleboard
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // NetworkTables publishers for AdvantageScope
        // These allow more advanced visualization and logging
        var table = NetworkTableInstance.getDefault().getTable("Swerve");
        posePublisher = table.getStructTopic("Pose", Pose2d.struct).publish();
        statePublisher = table.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    }

    // ========================================================================
    // PERIODIC - Updates every robot loop (50 times per second)
    // ========================================================================

    /**
     * Called periodically (every 20ms) to update odometry and logging.
     *
     * This method:
     *   1. Updates pose estimation using latest sensor data
     *   2. Updates the field visualization
     *   3. Publishes data to NetworkTables for logging
     *   4. Logs individual module states
     */
    @Override
    public void periodic() {
        // Update pose estimator with latest gyro and wheel data
        // This is how we track where the robot is on the field
        poseEstimator.update(getYaw(), getModulePositions());

        // Update the Field2d visualization in Shuffleboard
        field.setRobotPose(getPose());

        // Publish to NetworkTables for AdvantageScope
        posePublisher.set(getPose());
        statePublisher.set(getModuleStates());

        // Log individual module data to SmartDashboard
        // Useful for debugging wheel alignment and behavior
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Swerve/Module" + i + "/Angle", modules[i].getAngle().getDegrees());
            SmartDashboard.putNumber("Swerve/Module" + i + "/Velocity", modules[i].getVelocity());
        }
    }

    // ========================================================================
    // DRIVE METHODS - Control robot movement
    // ========================================================================

    /**
     * Drive the robot with the given velocities.
     *
     * This is the main driving method used by teleop commands.
     *
     * @param translation Translation velocity in m/s
     *                    X = forward/backward (positive = forward)
     *                    Y = left/right (positive = left)
     * @param rotation Rotational velocity in RADIANS per second
     *                 Positive = counter-clockwise
     * @param fieldRelative true = field-relative driving
     *                      false = robot-relative driving
     * @param openLoop true = direct power control (faster response)
     *                 false = velocity PID control (more accurate)
     *
     * EXAMPLE:
     *   // Drive forward at 2 m/s, strafe left at 1 m/s, rotate at 0.5 rad/s
     *   drive(new Translation2d(2.0, 1.0), 0.5, true, false);
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            // FIELD-RELATIVE: Adjust for robot's current heading
            // "Forward" means toward the far end of the field, regardless of robot orientation
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),  // Field X velocity
                translation.getY(),  // Field Y velocity
                rotation,            // Rotation velocity
                getYaw()             // Current robot heading (to convert field→robot)
            );
        } else {
            // ROBOT-RELATIVE: No adjustment needed
            // "Forward" means wherever the robot is facing
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        // Pass to the ChassisSpeeds version of drive()
        drive(speeds, openLoop);
    }

    /**
     * Drive the robot with the given chassis speeds.
     *
     * This is the lower-level drive method that actually commands the modules.
     *
     * @param speeds The desired chassis speeds (vx, vy, omega)
     * @param openLoop Whether to use open-loop control
     */
    public void drive(ChassisSpeeds speeds, boolean openLoop) {
        // DISCRETIZE: Compensate for the discrete nature of robot code
        // Without this, the robot can "drift" when rotating and translating
        // at the same time. Discretize adjusts for the 20ms time step.
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // CONVERT TO MODULE STATES: Use kinematics to calculate what each wheel should do
        // Input: "Robot should move at these speeds"
        // Output: "Each wheel should be at this angle going this fast"
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // DESATURATE: If any wheel would exceed max speed, scale ALL wheels down
        // This preserves the intended motion direction while staying within limits
        // Example: If one wheel needs 5 m/s but max is 4, scale all wheels by 4/5
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);

        // COMMAND EACH MODULE: Send the calculated state to each wheel
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i], openLoop);
        }
    }

    /**
     * Set the modules to an X pattern to resist pushing.
     *
     * This is called "ski stop" or "X-lock". The wheels point inward
     * forming an X shape, which makes it very hard for other robots
     * to push us. Great for defense!
     *
     *   X PATTERN:
     *      ╲   ╱
     *       ╲ ╱
     *        X
     *       ╱ ╲
     *      ╱   ╲
     */
    public void setX() {
        // Set each wheel to point toward/away from center at 45° angles
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);   // FL
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);  // FR
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);  // RL
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);   // RR
    }

    /**
     * Stop all modules (set speed to 0).
     */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    // ========================================================================
    // GYRO METHODS - Read and control robot rotation
    // ========================================================================

    /**
     * Get the current yaw angle from the gyro.
     *
     * YAW = rotation around the vertical axis (spinning like a top)
     * 0° typically means facing the far end of the field.
     *
     * @return The current yaw as a Rotation2d
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * Reset the gyro to a specific angle.
     *
     * [WHEN TO USE]
     * - At the start of a match to set the correct heading
     * - After repositioning the robot manually
     * - If the gyro drifts (rare with Pigeon2)
     *
     * @param angle The angle to set as the current heading
     *
     * EXAMPLE:
     *   resetYaw(Rotation2d.fromDegrees(180));  // Robot is facing our driver station
     */
    public void resetYaw(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    // ========================================================================
    // POSE METHODS - Robot position on the field
    // ========================================================================

    /**
     * Get the current estimated pose (position + rotation).
     *
     * [WHAT IS A POSE?]
     * Pose2d contains:
     *   - X position (meters from origin, typically your alliance wall)
     *   - Y position (meters from origin)
     *   - Rotation (which way robot is facing)
     *
     * @return The current estimated pose on the field
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the pose estimator to a specific pose.
     *
     * [WHEN TO USE]
     * - At the start of autonomous to set initial position
     * - After a vision system gives us a very confident reading
     *
     * @param pose The pose to set as current position
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Add a vision measurement to the pose estimator.
     *
     * [WHAT IS THIS?]
     * Vision systems (cameras with AprilTags) can give us position estimates.
     * We "add" these measurements to the estimator, which combines them
     * with wheel odometry for a better overall estimate.
     *
     * [WHY TIMESTAMP?]
     * The vision measurement might be from slightly in the past
     * (camera processing takes time). The timestamp helps the
     * estimator account for this delay.
     *
     * @param pose The measured pose from vision
     * @param timestampSeconds When the measurement was taken (FPGA timestamp)
     */
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    // ========================================================================
    // MODULE STATE GETTERS - Read wheel data
    // ========================================================================

    /**
     * Get the positions of all modules.
     *
     * Position = cumulative distance traveled by each wheel
     * Used for odometry (tracking how far we've moved)
     *
     * @return Array of 4 module positions [FL, FR, RL, RR]
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    /**
     * Get the states of all modules.
     *
     * State = current speed + current angle of each wheel
     * Used for logging and visualization
     *
     * @return Array of 4 module states [FL, FR, RL, RR]
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Get the kinematics object.
     *
     * [WHY EXPOSE THIS?]
     * Other systems (like autonomous path following) need the kinematics
     * to convert robot speeds to wheel speeds.
     *
     * @return The SwerveDriveKinematics object
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    // ========================================================================
    // DRIVE MODE CONTROL
    // ========================================================================

    /**
     * Toggle field-relative driving mode.
     *
     * Called when the driver presses a button to switch modes.
     */
    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    /**
     * Get whether field-relative mode is enabled.
     *
     * @return true if field-relative, false if robot-relative
     */
    public boolean isFieldRelative() {
        return fieldRelative;
    }

    // ========================================================================
    // COMMANDS - Actions for teleop and auto
    // ========================================================================

    /**
     * Create a teleop drive command.
     *
     * This is the main command that runs during teleop.
     * It reads joystick inputs and drives the robot.
     *
     * @param forward Supplier for forward velocity (-1 to 1, from joystick)
     * @param strafe Supplier for strafe velocity (-1 to 1, from joystick)
     * @param turn Supplier for turn velocity (-1 to 1, from joystick)
     * @return A command that continuously drives the robot
     *
     * [HOW IT WORKS]
     * 1. Read joystick values (suppliers return -1 to 1)
     * 2. Multiply by max speed to get actual velocity
     * 3. Flip for red alliance (field is mirrored)
     * 4. Send to drive() method
     *
     * [WHY SUPPLIERS?]
     * DoubleSupplier is like a function that returns a double.
     * This lets us pass in "how to get joystick value" rather than
     * the value itself, so it updates every loop.
     */
    public Command teleopCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return new RunCommand(() -> {
            // Convert joystick (-1 to 1) to velocity (m/s or rad/s)
            double vx = forward.getAsDouble() * SwerveConstants.MAX_SPEED;
            double vy = strafe.getAsDouble() * SwerveConstants.MAX_SPEED;
            double omega = turn.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;

            // ALLIANCE FLIP: On red alliance, the field is mirrored
            // So we flip the X and Y velocities
            // This way, "forward" always means "toward opponent's side"
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                vx = -vx;
                vy = -vy;
            }

            // Drive the robot!
            drive(new Translation2d(vx, vy), omega, fieldRelative, openLoop);
        }, this);  // "this" = SwerveDrive is required by this command
    }

    /**
     * Create a command to set modules to X pattern (ski stop).
     *
     * When this command runs, the wheels lock in an X pattern
     * making the robot very hard to push. Great for defense!
     *
     * @return A command that continuously holds the X pattern
     */
    public Command skiStopCommand() {
        return new RunCommand(this::setX, this);
    }

    /**
     * Create a command to reset the gyro to 180 degrees.
     *
     * [WHEN TO USE]
     * When the robot starts a match facing YOUR driver station,
     * you're looking at the robot from behind. From the robot's
     * perspective, it's facing 180° (toward you).
     *
     * Press this at the start of a match when lined up.
     *
     * @return A command that resets the gyro (runs once, instantly)
     */
    public Command resetGyroCommand() {
        return runOnce(() -> resetYaw(Rotation2d.fromDegrees(180)));
    }
}  // End of SwerveDrive class
