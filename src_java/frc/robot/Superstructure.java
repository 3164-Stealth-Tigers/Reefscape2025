package frc.robot;

/*
 * ========================================================================
 * SUPERSTRUCTURE - Multi-Subsystem Coordinator
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Coordinates multiple subsystems to perform complex actions that require
 * synchronized movement. Think of it as an "orchestra conductor" - it
 * doesn't play any instruments, but it tells all the musicians when and
 * how to play together.
 *
 * WHY DO WE NEED THIS?
 * --------------------
 * Some robot actions need multiple mechanisms to move in coordination:
 *
 *   Example: Scoring at Level 4 (highest level)
 *   1. Elevator needs to go to max height
 *   2. Arm needs to angle upward
 *   3. Robot needs to be at the correct position
 *   4. All this needs to happen smoothly together!
 *
 * [ANALOGY]
 * Imagine reaching for something on a high shelf:
 *   - Your legs (swerve) position you near the shelf
 *   - Your arm (elevator) extends upward
 *   - Your wrist (coral arm) angles to grab
 *   - Your hand (claw) grabs the object
 *
 * The Superstructure is like your brain coordinating all these movements.
 *
 * KINEMATICS:
 * -----------
 * The "end effector" is the claw/gripper at the end of the arm.
 * Its height depends on BOTH the elevator AND the arm angle:
 *
 *                      ╱ Arm (angled)
 *                     ╱
 *   ┌─────────────────╱──────────── End Effector Height
 *   │ Elevator       ●
 *   │    ↕          Claw
 *   │
 *   └───────────────────────────── Floor
 *
 *   End Effector Height = Carriage Height + (Arm Length × sin(angle))
 *
 * NOT A SUBSYSTEM:
 * ----------------
 * This is intentionally NOT a subsystem (doesn't extend SubsystemBase).
 * It's a utility/helper class that creates commands using other subsystems.
 * The individual subsystems own the hardware; Superstructure just coordinates.
 *
 * HOW TO MODIFY:
 * --------------
 * - Add new scoring positions: Create new command methods
 * - Change coordination logic: Modify setEndEffectorHeightCommand()
 * - Add new subsystems to coordinate: Add to constructor
 *
 * QUICK REFERENCE:
 * ----------------
 * → Check if ready to score: superstructure.readyToScore()
 * → Move to height: superstructure.setEndEffectorHeightCommand(height)
 * → Get subsystems: superstructure.getElevator(), getSwerve(), etc.
 *
 * ========================================================================
 */

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * ========================================================================
 * SUPERSTRUCTURE CLASS - Multi-Subsystem Coordinator
 * ========================================================================
 *
 * NOT a subsystem itself - this is a coordinator that creates commands
 * involving multiple subsystems working together.
 *
 * [WHY NOT A SUBSYSTEM?]
 * Subsystems own hardware. This class doesn't own any hardware; it just
 * creates commands that use multiple subsystems. Making it a subsystem
 * would cause issues with command requirements.
 */
public class Superstructure {

    // ========================================================================
    // SUBSYSTEM REFERENCES
    // ========================================================================

    /**
     * Reference to swerve drive (for checking robot velocity and position).
     */
    private final SwerveDrive swerve;

    /**
     * Reference to elevator (for height control).
     */
    private final Elevator elevator;

    /**
     * Reference to coral arm (for angle control).
     */
    private final CoralArm coralArm;

    /**
     * Reference to climber (for end-game climbing).
     */
    private final Climber climber;

    /**
     * Reference to auto-align (for position checking).
     */
    private final AutoAlign autoAlign;

    // ========================================================================
    // CONSTANTS - Thresholds for "ready to score" checks
    // ========================================================================

    /**
     * Maximum acceptable velocity when scoring (m/s).
     * Robot should be nearly stopped to score accurately.
     */
    private static final double MAXIMUM_VELOCITY_ERROR = 0.1;  // 10 cm/s

    /**
     * Maximum acceptable rotation speed when scoring (deg/s).
     * Robot shouldn't be spinning when trying to score.
     */
    private static final double MAXIMUM_ANGULAR_VELOCITY_ERROR = 5.0;  // 5 deg/s

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new Superstructure coordinator.
     *
     * [NOTE]
     * This just stores references to subsystems. It doesn't create them.
     * The subsystems are created in RobotContainer and passed here.
     *
     * @param swerve The swerve drive subsystem
     * @param elevator The elevator subsystem
     * @param coralArm The coral arm subsystem
     * @param climber The climber subsystem
     * @param autoAlign The auto-align subsystem
     */
    public Superstructure(SwerveDrive swerve, Elevator elevator, CoralArm coralArm,
                          Climber climber, AutoAlign autoAlign) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.climber = climber;
        this.autoAlign = autoAlign;
    }

    // ========================================================================
    // STATUS METHODS
    // ========================================================================

    /**
     * Check if the robot is ready to release a CORAL and score.
     *
     * [WHAT WE CHECK]
     * For a successful score, ALL of these must be true:
     *   1. Robot is at the correct position (autoAlign.isAtGoalPose())
     *   2. Robot is nearly stopped (not moving/rotating fast)
     *   3. Elevator is at the correct height
     *   4. Arm is at the correct angle
     *
     * [WHY ALL THESE CHECKS?]
     * If we release the CORAL while moving or not aligned, we'll miss!
     * This ensures we only score when everything is in position.
     *
     * @return True if ready to release CORAL and score
     */
    public boolean readyToScore() {
        // Get current robot speeds from the swerve drive
        // toChassisSpeeds() converts module states to overall robot velocity
        ChassisSpeeds speeds = swerve.getKinematics().toChassisSpeeds(swerve.getModuleStates());

        // Check ALL conditions for scoring readiness
        boolean readyToScore = (
            // 1. Robot is at the correct scoring position on the field
            autoAlign.isAtGoalPose() &&

            // 2. Robot is not moving too fast in X direction
            Math.abs(speeds.vxMetersPerSecond) < MAXIMUM_VELOCITY_ERROR &&

            // 3. Robot is not moving too fast in Y direction
            Math.abs(speeds.vyMetersPerSecond) < MAXIMUM_VELOCITY_ERROR &&

            // 4. Robot is not rotating too fast
            Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond)) < MAXIMUM_ANGULAR_VELOCITY_ERROR &&

            // 5. Elevator has reached the target scoring height
            elevator.atGoalHeight() &&

            // 6. Arm has reached the target scoring angle
            // sam was here ;)
            coralArm.atGoalRotation()
        );

        // Log the result for debugging
        SmartDashboard.putBoolean("Superstructure/ReadyToScore", readyToScore);
        return readyToScore;
    }

    // ========================================================================
    // COORDINATED MOVEMENT COMMANDS
    // ========================================================================

    /**
     * Create a command that moves the claw to a specified height.
     *
     * [CONVENIENCE METHOD]
     * Calls the full method with angle=null to auto-calculate.
     *
     * @param endEffectorHeight The target height for the claw (meters)
     * @return A command that coordinates elevator and arm movement
     */
    public Command setEndEffectorHeightCommand(double endEffectorHeight) {
        return setEndEffectorHeightCommand(endEffectorHeight, null);
    }

    /**
     * Create a command that moves the claw to a specified height and angle.
     *
     * [THE KINEMATICS MATH]
     * The end effector (claw) height is the sum of:
     *   - Carriage height (how high the elevator is)
     *   - Vertical component of arm length (arm_length × sin(angle))
     *
     *                    ╱ Arm
     *                   ╱
     *                  ╱  angle
     *   ┌────────────●────────────
     *   │           ╱│
     *   │ Elevator   │ arm_length × sin(angle)
     *   │           │
     *   │           │
     *   └───────────────────────
     *
     *   end_effector_height = carriage_height + (arm_length × sin(angle))
     *
     * [HOW IT WORKS]
     * 1. If angle not specified, calculate optimal angle:
     *    - If height is reachable with horizontal arm, use horizontal
     *    - If height is higher, angle arm upward
     * 2. Calculate required carriage height from end effector height and angle
     * 3. Move elevator and arm simultaneously
     *
     * @param endEffectorHeight The target height for the claw (meters)
     * @param angle The arm angle (null to auto-calculate)
     * @return A command that coordinates elevator and arm movement
     * @throws IllegalArgumentException if the height is unreachable
     */
    public Command setEndEffectorHeightCommand(double endEffectorHeight, Rotation2d angle) {

        // ================================================================
        // CALCULATE ANGLE (if not provided)
        // ================================================================
        if (angle == null) {
            if (endEffectorHeight <= ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT) {
                // Height is reachable with arm horizontal
                angle = new Rotation2d();  // 0 degrees
            } else {
                // Need to angle arm upward to reach higher
                // height_diff = how much higher than max elevator height
                double heightDiff = endEffectorHeight - ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT;

                // Inverse sine: angle = arcsin(height_diff / arm_length)
                angle = new Rotation2d(Math.asin(heightDiff / CoralArmConstants.ARM_LENGTH));
            }
        }

        // ================================================================
        // CALCULATE CARRIAGE HEIGHT
        // ================================================================
        // Rearranging: carriage = end_effector - (arm_length × sin(angle))
        double carriageHeight = endEffectorHeight - (CoralArmConstants.ARM_LENGTH * angle.getSin());

        // ================================================================
        // VALIDATE - Make sure it's physically possible
        // ================================================================
        if (carriageHeight > ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT ||
            carriageHeight < ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT) {
            throw new IllegalArgumentException(
                "Calculated carriage height is out of bounds: " + carriageHeight + " meters. " +
                "Target height " + endEffectorHeight + "m may be unreachable."
            );
        }

        // Make final copies for lambda (Java requirement)
        final Rotation2d finalAngle = angle;
        final double finalHeight = carriageHeight;

        // ================================================================
        // CREATE PARALLEL COMMAND
        // ================================================================
        // Move elevator AND arm at the same time for smooth motion
        return Commands.parallel(
            coralArm.setAngleCommand(finalAngle),      // Move arm to angle
            elevator.setHeightCommand(finalHeight)     // Move elevator to height
        ).beforeStarting(
            // Log what we're doing (helpful for debugging)
            Commands.print("Setting height: " + finalHeight + "m, angle: " + finalAngle.getDegrees() + "°")
        ).withName("SetEndEffectorHeight");
    }

    // ========================================================================
    // ACCESSORS - Get references to subsystems
    // ========================================================================
    // These let other code access the subsystems through Superstructure

    public SwerveDrive getSwerve() {
        return swerve;
    }

    public Elevator getElevator() {
        return elevator;
    }

    public CoralArm getCoralArm() {
        return coralArm;
    }

    public Climber getClimber() {
        return climber;
    }

    public AutoAlign getAutoAlign() {
        return autoAlign;
    }

}  // End of Superstructure class
