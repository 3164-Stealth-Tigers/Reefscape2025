package frc.robot.subsystems;

/*
 * ========================================================================
 * AUTO-ALIGN SUBSYSTEM - Automated Positioning for Scoring
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Handles automatic alignment of the robot to scoring positions on the REEF.
 * When the driver selects a reef position (A-L), this subsystem calculates
 * where the robot needs to be and drives there automatically.
 *
 * Think of it like a "GPS navigation" system for the robot - you tell it
 * where you want to score, and it figures out how to get there.
 *
 * KEY FEATURES:
 * -------------
 * 1. POSE CALCULATION - Figures out exact position to score at each reef spot
 * 2. COLLISION DETECTION - Checks if path would hit the reef structure
 * 3. AUTO-DRIVE - Uses PID control to drive to calculated positions
 *
 * THE REEF STRUCTURE:
 * -------------------
 * The REEF is a hexagonal structure in the center of the field with 12
 * scoring positions labeled A through L:
 *
 *              G   H
 *           F         I
 *          E    REEF   J
 *           D         K
 *              C   L
 *            B       A
 *
 * Each position (pipe) has:
 *   - A location on the field (X, Y coordinates)
 *   - A rotation (which way the robot should face)
 *   - An approach position (where to line up before final approach)
 *   - A scoring position (final position against the reef)
 *
 * COORDINATE SYSTEM:
 * ------------------
 *   - X = Distance along the long side of the field
 *   - Y = Distance along the short side of the field
 *   - Origin (0,0) = Corner of your alliance's side
 *   - Angles: 0° = Facing opponent's alliance wall
 *
 * COLLISION DETECTION:
 * --------------------
 * Before driving to a position, we check if the straight-line path would
 * hit the reef. This uses circle-circle collision math:
 *   - Robot is approximated as a circle (half the width)
 *   - Reef is approximated as a circle (REEF_RADIUS)
 *   - We check if these circles would overlap during the path
 *
 * HOW TO MODIFY:
 * --------------
 * - Change approach distances: REEF_WALL_TO_BUMPER_DISTANCE_APPROACH
 * - Change scoring distances: REEF_WALL_TO_BUMPER_DISTANCE_FINAL
 * - Adjust tolerance: MAXIMUM_POSITION_ERROR, MAXIMUM_ANGULAR_POSITION_ERROR
 *
 * QUICK REFERENCE:
 * ----------------
 * → Get scoring position: AutoAlign.getRobotScoringPose("A")
 * → Check if at goal: autoAlign.isAtGoalPose()
 * → Check collision: autoAlign.willCollideWithReef()
 *
 * ========================================================================
 */

import java.util.Set;

// PathPlanner library for trajectory following
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

// WPILib geometry classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants and utilities
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotPhysicalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.CoralStation;

/**
 * ========================================================================
 * AUTO-ALIGN SUBSYSTEM
 * ========================================================================
 *
 * Provides automated positioning for scoring on the reef and collecting
 * from coral stations. Uses PID control to drive to calculated positions
 * and includes collision detection to avoid hitting the reef structure.
 *
 * @see FieldUtil for reef position calculations
 * @see Constants.FieldConstants for field measurements
 */
public class AutoAlign extends SubsystemBase {

    // ========================================================================
    // HARDWARE - Reference to swerve drive
    // ========================================================================

    /**
     * Reference to the swerve drive subsystem.
     * We need this to:
     *   1. Get the robot's current position (getPose())
     *   2. Send drive commands to move to the goal
     */
    private final SwerveDrive swerve;

    // ========================================================================
    // STATE - Current goal and settings
    // ========================================================================

    /**
     * The target position we're trying to reach.
     *
     * [WHAT IS A POSE?]
     * A Pose2d contains:
     *   - X position (meters along field length)
     *   - Y position (meters along field width)
     *   - Rotation (which way the robot is facing)
     *
     * This is set when a driver selects a reef position (A-L).
     */
    private Pose2d goalPose = new Pose2d();

    /**
     * Whether to use the "close" behavior (wait until near target for actions).
     * When true, certain actions (like extending arm) wait until robot is close.
     */
    private boolean useClose = true;

    // ========================================================================
    // CONSTANTS - Tunable positioning values
    // ========================================================================

    /**
     * How close the robot must be (in meters) before "close" actions can run.
     * 0.5 meters = about 20 inches
     *
     * [WHY THIS MATTERS]
     * We don't want the arm to extend while far from the reef - it could
     * hit something. So we wait until "close enough" before starting
     * scoring motions.
     */
    private static final double CLOSE_RADIUS = 0.5;

    /**
     * Enable/disable the "ready for close" check.
     * Set to false for testing if you want to skip the distance check.
     */
    private static final boolean USE_READY_FOR_CLOSE = true;

    /**
     * Position tolerance (meters) - how close is "close enough"?
     * 0.05 meters = 5 centimeters = about 2 inches
     *
     * [HOW THIS IS USED]
     * isAtGoalPose() returns true when within this distance.
     * If robot keeps oscillating, increase this value.
     */
    private static final double MAXIMUM_POSITION_ERROR = 0.05;

    /**
     * Angular tolerance (degrees) - how aligned is "aligned enough"?
     * 3 degrees is pretty tight - may need to increase if jittery.
     */
    private static final double MAXIMUM_ANGULAR_POSITION_ERROR = 3.0;

    /**
     * Distance from reef wall to robot bumper when scoring.
     * 0.02 meters = 2 centimeters = basically touching
     *
     * [SCORING vs APPROACH]
     * We use two distances:
     *   - FINAL = where we actually score (very close)
     *   - APPROACH = where we line up first (further back)
     *
     *           APPROACH        FINAL
     *              ↓              ↓
     *   Robot → [    ] ------→ [==] REEF
     *           15cm back       2cm back
     */
    private static final double REEF_WALL_TO_BUMPER_DISTANCE_FINAL = 0.02;

    /**
     * Distance from reef wall to robot bumper when approaching.
     * 0.15 meters = 15 centimeters = about 6 inches
     *
     * The robot drives here first, then moves in for final scoring.
     */
    private static final double REEF_WALL_TO_BUMPER_DISTANCE_APPROACH = 0.15;

    /**
     * Distance from coral station wall to robot bumper when intaking.
     * Similar to reef scoring distance.
     */
    private static final double CORAL_STATION_WALL_TO_BUMPER_DISTANCE = 0.02;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new AutoAlign subsystem.
     *
     * [SIMPLE CONSTRUCTOR]
     * This subsystem doesn't have its own hardware - it just calculates
     * positions and tells the swerve drive where to go.
     *
     * @param swerve The swerve drive subsystem (needed for position and driving)
     */
    public AutoAlign(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    // ========================================================================
    // PERIODIC - Runs every robot loop (50 times per second)
    // ========================================================================

    @Override
    public void periodic() {
        // Log status to SmartDashboard for debugging
        // You can see these values in Shuffleboard or the Driver Station

        // Will the robot hit the reef if it drives straight to the goal?
        SmartDashboard.putBoolean("AutoAlign/WillCollide", willCollideWithReef());

        // Has the robot reached its target position?
        SmartDashboard.putBoolean("AutoAlign/AtGoal", isAtGoalPose());

        // Is the "close wait" feature enabled?
        SmartDashboard.putBoolean("AutoAlign/UseClose", useClose);

        // Is the robot close enough for scoring actions to start?
        SmartDashboard.putBoolean("AutoAlign/ReadyForClose", isReadyForClose());
    }

    // ========================================================================
    // STATUS METHODS - Check if we're ready/at position
    // ========================================================================

    /**
     * Check if the robot is close enough to its scoring position for close actions.
     *
     * [WHY WE NEED THIS]
     * Some actions (like extending the arm) should only happen when
     * the robot is close to the reef. This prevents collisions during
     * the approach phase.
     *
     * [LOGIC]
     * Returns true if:
     *   - Robot is within CLOSE_RADIUS of the goal, OR
     *   - USE_READY_FOR_CLOSE is disabled (testing mode), OR
     *   - useClose is disabled (skip the check)
     *
     * @return True if ready for close actions
     */
    public boolean isReadyForClose() {
        // Calculate distance from current position to goal position
        boolean ready = goalPose.getTranslation().getDistance(swerve.getPose().getTranslation()) < CLOSE_RADIUS;

        // Return true if close, OR if the feature is disabled
        return ready || !USE_READY_FOR_CLOSE || !useClose;
    }

    /**
     * Check if the current path to the goal will collide with the reef.
     *
     * [HOW IT WORKS]
     * Draws an imaginary straight line from current position to goal.
     * Checks if this line would pass through the reef's circular area.
     *
     *   Current ─────────────→ Goal
     *              ┌───┐
     *              │REEF│  ← Would we hit this?
     *              └───┘
     *
     * [WHEN TO USE]
     * Before starting auto-align, check this to warn the driver if they
     * need to manually move around the reef first.
     *
     * @return True if a collision would occur
     */
    public boolean willCollideWithReef() {
        // Get the reef center, flipped for our alliance
        Translation2d reefCenter = FieldUtil.flipAlliance(
            new Translation2d(FieldConstants.REEF_CENTER_X, FieldConstants.REEF_CENTER_Y)
        );

        // Check collision between robot (moving circle) and reef (stationary circle)
        return willCollide(
            swerve.getPose().getTranslation(),                  // Where we are
            goalPose.getTranslation(),                          // Where we want to go
            reefCenter,                                          // Reef location
            RobotPhysicalConstants.ROBOT_WIDTH_WITH_BUMPERS / 2, // Robot "radius"
            FieldConstants.REEF_RADIUS                           // Reef radius
        );
    }

    /**
     * Check if the robot is at the goal pose (within tolerance).
     *
     * [WHAT WE CHECK]
     * Two things must be true:
     *   1. Position error < MAXIMUM_POSITION_ERROR (5 cm)
     *   2. Angle error < MAXIMUM_ANGULAR_POSITION_ERROR (3 degrees)
     *
     * [POSITION ERROR]
     * How far away we are from the target (in meters).
     * Uses Euclidean distance: √((x2-x1)² + (y2-y1)²)
     *
     * [ANGULAR ERROR]
     * How much we need to rotate to face the right direction.
     * The rotation "minus" handles angle wrapping (359° to 1° = 2°, not 358°).
     *
     * @return True if at the goal pose within tolerance
     */
    public boolean isAtGoalPose() {
        Pose2d currentPose = swerve.getPose();

        // Calculate distance from current position to goal position
        double positionError = goalPose.getTranslation().getDistance(currentPose.getTranslation());

        // Calculate angle difference (handles wrapping automatically)
        double angularError = Math.abs(goalPose.getRotation().minus(currentPose.getRotation()).getDegrees());

        // Both must be within tolerance
        return positionError < MAXIMUM_POSITION_ERROR && angularError < MAXIMUM_ANGULAR_POSITION_ERROR;
    }

    // ========================================================================
    // GETTERS AND SETTERS
    // ========================================================================

    /**
     * Get the current goal pose.
     *
     * @return The position we're trying to reach
     */
    public Pose2d getGoalPose() {
        return goalPose;
    }

    /**
     * Set the goal pose directly.
     *
     * [WHEN TO USE]
     * Usually you'd use getRobotScoringPose() to calculate the goal,
     * then pass it here. The DriveToScoringPositionCommand does this
     * automatically.
     *
     * @param pose The new target position
     */
    public void setGoalPose(Pose2d pose) {
        this.goalPose = pose;
    }

    // ========================================================================
    // POSE CALCULATION - Figure out where to position the robot
    // ========================================================================

    /**
     * Calculate the robot's scoring pose for a given reef position.
     *
     * [WHAT THIS CALCULATES]
     * Given a reef position label (like "A" or "G"), this figures out
     * exactly where the robot needs to be to score at that position.
     *
     * [THE CALCULATION]
     *
     *    REEF PIPE
     *        ║
     *        ║
     *        ║←─ pipeTranslation (where the pipe is)
     *        ║
     *   ╔════╬════╗
     *   ║         ║←─ Robot center needs to be here
     *   ║  ROBOT  ║
     *   ║         ║
     *   ╚═════════╝
     *   └──────────┘
     *     half robot length + gap
     *
     * We start with the pipe location, then transform (move) backward
     * by half the robot length plus a small gap.
     *
     * @param position The reef position label (e.g., "A", "B", ... "L")
     * @return The target pose (position + rotation) for scoring
     */
    public static Pose2d getRobotScoringPose(String position) {
        // Get the (X, Y) location of this reef pipe
        Translation2d pipeTranslation = FieldUtil.getReefPipeTranslation(position);

        // Get which direction the robot should face at this position
        // Each reef face has a specific angle (e.g., positions A/B face one way,
        // positions C/D face another way, etc.)
        double rotationDegrees = FieldUtil.getReefRotation(position);
        Rotation2d rotation = FieldUtil.flipAlliance(Rotation2d.fromDegrees(rotationDegrees));

        // Create a pose at the pipe location
        Pose2d robotPose = new Pose2d(pipeTranslation, rotation);

        // Transform (move) the robot backward from the pipe
        // The robot's CENTER needs to be:
        //   (half robot length) + (gap from reef wall) away from the pipe
        //
        // transformBy with negative X moves backward relative to the robot's facing
        robotPose = robotPose.transformBy(new Transform2d(
            -RobotPhysicalConstants.ROBOT_LENGTH_WITH_BUMPERS / 2 - REEF_WALL_TO_BUMPER_DISTANCE_FINAL,
            0,               // No sideways offset
            new Rotation2d() // No rotation change
        ));

        return robotPose;
    }

    /**
     * Calculate the robot's approach pose for a given reef position.
     *
     * [WHY TWO POSES?]
     * We use a two-step approach to scoring:
     *   1. Drive to APPROACH pose (15cm away) - line up
     *   2. Drive to SCORING pose (2cm away) - final push
     *
     * This is safer and more reliable than trying to go directly
     * to the final position from far away.
     *
     *   [APPROACH]     [SCORING]
     *       ↓             ↓
     *   ╔═══╗         ╔═══╗
     *   ║   ║ ───→    ║   ║ REEF
     *   ╚═══╝         ╚═══╝
     *   15cm away     2cm away
     *
     * @param position The reef position label ("A" through "L")
     * @return The approach pose (further from reef than scoring pose)
     */
    public static Pose2d getRobotApproachPose(String position) {
        // Start with the scoring pose
        Pose2d scoringPose = getRobotScoringPose(position);

        // Move backward by the difference between approach and final distances
        // Approach is 15cm, Final is 2cm, so we move back 13cm more
        return scoringPose.transformBy(new Transform2d(
            -(REEF_WALL_TO_BUMPER_DISTANCE_APPROACH - REEF_WALL_TO_BUMPER_DISTANCE_FINAL),
            0,               // No sideways offset
            new Rotation2d() // No rotation change
        ));
    }

    /**
     * Calculate the robot's intake pose for a coral station.
     *
     * [WHAT ARE CORAL STATIONS?]
     * Coral stations are where human players feed game pieces to the robot.
     * There are stations on both sides of the field (LEFT and RIGHT).
     *
     * [THE CALCULATION]
     * Similar to scoring poses - we take the station location and
     * offset backward so the robot lines up correctly for intake.
     *
     * @param station The coral station (LEFT or RIGHT)
     * @return The intake pose for collecting from that station
     */
    public static Pose2d getRobotIntakePose(CoralStation station) {
        // Get the station's pose, flipped for our alliance
        Pose2d stationPose = FieldUtil.flipAlliance(station.getPose());

        // Offset backward so robot center is in the right spot
        return stationPose.transformBy(new Transform2d(
            -RobotPhysicalConstants.ROBOT_LENGTH_WITH_BUMPERS / 2 - CORAL_STATION_WALL_TO_BUMPER_DISTANCE,
            0,               // No sideways offset
            new Rotation2d() // No rotation change
        ));
    }

    // ========================================================================
    // COLLISION DETECTION - Check if path hits the reef
    // ========================================================================

    /**
     * Check if a moving circle will collide with a stationary circle.
     *
     * [THE MATH EXPLAINED]
     * We model the robot as a circle moving from point A to point B.
     * The reef is a stationary circle at point C.
     * We want to know: do these circles ever overlap during the path?
     *
     *   Moving circle A (robot):
     *
     *     Start ○───────────────○ End
     *           aInitial       aFinal
     *
     *               ○ Stationary circle B (reef)
     *
     * [PARAMETERIZED PATH]
     * We express the robot's position along the path as:
     *   P(t) = aInitial + t * (aFinal - aInitial)
     *
     * Where t goes from 0 (at start) to 1 (at end).
     *
     * [COLLISION CONDITION]
     * Circles overlap when distance between centers < sum of radii.
     * We solve for when: distance(P(t), b) = radiusA + radiusB
     *
     * This becomes a quadratic equation: At² + Bt + C = 0
     * If solutions exist AND are between 0 and 1, collision will occur.
     *
     * @param aInitial Starting position of moving circle (robot start)
     * @param aFinal Ending position of moving circle (robot goal)
     * @param b Position of stationary circle (reef center)
     * @param radiusA Radius of moving circle (half robot width)
     * @param radiusB Radius of stationary circle (reef radius)
     * @return True if collision will occur during the path
     */
    public static boolean willCollide(Translation2d aInitial, Translation2d aFinal,
                                      Translation2d b, double radiusA, double radiusB) {

        // Calculate the direction vector (how far we're moving)
        double dx = aFinal.getX() - aInitial.getX();  // X component of motion
        double dy = aFinal.getY() - aInitial.getY();  // Y component of motion

        // ----------------------------------------------------------------
        // QUADRATIC EQUATION SETUP
        // ----------------------------------------------------------------
        // We're solving: |P(t) - b|² = (radiusA + radiusB)²
        // Expanding this gives us: At² + Bt + C = 0

        // A coefficient: related to path length squared
        double A = dx * dx + dy * dy;

        // B coefficient: related to initial position relative to obstacle
        double B = 2 * ((aInitial.getX() - b.getX()) * dx + (aInitial.getY() - b.getY()) * dy);

        // C coefficient: initial distance squared minus collision distance squared
        double C = Math.pow(aInitial.getX() - b.getX(), 2)
                 + Math.pow(aInitial.getY() - b.getY(), 2)
                 - Math.pow(radiusA + radiusB, 2);

        // Discriminant tells us if solutions exist
        // discriminant < 0: no real solutions (circles never touch)
        // discriminant = 0: one solution (circles just touch at one point)
        // discriminant > 0: two solutions (circles overlap for a range)
        double discriminant = B * B - 4 * A * C;

        // If A = 0, robot isn't moving (start = end), no collision possible
        if (A == 0) {
            return false;
        }

        // ----------------------------------------------------------------
        // SOLVE AND CHECK SOLUTIONS
        // ----------------------------------------------------------------

        // If discriminant is negative, no real solutions = no collision
        if (discriminant >= 0) {
            // Quadratic formula: t = (-B ± √discriminant) / (2A)
            double t1 = (-B + Math.sqrt(discriminant)) / (2 * A);
            double t2 = (-B - Math.sqrt(discriminant)) / (2 * A);

            // Check if BOTH solutions are within our path (t ∈ [0, 1])
            // t=0 is start, t=1 is end
            // If both are in range, the path passes through the obstacle
            if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1) {
                return true;
            }
        }

        // No collision detected
        return false;
    }

    // ========================================================================
    // COMMAND UTILITIES
    // ========================================================================

    /**
     * Wrap a command to wait until close to the reef before executing.
     *
     * [WHY THIS EXISTS]
     * Some scoring actions (like extending the arm) shouldn't start until
     * the robot is close to the reef. This wrapper adds that "wait" logic
     * to any command.
     *
     * [EXAMPLE]
     * Instead of running armExtendCommand immediately, we can do:
     *   autoAlign.closeCommand(armExtendCommand)
     *
     * This waits until isReadyForClose() is true, THEN runs the arm command.
     *
     * @param cmd The command to wrap
     * @return A new command that waits for proximity, then runs the original
     */
    public Command closeCommand(Command cmd) {
        return Commands.waitUntil(this::isReadyForClose)  // Wait until close
            .andThen(cmd)                                   // Then run the command
            .withName("CloseWait(" + cmd.getName() + ")");  // Descriptive name
    }

    // ========================================================================
    // COMMAND CLASSES - Actions for auto-alignment
    // ========================================================================
    //
    // [WHAT ARE INNER COMMAND CLASSES?]
    // These are Command classes defined INSIDE the AutoAlign class.
    // This is a common FRC pattern - keeps related code together.
    //
    // Why "static"?
    //   - They don't need access to AutoAlign's instance methods
    //   - Can be created without an AutoAlign instance
    //   - Actually, wait - they DO need autoAlign, so they're passed in constructor
    //
    // [HOW TO USE]
    // To create and schedule one of these commands:
    //   new AutoAlign.DriveToScoringPositionCommand(autoAlign, "A").schedule()
    //
    // Or bind it to a button:
    //   someButton.onTrue(new AutoAlign.DriveToScoringPositionCommand(autoAlign, "A"))
    //
    // ========================================================================

    /**
     * Command to drive the robot to a scoring position on the reef.
     *
     * [WHAT IT DOES]
     * When scheduled, this command:
     *   1. Calculates where the robot needs to be for the selected reef position
     *   2. Uses a PID controller to drive there
     *   3. Keeps running until interrupted (doesn't finish on its own)
     *
     * [THE CONTROLLER]
     * PPHolonomicDriveController is from PathPlanner library.
     * "Holonomic" means it can control X, Y, and rotation independently
     * (which is what swerve drive can do - move any direction while rotating).
     *
     * [TYPICAL USAGE]
     *   // When button pressed, drive to position A
     *   buttonA.onTrue(new DriveToScoringPositionCommand(autoAlign, "A"));
     */
    public static class DriveToScoringPositionCommand extends Command {

        // References to subsystems we need
        private final AutoAlign autoAlign;
        private final SwerveDrive swerve;

        // Which reef position we're targeting (A-L)
        private final String label;

        // The PID controller that calculates how fast to drive
        // (PathPlanner's holonomic controller handles X, Y, and theta)
        private final PPHolonomicDriveController controller;

        // The target state we're driving to (position + velocity)
        private PathPlannerTrajectoryState target;

        /**
         * Create a new DriveToScoringPositionCommand.
         *
         * @param autoAlign The AutoAlign subsystem (for goal tracking)
         * @param label The reef position to drive to ("A" through "L")
         */
        public DriveToScoringPositionCommand(AutoAlign autoAlign, String label) {
            this.autoAlign = autoAlign;
            this.swerve = autoAlign.swerve;
            this.label = label;

            // Create the holonomic drive controller with PID constants
            // - First parameter: PID for X/Y position control
            // - Second parameter: PID for rotation (theta) control
            this.controller = new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.AUTO_XY_kP),    // X/Y PID
                new PIDConstants(SwerveConstants.AUTO_THETA_kP)  // Rotation PID
            );

            // Declare subsystem requirements (prevents conflicts)
            // This command needs EXCLUSIVE access to both subsystems
            addRequirements(autoAlign, swerve);

            // Give the command a descriptive name (shows in dashboard)
            setName("DriveToScoring(" + label + ")");
        }

        /**
         * Called once when the command starts.
         * Sets up the target position and resets the controller.
         */
        @Override
        public void initialize() {
            // Calculate where we need to be for this reef position
            Pose2d targetPose = getRobotScoringPose(label);

            // Store it in AutoAlign so other systems can see our goal
            autoAlign.setGoalPose(targetPose);

            // Create a trajectory state for the controller
            // (PathPlanner's controller expects this format)
            target = new PathPlannerTrajectoryState();
            target.pose = targetPose;

            // Reset the controller (clears any accumulated error)
            // Pass current position and zero velocity as starting point
            controller.reset(swerve.getPose(), new ChassisSpeeds());
        }

        /**
         * Called repeatedly while the command is running (~50Hz).
         * Calculates drive speeds and sends them to swerve.
         */
        @Override
        public void execute() {
            // Ask controller: "Given where I am and where I want to be,
            // how fast should I drive?"
            ChassisSpeeds output = controller.calculateRobotRelativeSpeeds(
                swerve.getPose(),  // Where we are now
                target             // Where we want to be
            );

            // Send the calculated speeds to the swerve drive
            swerve.drive(output, DrivingConstants.OPEN_LOOP);
        }

        /**
         * Called once when the command ends (either interrupted or finished).
         * Stops the robot from moving.
         *
         * @param interrupted True if command was cancelled, false if ended naturally
         */
        @Override
        public void end(boolean interrupted) {
            // Stop all motors (zero velocity)
            swerve.drive(new ChassisSpeeds(), DrivingConstants.OPEN_LOOP);
        }

        /**
         * Allow this command to run even when the robot is disabled.
         * Useful for testing/visualization without enabling the robot.
         */
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        /**
         * Return the set of subsystems this command uses.
         * The scheduler uses this to prevent command conflicts.
         */
        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(autoAlign, swerve);
        }
    }

    /**
     * Command to drive to an approach position near the reef.
     *
     * [DIFFERENCE FROM SCORING COMMAND]
     * This drives to the APPROACH position (further from reef) instead of
     * the SCORING position (close to reef). Used as a first step in
     * two-stage scoring sequences.
     *
     * [TYPICAL USAGE]
     * Run this first, then run DriveToScoringPositionCommand:
     *
     *   Commands.sequence(
     *       new DriveToApproachPositionCommand(autoAlign, "A"),
     *       Commands.waitSeconds(0.5),  // Brief pause to stabilize
     *       new DriveToScoringPositionCommand(autoAlign, "A")
     *   )
     */
    public static class DriveToApproachPositionCommand extends Command {

        // Same structure as DriveToScoringPositionCommand
        private final AutoAlign autoAlign;
        private final SwerveDrive swerve;
        private final String label;
        private final PPHolonomicDriveController controller;
        private PathPlannerTrajectoryState target;

        /**
         * Create a new DriveToApproachPositionCommand.
         *
         * @param autoAlign The AutoAlign subsystem
         * @param label The reef position to approach ("A" through "L")
         */
        public DriveToApproachPositionCommand(AutoAlign autoAlign, String label) {
            this.autoAlign = autoAlign;
            this.swerve = autoAlign.swerve;
            this.label = label;

            // Same controller setup as scoring command
            this.controller = new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.AUTO_XY_kP),
                new PIDConstants(SwerveConstants.AUTO_THETA_kP)
            );

            addRequirements(autoAlign, swerve);
            setName("DriveToApproach(" + label + ")");
        }

        @Override
        public void initialize() {
            // KEY DIFFERENCE: Use getRobotApproachPose instead of getRobotScoringPose
            Pose2d targetPose = getRobotApproachPose(label);
            autoAlign.setGoalPose(targetPose);

            target = new PathPlannerTrajectoryState();
            target.pose = targetPose;
            controller.reset(swerve.getPose(), new ChassisSpeeds());
        }

        @Override
        public void execute() {
            // Same execution logic - drive towards target
            ChassisSpeeds output = controller.calculateRobotRelativeSpeeds(swerve.getPose(), target);
            swerve.drive(output, DrivingConstants.OPEN_LOOP);
        }

        @Override
        public void end(boolean interrupted) {
            // Stop the robot
            swerve.drive(new ChassisSpeeds(), DrivingConstants.OPEN_LOOP);
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(autoAlign, swerve);
        }
    }

}  // End of AutoAlign class
