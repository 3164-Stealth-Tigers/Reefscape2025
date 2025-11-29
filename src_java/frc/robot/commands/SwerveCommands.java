package frc.robot.commands;

/*
 * ========================================================================
 * SWERVE COMMANDS - Pre-Built Commands for Swerve Drive
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Contains reusable commands for controlling the swerve drive:
 *   1. Ski Stop - Lock wheels in X pattern to resist pushing
 *   2. Drive To Pose - Autonomously drive to a specific position
 *   3. Drive Distance - Drive a set distance in a straight line
 *
 * WHY SEPARATE FROM SWERVE DRIVE?
 * -------------------------------
 * We could put these commands inside SwerveDrive, but keeping them
 * separate makes the code more organized and easier to find.
 *
 * COMMAND TYPES:
 * --------------
 *
 *   FACTORY METHOD (skiStopCommand):
 *   ┌──────────────────────────────────────┐
 *   │  Command cmd = SwerveCommands.       │
 *   │      skiStopCommand(swerve);         │
 *   └──────────────────────────────────────┘
 *   Simple commands can be created with a static method.
 *
 *   COMMAND CLASS (DriveToPoseCommand):
 *   ┌──────────────────────────────────────┐
 *   │  Command cmd = new DriveToPoseCommand│
 *   │      (swerve, targetPose);           │
 *   └──────────────────────────────────────┘
 *   Complex commands that need state (variables) use a class.
 *
 * PATH PLANNER CONTROLLER:
 * ------------------------
 * DriveToPoseCommand uses PathPlanner's PPHolonomicDriveController.
 * This controller calculates the speeds needed to reach a target pose.
 * "Holonomic" means it can control X, Y, and rotation independently
 * (which is exactly what swerve drives can do!).
 *
 *   Current Pose ──→ [Controller] ──→ ChassisSpeeds ──→ Target Pose
 *
 * HOW TO MODIFY:
 * --------------
 * - Change ski stop pattern: Modify skiStopCommand() or SwerveDrive.setX()
 * - Tune drive-to-pose: Adjust SwerveConstants.AUTO_XY_kP, AUTO_THETA_kP
 * - Add new commands: Follow the pattern of existing commands
 *
 * QUICK REFERENCE:
 * ----------------
 * → Lock wheels: SwerveCommands.skiStopCommand(swerve)
 * → Drive to pose: new DriveToPoseCommand(swerve, targetPose)
 * → Drive distance: new DriveDistanceCommand(swerve, xSpeed, ySpeed, dist)
 *
 * ========================================================================
 */

import java.util.Set;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * ========================================================================
 * SWERVE COMMANDS - Utility Class for Swerve Drive Commands
 * ========================================================================
 *
 * This is a utility class - all methods are static, no instances needed.
 * Contains pre-built commands for common swerve drive operations.
 *
 * [UTILITY CLASS PATTERN]
 * - Private constructor (can't create instances)
 * - All methods are static
 * - No state (instance variables)
 */
public final class SwerveCommands {

    /**
     * Private constructor prevents instantiation.
     * Utility classes should never be instantiated.
     */
    private SwerveCommands() {
        // Utility class - prevent instantiation
    }

    // ========================================================================
    // SKI STOP COMMAND
    // ========================================================================
    //
    // The "ski stop" (or "X-lock") sets wheels in an X pattern:
    //
    //     ╲  ╱
    //      ╳
    //     ╱  ╲
    //
    // This makes it very hard for other robots to push us!
    // Like digging in your ski edges to stop on a slope.
    //
    // ========================================================================

    /**
     * Create a command that sets the wheels in an X pattern to resist pushing.
     *
     * [WHEN TO USE]
     * - Defense: When another robot is trying to push you
     * - End of autonomous: Lock position while waiting for teleop
     * - Emergency: If you need to stop immediately
     *
     * [HOW IT WORKS]
     * Each wheel points toward/away from center, creating an X pattern.
     * This locks the robot in place - it can't be pushed in any direction.
     *
     * @param swerve The swerve drive subsystem
     * @return A command that continuously sets the X pattern (runs until cancelled)
     */
    public static Command skiStopCommand(SwerveDrive swerve) {
        // Commands.run() creates a command that calls the lambda repeatedly
        // swerve::setX is a method reference - it's like () -> swerve.setX()
        return Commands.run(swerve::setX, swerve).withName("SkiStop");
    }

    // ========================================================================
    // DRIVE TO POSE COMMAND
    // ========================================================================
    //
    // Uses a PID controller to drive the robot to a specific position and
    // rotation on the field. This is useful for autonomous and auto-align.
    //
    // [CONTROL LOOP]
    //   ┌─────────────────────────────────────────────────────────────┐
    //   │  Target Pose ─┐                                            │
    //   │               ↓                                            │
    //   │  Current Pose → [Calculate Error] → [PID] → ChassisSpeeds  │
    //   │       ↑                                          │         │
    //   │       └──────────── Robot ←──────────────────────┘         │
    //   └─────────────────────────────────────────────────────────────┘
    //
    // ========================================================================

    /**
     * Command that drives the robot to a specific pose on the field.
     *
     * [WHAT IS A POSE?]
     * A Pose2d contains X position, Y position, and rotation (heading).
     * It fully describes where the robot is on the field and which way it's facing.
     *
     * [PATH PLANNER CONTROLLER]
     * We use PathPlanner's holonomic controller instead of writing our own.
     * It handles the math of driving to a pose with smooth motion.
     */
    public static class DriveToPoseCommand extends Command {

        // ================================================================
        // INSTANCE VARIABLES
        // ================================================================

        /** The swerve drive to control */
        private final SwerveDrive swerve;

        /**
         * Target state (wraps the target pose).
         * PathPlanner uses trajectory states, so we wrap our pose in one.
         */
        private final PathPlannerTrajectoryState target;

        /**
         * The controller that calculates how to reach the target.
         * "Holonomic" = can control X, Y, and rotation independently.
         */
        private final PPHolonomicDriveController controller;

        // ================================================================
        // CONSTRUCTOR
        // ================================================================

        /**
         * Create a drive to pose command.
         *
         * [NOTE]
         * This command runs until manually cancelled - it doesn't have
         * an isFinished() condition. Use .until() to add a finish condition:
         *
         *   new DriveToPoseCommand(swerve, pose)
         *       .until(() -> atTarget())  // Finish when at target
         *
         * @param swerve The swerve drive subsystem
         * @param targetPose The target pose in field coordinates
         */
        public DriveToPoseCommand(SwerveDrive swerve, Pose2d targetPose) {
            this.swerve = swerve;

            // Wrap the pose in a trajectory state (PathPlanner requirement)
            this.target = new PathPlannerTrajectoryState();
            this.target.pose = targetPose;

            // Create the holonomic controller with separate PID for:
            //   - XY position (how fast to drive toward target)
            //   - Theta/rotation (how fast to turn toward target heading)
            this.controller = new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.AUTO_XY_kP),      // Position PID
                new PIDConstants(SwerveConstants.AUTO_THETA_kP)    // Rotation PID
            );

            // Tell the command scheduler this command uses the swerve drive
            addRequirements(swerve);
            setName("DriveToPose");
        }

        // ================================================================
        // COMMAND LIFECYCLE
        // ================================================================

        /**
         * Called once when the command starts.
         * Resets the controller to avoid using stale data.
         */
        @Override
        public void initialize() {
            // Reset controller with current pose and zero velocity
            // This clears any accumulated integral error
            controller.reset(swerve.getPose(), new ChassisSpeeds());
        }

        /**
         * Called repeatedly while command runs (every 20ms).
         * Calculates and applies the speeds needed to reach target.
         */
        @Override
        public void execute() {
            // Calculate the speeds needed to move toward target
            // Robot-relative means speeds are relative to robot's current heading
            ChassisSpeeds output = controller.calculateRobotRelativeSpeeds(swerve.getPose(), target);

            // Apply the calculated speeds to the swerve drive
            swerve.drive(output, DrivingConstants.OPEN_LOOP);
        }

        /**
         * Called once when command ends (either finished or interrupted).
         * Stops the robot.
         *
         * @param interrupted True if command was interrupted, false if finished normally
         */
        @Override
        public void end(boolean interrupted) {
            // Stop all movement
            swerve.drive(new ChassisSpeeds(), DrivingConstants.OPEN_LOOP);
        }

        /**
         * Declares which subsystems this command requires.
         * Required to prevent multiple commands from using swerve simultaneously.
         */
        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(swerve);
        }
    }

    // ========================================================================
    // DRIVE DISTANCE COMMAND
    // ========================================================================
    //
    // Drives the robot a specific distance in a straight line.
    // Useful for simple autonomous movements.
    //
    // [HOW IT WORKS]
    //   1. Record starting position
    //   2. Drive at specified speed
    //   3. Measure distance from start
    //   4. Stop when distance reached
    //
    //   Start ─────────────────→ End
    //   [Record]    [Drive]    [Stop when distance reached]
    //
    // ========================================================================

    /**
     * Command that drives the robot a specific distance in a straight line.
     *
     * [DIFFERENCE FROM DRIVE TO POSE]
     * - DriveToPose: Goes to a specific X,Y position on the field
     * - DriveDistance: Goes a certain distance from where you are now
     *
     * DriveDistance is simpler but less precise. Use it for quick movements
     * where exact positioning doesn't matter.
     */
    public static class DriveDistanceCommand extends Command {

        // ================================================================
        // INSTANCE VARIABLES
        // ================================================================

        /** The swerve drive to control */
        private final SwerveDrive swerve;

        /**
         * The speed to drive at (X and Y components).
         * The direction of motion is determined by the ratio of X to Y speed.
         */
        private final Translation2d speeds;

        /** The distance to travel before stopping (meters) */
        private final double distance;

        /**
         * Where we started - used to measure how far we've gone.
         * Set in initialize(), used in isFinished().
         */
        private Translation2d initialTranslation;

        // ================================================================
        // CONSTRUCTOR
        // ================================================================

        /**
         * Create a drive distance command.
         *
         * [COORDINATE SYSTEM]
         * The FRC field coordinate system:
         *   - X positive: Toward the Red Driver Station
         *   - Y positive: To the left (when looking from Blue toward Red)
         *
         *        +Y (left)
         *           ↑
         *           │
         *   Blue ───┼───→ +X Red
         *           │
         *
         * @param swerve The swerve drive subsystem
         * @param xSpeed Speed in x-direction (m/s), positive toward Red Driver Station
         * @param ySpeed Speed in y-direction (m/s), positive to the left
         * @param distance Distance to travel (m), must be positive
         */
        public DriveDistanceCommand(SwerveDrive swerve, double xSpeed, double ySpeed, double distance) {
            this.swerve = swerve;
            this.speeds = new Translation2d(xSpeed, ySpeed);  // Combine into one object
            this.distance = distance;
            addRequirements(swerve);
            setName("DriveDistance");
        }

        // ================================================================
        // COMMAND LIFECYCLE
        // ================================================================

        /**
         * Called once when command starts.
         * Records the starting position so we can measure distance traveled.
         */
        @Override
        public void initialize() {
            // Save where we started
            initialTranslation = swerve.getPose().getTranslation();
        }

        /**
         * Called repeatedly while command runs.
         * Drives at constant speed in the specified direction.
         */
        @Override
        public void execute() {
            // Drive at the specified speed
            // Parameters: translation speed, rotation speed, field-relative, open-loop
            swerve.drive(speeds, 0, false, false);
        }

        /**
         * Called when command ends.
         * Stops all robot movement.
         */
        @Override
        public void end(boolean interrupted) {
            // Stop the robot
            swerve.drive(new Translation2d(), 0, false, false);
        }

        /**
         * Determines when the command should finish.
         *
         * [THE MATH]
         * We calculate the straight-line distance from start to current position.
         * When this distance exceeds our target, we're done.
         *
         * distance = √[(x₂-x₁)² + (y₂-y₁)²]  (Pythagorean theorem)
         *
         * @return True when we've traveled the specified distance
         */
        @Override
        public boolean isFinished() {
            // Calculate distance from starting position
            double distanceTraveled = swerve.getPose().getTranslation().getDistance(initialTranslation);
            return distanceTraveled >= distance;
        }

        /**
         * Declares which subsystems this command requires.
         */
        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(swerve);
        }
    }

}  // End of SwerveCommands class
