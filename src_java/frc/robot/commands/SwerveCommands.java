package frc.robot.commands;

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
 * Swerve drive commands including ski stop, drive to pose, and drive distance.
 */
public final class SwerveCommands {

    private SwerveCommands() {
        // Utility class - prevent instantiation
    }

    /**
     * Create a command that sets the wheels in an X pattern to resist pushing.
     *
     * @param swerve The swerve drive subsystem
     * @return The ski stop command
     */
    public static Command skiStopCommand(SwerveDrive swerve) {
        return Commands.run(swerve::setX, swerve).withName("SkiStop");
    }

    // ==================== Command Classes ====================

    /**
     * Command that drives the robot to a specific pose on the field.
     */
    public static class DriveToPoseCommand extends Command {
        private final SwerveDrive swerve;
        private final PathPlannerTrajectoryState target;
        private final PPHolonomicDriveController controller;

        /**
         * Create a drive to pose command.
         *
         * @param swerve The swerve drive subsystem
         * @param targetPose The target pose in field coordinates
         */
        public DriveToPoseCommand(SwerveDrive swerve, Pose2d targetPose) {
            this.swerve = swerve;
            this.target = new PathPlannerTrajectoryState();
            this.target.pose = targetPose;
            this.controller = new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.AUTO_XY_kP),
                new PIDConstants(SwerveConstants.AUTO_THETA_kP)
            );
            addRequirements(swerve);
            setName("DriveToPose");
        }

        @Override
        public void initialize() {
            controller.reset(swerve.getPose(), new ChassisSpeeds());
        }

        @Override
        public void execute() {
            ChassisSpeeds output = controller.calculateRobotRelativeSpeeds(swerve.getPose(), target);
            swerve.drive(output, DrivingConstants.OPEN_LOOP);
        }

        @Override
        public void end(boolean interrupted) {
            swerve.drive(new ChassisSpeeds(), DrivingConstants.OPEN_LOOP);
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(swerve);
        }
    }

    /**
     * Command that drives the robot a specific distance in a straight line.
     */
    public static class DriveDistanceCommand extends Command {
        private final SwerveDrive swerve;
        private final Translation2d speeds;
        private final double distance;
        private Translation2d initialTranslation;

        /**
         * Create a drive distance command.
         *
         * @param swerve The swerve drive subsystem
         * @param xSpeed Speed in x-direction (m/s), positive toward Red Driver Station
         * @param ySpeed Speed in y-direction (m/s), positive to the left
         * @param distance Distance to travel (m), must be positive
         */
        public DriveDistanceCommand(SwerveDrive swerve, double xSpeed, double ySpeed, double distance) {
            this.swerve = swerve;
            this.speeds = new Translation2d(xSpeed, ySpeed);
            this.distance = distance;
            addRequirements(swerve);
            setName("DriveDistance");
        }

        @Override
        public void initialize() {
            initialTranslation = swerve.getPose().getTranslation();
        }

        @Override
        public void execute() {
            swerve.drive(speeds, 0, false, false);
        }

        @Override
        public void end(boolean interrupted) {
            swerve.drive(new Translation2d(), 0, false, false);
        }

        @Override
        public boolean isFinished() {
            return swerve.getPose().getTranslation().getDistance(initialTranslation) >= distance;
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(swerve);
        }
    }
}
