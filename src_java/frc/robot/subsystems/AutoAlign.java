package frc.robot.subsystems;

import java.util.Set;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

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
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotPhysicalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.CoralStation;

/**
 * Auto-alignment subsystem for automated scoring and intake positioning.
 * Provides collision detection with the reef structure.
 */
public class AutoAlign extends SubsystemBase {
    private final SwerveDrive swerve;
    private Pose2d goalPose = new Pose2d();
    private boolean useClose = true;

    // Constants for driving behavior (should be in Constants.java)
    private static final double CLOSE_RADIUS = 0.5;  // meters
    private static final boolean USE_READY_FOR_CLOSE = true;
    private static final double MAXIMUM_POSITION_ERROR = 0.05;  // meters
    private static final double MAXIMUM_ANGULAR_POSITION_ERROR = 3.0;  // degrees
    private static final double REEF_WALL_TO_BUMPER_DISTANCE_FINAL = 0.02;
    private static final double REEF_WALL_TO_BUMPER_DISTANCE_APPROACH = 0.15;
    private static final double CORAL_STATION_WALL_TO_BUMPER_DISTANCE = 0.02;

    /**
     * Creates a new AutoAlign subsystem.
     *
     * @param swerve The swerve drive subsystem
     */
    public AutoAlign(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("AutoAlign/WillCollide", willCollideWithReef());
        SmartDashboard.putBoolean("AutoAlign/AtGoal", isAtGoalPose());
        SmartDashboard.putBoolean("AutoAlign/UseClose", useClose);
        SmartDashboard.putBoolean("AutoAlign/ReadyForClose", isReadyForClose());
    }

    /**
     * Check if the robot is close enough to its scoring position for close actions.
     *
     * @return True if ready for close actions
     */
    public boolean isReadyForClose() {
        boolean ready = goalPose.getTranslation().getDistance(swerve.getPose().getTranslation()) < CLOSE_RADIUS;
        return ready || !USE_READY_FOR_CLOSE || !useClose;
    }

    /**
     * Check if the current path to the goal will collide with the reef.
     *
     * @return True if a collision would occur
     */
    public boolean willCollideWithReef() {
        Translation2d reefCenter = FieldUtil.flipAlliance(
            new Translation2d(FieldConstants.REEF_CENTER_X, FieldConstants.REEF_CENTER_Y)
        );
        return willCollide(
            swerve.getPose().getTranslation(),
            goalPose.getTranslation(),
            reefCenter,
            RobotPhysicalConstants.ROBOT_WIDTH_WITH_BUMPERS / 2,
            FieldConstants.REEF_RADIUS
        );
    }

    /**
     * Check if the robot is at the goal pose.
     *
     * @return True if at the goal pose within tolerance
     */
    public boolean isAtGoalPose() {
        Pose2d currentPose = swerve.getPose();
        double positionError = goalPose.getTranslation().getDistance(currentPose.getTranslation());
        double angularError = Math.abs(goalPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        return positionError < MAXIMUM_POSITION_ERROR && angularError < MAXIMUM_ANGULAR_POSITION_ERROR;
    }

    /**
     * Get the goal pose.
     *
     * @return The current goal pose
     */
    public Pose2d getGoalPose() {
        return goalPose;
    }

    /**
     * Set the goal pose.
     *
     * @param pose The new goal pose
     */
    public void setGoalPose(Pose2d pose) {
        this.goalPose = pose;
    }

    /**
     * Calculate the robot's scoring pose for a given reef position.
     *
     * @param position The reef position label (e.g., "A", "B", etc.)
     * @return The target pose for scoring
     */
    public static Pose2d getRobotScoringPose(String position) {
        Translation2d pipeTranslation = FieldUtil.getReefPipeTranslation(position);

        // Get rotation for this position (would need to be defined)
        double rotationDegrees = FieldUtil.getReefRotation(position);
        Rotation2d rotation = FieldUtil.flipAlliance(Rotation2d.fromDegrees(rotationDegrees));

        Pose2d robotPose = new Pose2d(pipeTranslation, rotation);

        // Apply transforms to position robot correctly
        robotPose = robotPose.transformBy(new Transform2d(
            -RobotPhysicalConstants.ROBOT_LENGTH_WITH_BUMPERS / 2 - REEF_WALL_TO_BUMPER_DISTANCE_FINAL,
            0,
            new Rotation2d()
        ));

        return robotPose;
    }

    /**
     * Calculate the robot's approach pose for a given reef position.
     *
     * @param position The reef position label
     * @return The approach pose
     */
    public static Pose2d getRobotApproachPose(String position) {
        Pose2d scoringPose = getRobotScoringPose(position);
        return scoringPose.transformBy(new Transform2d(
            -(REEF_WALL_TO_BUMPER_DISTANCE_APPROACH - REEF_WALL_TO_BUMPER_DISTANCE_FINAL),
            0,
            new Rotation2d()
        ));
    }

    /**
     * Calculate the robot's intake pose for a coral station.
     *
     * @param station The coral station
     * @return The intake pose
     */
    public static Pose2d getRobotIntakePose(CoralStation station) {
        Pose2d stationPose = FieldUtil.flipAlliance(station.getPose());
        return stationPose.transformBy(new Transform2d(
            -RobotPhysicalConstants.ROBOT_LENGTH_WITH_BUMPERS / 2 - CORAL_STATION_WALL_TO_BUMPER_DISTANCE,
            0,
            new Rotation2d()
        ));
    }

    /**
     * Check if a moving circle will collide with a stationary circle.
     *
     * @param aInitial Initial position of moving circle
     * @param aFinal Final position of moving circle
     * @param b Position of stationary circle
     * @param radiusA Radius of moving circle
     * @param radiusB Radius of stationary circle
     * @return True if collision will occur
     */
    public static boolean willCollide(Translation2d aInitial, Translation2d aFinal,
                                      Translation2d b, double radiusA, double radiusB) {
        // Setup quadratic coefficients
        double dx = aFinal.getX() - aInitial.getX();
        double dy = aFinal.getY() - aInitial.getY();

        double A = dx * dx + dy * dy;
        double B = 2 * ((aInitial.getX() - b.getX()) * dx + (aInitial.getY() - b.getY()) * dy);
        double C = Math.pow(aInitial.getX() - b.getX(), 2) + Math.pow(aInitial.getY() - b.getY(), 2)
                 - Math.pow(radiusA + radiusB, 2);

        double discriminant = B * B - 4 * A * C;

        if (A == 0) {
            return false;
        }

        // Check if there is at least one solution
        if (discriminant >= 0) {
            double t1 = (-B + Math.sqrt(discriminant)) / (2 * A);
            double t2 = (-B - Math.sqrt(discriminant)) / (2 * A);

            // Check if roots are within [0, 1]
            if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1) {
                return true;
            }
        }

        return false;
    }

    /**
     * Wrap a command to wait until close to the reef before executing.
     *
     * @param cmd The command to wrap
     * @return The wrapped command
     */
    public Command closeCommand(Command cmd) {
        return Commands.waitUntil(this::isReadyForClose)
            .andThen(cmd)
            .withName("CloseWait(" + cmd.getName() + ")");
    }

    // ==================== Command Classes ====================

    /**
     * Command to drive to a scoring position on the reef.
     */
    public static class DriveToScoringPositionCommand extends Command {
        private final AutoAlign autoAlign;
        private final SwerveDrive swerve;
        private final String label;
        private final PPHolonomicDriveController controller;
        private PathPlannerTrajectoryState target;

        public DriveToScoringPositionCommand(AutoAlign autoAlign, String label) {
            this.autoAlign = autoAlign;
            this.swerve = autoAlign.swerve;
            this.label = label;
            this.controller = new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.AUTO_XY_kP),
                new PIDConstants(SwerveConstants.AUTO_THETA_kP)
            );
            addRequirements(autoAlign, swerve);
            setName("DriveToScoring(" + label + ")");
        }

        @Override
        public void initialize() {
            Pose2d targetPose = getRobotScoringPose(label);
            autoAlign.setGoalPose(targetPose);
            target = new PathPlannerTrajectoryState();
            target.pose = targetPose;
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
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(autoAlign, swerve);
        }
    }

    /**
     * Command to drive to an approach position near the reef.
     */
    public static class DriveToApproachPositionCommand extends Command {
        private final AutoAlign autoAlign;
        private final SwerveDrive swerve;
        private final String label;
        private final PPHolonomicDriveController controller;
        private PathPlannerTrajectoryState target;

        public DriveToApproachPositionCommand(AutoAlign autoAlign, String label) {
            this.autoAlign = autoAlign;
            this.swerve = autoAlign.swerve;
            this.label = label;
            this.controller = new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.AUTO_XY_kP),
                new PIDConstants(SwerveConstants.AUTO_THETA_kP)
            );
            addRequirements(autoAlign, swerve);
            setName("DriveToApproach(" + label + ")");
        }

        @Override
        public void initialize() {
            Pose2d targetPose = getRobotApproachPose(label);
            autoAlign.setGoalPose(targetPose);
            target = new PathPlannerTrajectoryState();
            target.pose = targetPose;
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
            return Set.of(autoAlign, swerve);
        }
    }
}
