package frc.robot;

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
 * Superstructure class that coordinates multiple subsystems for complex actions.
 * This is not a subsystem itself, but rather a coordinator for the elevator, arm, and climber.
 */
public class Superstructure {
    private final SwerveDrive swerve;
    private final Elevator elevator;
    private final CoralArm coralArm;
    private final Climber climber;
    private final AutoAlign autoAlign;

    // Velocity thresholds
    private static final double MAXIMUM_VELOCITY_ERROR = 0.1;  // m/s
    private static final double MAXIMUM_ANGULAR_VELOCITY_ERROR = 5.0;  // deg/s

    /**
     * Creates a new Superstructure coordinator.
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

    /**
     * Check if the robot is ready to release a CORAL and score.
     *
     * @return True if ready to score
     */
    public boolean readyToScore() {
        // Get current robot speeds
        ChassisSpeeds speeds = swerve.getKinematics().toChassisSpeeds(swerve.getModuleStates());

        boolean readyToScore = (
            // Robot is at the scoring position
            autoAlign.isAtGoalPose() &&
            // Robot is not moving
            Math.abs(speeds.vxMetersPerSecond) < MAXIMUM_VELOCITY_ERROR &&
            Math.abs(speeds.vyMetersPerSecond) < MAXIMUM_VELOCITY_ERROR &&
            Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond)) < MAXIMUM_ANGULAR_VELOCITY_ERROR &&
            // Elevator has reached scoring height
            elevator.atGoalHeight() &&
            // Arm has reached scoring rotation
            coralArm.atGoalRotation()
        );

        SmartDashboard.putBoolean("Superstructure/ReadyToScore", readyToScore);
        return readyToScore;
    }

    /**
     * Create a command that moves the robot's claw (end effector) to a specified height.
     * The elevator and arm move simultaneously to achieve the desired height.
     *
     * @param endEffectorHeight The height (in meters) to move the end effector to
     * @return A command that moves to the specified height
     */
    public Command setEndEffectorHeightCommand(double endEffectorHeight) {
        return setEndEffectorHeightCommand(endEffectorHeight, null);
    }

    /**
     * Create a command that moves the robot's claw (end effector) to a specified height and angle.
     *
     * @param endEffectorHeight The height (in meters) to move the end effector to
     * @param angle The arm angle, or null to calculate automatically
     * @return A command that moves to the specified height
     */
    public Command setEndEffectorHeightCommand(double endEffectorHeight, Rotation2d angle) {
        // Calculate angle if not provided
        if (angle == null) {
            if (endEffectorHeight <= ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT) {
                angle = new Rotation2d();  // 0 degrees (horizontal)
            } else {
                double heightDiff = endEffectorHeight - ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT;
                angle = new Rotation2d(Math.asin(heightDiff / CoralArmConstants.ARM_LENGTH));
            }
        }

        // Calculate required carriage height
        double carriageHeight = endEffectorHeight - (CoralArmConstants.ARM_LENGTH * angle.getSin());

        // Validate carriage height
        if (carriageHeight > ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT ||
            carriageHeight < ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT) {
            throw new IllegalArgumentException(
                "Calculated carriage height is out of bounds: " + carriageHeight + " meters"
            );
        }

        final Rotation2d finalAngle = angle;
        final double finalHeight = carriageHeight;

        return Commands.parallel(
            coralArm.setAngleCommand(finalAngle),
            elevator.setHeightCommand(finalHeight)
        ).beforeStarting(
            Commands.print("Setting height: " + finalHeight + "m, angle: " + finalAngle.getDegrees() + "°")
        ).withName("SetEndEffectorHeight");
    }

    // ==================== Accessors ====================

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
}
