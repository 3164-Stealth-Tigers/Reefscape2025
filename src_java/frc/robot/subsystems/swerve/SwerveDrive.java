package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

/**
 * Swerve drive subsystem using WPILib's native swerve classes.
 * Replaces swervepy with pure WPILib implementation.
 */
public class SwerveDrive extends SubsystemBase {
    // Swerve modules (FL, FR, RL, RR)
    private final SwerveModule[] modules;

    // Gyro
    private final Pigeon2 gyro;

    // Kinematics
    private final SwerveDriveKinematics kinematics;

    // Pose estimation
    private final SwerveDrivePoseEstimator poseEstimator;

    // Field visualization
    private final Field2d field;

    // NetworkTables publishers for AdvantageScope
    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<SwerveModuleState> statePublisher;

    // Drive mode settings
    private boolean fieldRelative = true;
    private boolean openLoop = SwerveConstants.DRIVE_OPEN_LOOP_RAMP > 0;

    /**
     * Creates a new SwerveDrive subsystem.
     */
    public SwerveDrive() {
        // Initialize gyro
        gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
        gyro.reset();

        // Define module positions relative to robot center
        Translation2d[] moduleLocations = {
            new Translation2d(SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),   // FL
            new Translation2d(SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2),  // FR
            new Translation2d(-SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),  // RL
            new Translation2d(-SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2)  // RR
        };

        // Initialize kinematics
        kinematics = new SwerveDriveKinematics(moduleLocations);

        // Initialize swerve modules
        modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_AZIMUTH_ID,
                           SwerveConstants.FL_CANCODER_ID, SwerveConstants.FL_ENCODER_OFFSET),
            new SwerveModule(1, SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_AZIMUTH_ID,
                           SwerveConstants.FR_CANCODER_ID, SwerveConstants.FR_ENCODER_OFFSET),
            new SwerveModule(2, SwerveConstants.RL_DRIVE_ID, SwerveConstants.RL_AZIMUTH_ID,
                           SwerveConstants.RL_CANCODER_ID, SwerveConstants.RL_ENCODER_OFFSET),
            new SwerveModule(3, SwerveConstants.RR_DRIVE_ID, SwerveConstants.RR_AZIMUTH_ID,
                           SwerveConstants.RR_CANCODER_ID, SwerveConstants.RR_ENCODER_OFFSET)
        };

        // Initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d()
        );

        // Initialize field visualization
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Initialize NT publishers
        var table = NetworkTableInstance.getDefault().getTable("Swerve");
        posePublisher = table.getStructTopic("Pose", Pose2d.struct).publish();
        statePublisher = table.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    }

    @Override
    public void periodic() {
        // Update pose estimator
        poseEstimator.update(getYaw(), getModulePositions());

        // Update field visualization
        field.setRobotPose(getPose());

        // Publish to NetworkTables
        posePublisher.set(getPose());
        statePublisher.set(getModuleStates());

        // Log module states
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Swerve/Module" + i + "/Angle", modules[i].getAngle().getDegrees());
            SmartDashboard.putNumber("Swerve/Module" + i + "/Velocity", modules[i].getVelocity());
        }
    }

    /**
     * Drive the robot with the given velocities.
     *
     * @param translation Translation velocity (x forward, y left) in m/s
     * @param rotation Rotational velocity in rad/s
     * @param fieldRelative Whether to drive field-relative
     * @param openLoop Whether to use open-loop control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
            );
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        drive(speeds, openLoop);
    }

    /**
     * Drive the robot with the given chassis speeds.
     *
     * @param speeds The desired chassis speeds
     * @param openLoop Whether to use open-loop control
     */
    public void drive(ChassisSpeeds speeds, boolean openLoop) {
        // Discretize to reduce skew
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Convert to module states
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Desaturate wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);

        // Set module states
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i], openLoop);
        }
    }

    /**
     * Set the modules to an X pattern to resist pushing.
     */
    public void setX() {
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    }

    /**
     * Stop all modules.
     */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /**
     * Get the current yaw angle from the gyro.
     *
     * @return The current yaw as a Rotation2d
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * Reset the gyro to a specific angle.
     *
     * @param angle The angle to reset to
     */
    public void resetYaw(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    /**
     * Get the current estimated pose.
     *
     * @return The current pose
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the pose estimator to a specific pose.
     *
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Add a vision measurement to the pose estimator.
     *
     * @param pose The measured pose
     * @param timestampSeconds The timestamp of the measurement
     */
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    /**
     * Get the positions of all modules.
     *
     * @return Array of module positions
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
     * @return Array of module states
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
     * @return The SwerveDriveKinematics
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Toggle field-relative driving mode.
     */
    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    /**
     * Get whether field-relative mode is enabled.
     *
     * @return True if field-relative mode is enabled
     */
    public boolean isFieldRelative() {
        return fieldRelative;
    }

    /**
     * Create a teleop drive command.
     *
     * @param forward Supplier for forward velocity (-1 to 1)
     * @param strafe Supplier for strafe velocity (-1 to 1)
     * @param turn Supplier for turn velocity (-1 to 1)
     * @return A command that drives the robot
     */
    public Command teleopCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return new RunCommand(() -> {
            double vx = forward.getAsDouble() * SwerveConstants.MAX_SPEED;
            double vy = strafe.getAsDouble() * SwerveConstants.MAX_SPEED;
            double omega = turn.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;

            // Apply alliance flipping if on red alliance
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                vx = -vx;
                vy = -vy;
            }

            drive(new Translation2d(vx, vy), omega, fieldRelative, openLoop);
        }, this);
    }

    /**
     * Create a command to set modules to X pattern (ski stop).
     *
     * @return A command that locks the wheels
     */
    public Command skiStopCommand() {
        return new RunCommand(this::setX, this);
    }

    /**
     * Create a command to reset the gyro to 180 degrees.
     *
     * @return A command that resets the gyro
     */
    public Command resetGyroCommand() {
        return runOnce(() -> resetYaw(Rotation2d.fromDegrees(180)));
    }
}
