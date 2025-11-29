package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Climber subsystem for end-game climbing.
 * Uses two SparkFlex motors in leader/follower configuration.
 */
public class Climber extends SubsystemBase {
    // Motors
    private final SparkFlex leaderMotor;
    private final SparkFlex followerMotor;

    // Encoder
    private final AbsoluteEncoder encoder;

    // Constants (need to be added to Constants.java)
    private static final double ZERO_OFFSET = 0.0;
    private static final double FORWARD_LIMIT_DEGREES = 360.0;
    private static final double BACKWARD_LIMIT_DEGREES = 0.0;
    private static final boolean LIMITS_ENABLED = true;
    private static final boolean INVERT_FOLLOWER = false;

    /**
     * Creates a new Climber subsystem.
     */
    public Climber() {
        // Initialize motors
        followerMotor = new SparkFlex(ClimberConstants.FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        leaderMotor = new SparkFlex(ClimberConstants.LEADER_MOTOR_ID, MotorType.kBrushless);

        encoder = leaderMotor.getAbsoluteEncoder();

        configureMotors();
    }

    /**
     * Configure both motors with appropriate settings.
     */
    private void configureMotors() {
        // Leader configuration
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        leaderConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(true);

        leaderConfig.absoluteEncoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360.0 / 60.0)
            .zeroOffset(ZERO_OFFSET)
            .inverted(false);

        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        leaderConfig.softLimit
            .forwardSoftLimit(FORWARD_LIMIT_DEGREES)
            .reverseSoftLimit(BACKWARD_LIMIT_DEGREES)
            .forwardSoftLimitEnabled(LIMITS_ENABLED)
            .reverseSoftLimitEnabled(LIMITS_ENABLED);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower configuration
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .follow(ClimberConstants.LEADER_MOTOR_ID, INVERT_FOLLOWER);

        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Angle", getAngle().getDegrees());
    }

    /**
     * Move the climber at a specified power.
     *
     * @param power The power to apply (-1 to 1)
     */
    public void moveClimber(double power) {
        leaderMotor.set(power);
    }

    /**
     * Stop the climber motors.
     */
    public void stop() {
        leaderMotor.set(0);
    }

    /**
     * Get the current angle of the climber.
     *
     * @return The current angle as a Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    // ==================== Commands ====================

    /**
     * Command to raise the robot (climb up).
     *
     * @return The climb command
     */
    public Command raiseRobotCommand() {
        return Commands.startEnd(
            () -> moveClimber(1),
            this::stop,
            this
        ).withName("Raise Robot");
    }

    /**
     * Command to lower the robot.
     *
     * @return The lower command
     */
    public Command lowerRobotCommand() {
        return Commands.startEnd(
            () -> moveClimber(-1),
            this::stop,
            this
        ).withName("Lower Robot");
    }
}
