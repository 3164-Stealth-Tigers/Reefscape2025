package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

/**
 * A single swerve module with NEO drive motor, NEO azimuth motor, and CANCoder.
 * Uses WPILib's native swerve classes for kinematics.
 */
public class SwerveModule {
    private final int moduleNumber;

    // Motors
    private final SparkMax driveMotor;
    private final SparkMax azimuthMotor;

    // Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder azimuthEncoder;
    private final CANcoder canCoder;

    // Controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController azimuthController;

    // Feedforward
    private final SimpleMotorFeedforward driveFeedforward;

    // Offset for absolute encoder zeroing
    private final Rotation2d encoderOffset;

    // Last commanded state for optimization
    private SwerveModuleState lastState = new SwerveModuleState();

    /**
     * Creates a new SwerveModule.
     *
     * @param moduleNumber The module number (0-3 for FL, FR, RL, RR)
     * @param driveMotorId CAN ID of the drive motor
     * @param azimuthMotorId CAN ID of the azimuth motor
     * @param canCoderId CAN ID of the CANCoder
     * @param encoderOffset Offset for the absolute encoder in degrees
     */
    public SwerveModule(int moduleNumber, int driveMotorId, int azimuthMotorId,
                        int canCoderId, double encoderOffset) {
        this.moduleNumber = moduleNumber;
        this.encoderOffset = Rotation2d.fromDegrees(encoderOffset);

        // Initialize drive motor
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();

        // Initialize azimuth motor
        azimuthMotor = new SparkMax(azimuthMotorId, MotorType.kBrushless);
        azimuthEncoder = azimuthMotor.getEncoder();
        azimuthController = azimuthMotor.getClosedLoopController();

        // Initialize CANCoder
        canCoder = new CANcoder(canCoderId);

        // Configure drive feedforward
        driveFeedforward = new SimpleMotorFeedforward(
            SwerveConstants.DRIVE_kS,
            SwerveConstants.DRIVE_kV,
            SwerveConstants.DRIVE_kA
        );

        configureMotors();
        configureCANCoder();
        resetToAbsolute();
    }

    /**
     * Configure motor controllers with appropriate settings.
     */
    private void configureMotors() {
        // Drive motor configuration
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT)
            .openLoopRampRate(SwerveConstants.DRIVE_OPEN_LOOP_RAMP)
            .closedLoopRampRate(SwerveConstants.DRIVE_CLOSED_LOOP_RAMP);

        // Convert rotations to meters
        double drivePositionFactor = SwerveConstants.WHEEL_CIRCUMFERENCE / SwerveConstants.DRIVE_GEAR_RATIO;
        double driveVelocityFactor = drivePositionFactor / 60.0;

        driveConfig.encoder
            .positionConversionFactor(drivePositionFactor)
            .velocityConversionFactor(driveVelocityFactor);

        driveConfig.closedLoop
            .p(SwerveConstants.DRIVE_kP)
            .i(SwerveConstants.DRIVE_kI)
            .d(SwerveConstants.DRIVE_kD);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Azimuth motor configuration
        SparkMaxConfig azimuthConfig = new SparkMaxConfig();
        azimuthConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveConstants.AZIMUTH_CURRENT_LIMIT)
            .inverted(true);

        // Convert rotations to degrees
        double azimuthPositionFactor = 360.0 / SwerveConstants.AZIMUTH_GEAR_RATIO;
        double azimuthVelocityFactor = azimuthPositionFactor / 60.0;

        azimuthConfig.encoder
            .positionConversionFactor(azimuthPositionFactor)
            .velocityConversionFactor(azimuthVelocityFactor);

        azimuthConfig.closedLoop
            .p(SwerveConstants.AZIMUTH_kP)
            .i(SwerveConstants.AZIMUTH_kI)
            .d(SwerveConstants.AZIMUTH_kD)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(0)
            .positionWrappingMaxInput(360);

        azimuthMotor.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configure the CANCoder for absolute position sensing.
     */
    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(config);
    }

    /**
     * Reset the azimuth encoder to the absolute position from CANCoder.
     */
    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteAngle().getDegrees();
        azimuthEncoder.setPosition(absolutePosition);
    }

    /**
     * Get the absolute angle from the CANCoder, accounting for offset.
     *
     * @return The absolute angle as a Rotation2d
     */
    public Rotation2d getAbsoluteAngle() {
        double angle = canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        return Rotation2d.fromDegrees(angle).minus(encoderOffset);
    }

    /**
     * Get the current angle of the module from the integrated encoder.
     *
     * @return The current angle as a Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(azimuthEncoder.getPosition());
    }

    /**
     * Get the current velocity of the drive motor.
     *
     * @return The velocity in meters per second
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Get the current position of the drive motor.
     *
     * @return The position in meters
     */
    public double getPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Get the current state of the swerve module.
     *
     * @return The current SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Get the current position of the swerve module.
     *
     * @return The current SwerveModulePosition
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    /**
     * Set the desired state of the swerve module.
     *
     * @param desiredState The desired state (speed and angle)
     * @param openLoop Whether to use open-loop control for drive motor
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
        // Optimize the state to avoid rotating more than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        // Set azimuth angle
        azimuthController.setReference(
            desiredState.angle.getDegrees(),
            SparkMax.ControlType.kPosition
        );

        // Set drive velocity
        if (openLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double feedforward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity,
                0,
                feedforward
            );
        }

        lastState = desiredState;
    }

    /**
     * Stop the module by setting both motors to 0.
     */
    public void stop() {
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    /**
     * Get the module number.
     *
     * @return The module number (0-3)
     */
    public int getModuleNumber() {
        return moduleNumber;
    }
}
