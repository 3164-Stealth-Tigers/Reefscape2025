package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralArmConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Coral arm subsystem for rotating the scoring mechanism.
 * Uses a NEO Vortex motor with an absolute encoder.
 */
public class CoralArm extends SubsystemBase {
    // Motor and encoder
    private final SparkFlex motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController controller;

    // Feedforward
    private final ArmFeedforward feedforward;

    // Encoder offset for the real robot
    private final double encoderOffset;

    // Mechanism visualization
    private final Mechanism2d mechanism;
    private final MechanismLigament2d armLigament;

    // SysId routine
    private final SysIdRoutine sysIdRoutine;

    // Goal tracking
    private Rotation2d goalRotation = null;

    /**
     * Creates a new CoralArm subsystem.
     */
    public CoralArm() {
        // Initialize motor
        motor = new SparkFlex(CoralArmConstants.MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Initialize feedforward
        feedforward = new ArmFeedforward(
            CoralArmConstants.FEEDFORWARD_CONSTANTS[0],  // kS
            CoralArmConstants.FEEDFORWARD_CONSTANTS[1],  // kG
            CoralArmConstants.FEEDFORWARD_CONSTANTS[2],  // kV
            CoralArmConstants.FEEDFORWARD_CONSTANTS[3]   // kA
        );

        // Use encoder offset on real robot, but don't offset in simulation
        encoderOffset = RobotBase.isReal() ? CoralArmConstants.ENCODER_OFFSET : 0;

        // Configure motor
        configureMotor();

        // Reset NEO encoder to absolute encoder position
        motor.getEncoder().setPosition(absoluteEncoder.getPosition());

        // Setup mechanism visualization
        mechanism = new Mechanism2d(5, 5);
        MechanismRoot2d root = mechanism.getRoot("armPivot", 1, 2.5);
        armLigament = root.append(new MechanismLigament2d("arm", 3, getAngle().getDegrees()));
        SmartDashboard.putData("Arm Mechanism", mechanism);

        // Setup SysId routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds.of(1)),
                Volts.of(3),
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                voltage -> setVoltage(voltage.in(Volts)),
                log -> {
                    log.motor("arm-pivot")
                        .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
                        .angularPosition(Degrees.of(getAngle().getDegrees()))
                        .angularVelocity(DegreesPerSecond.of(absoluteEncoder.getVelocity()));
                },
                this
            )
        );
    }

    /**
     * Configure the motor with appropriate settings.
     */
    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);

        // Configure absolute encoder
        config.absoluteEncoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360.0 / 60.0);

        // Configure relative encoder
        config.encoder
            .positionConversionFactor(360.0 / CoralArmConstants.GEAR_RATIO)
            .velocityConversionFactor(360.0 / CoralArmConstants.GEAR_RATIO / 60.0);

        // Configure closed-loop control
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(CoralArmConstants.kP)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        // Configure soft limits
        config.softLimit
            .forwardSoftLimit(CoralArmConstants.MAXIMUM_ANGLE.getDegrees() + encoderOffset)
            .reverseSoftLimit(CoralArmConstants.MINIMUM_ANGLE.getDegrees() + encoderOffset)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Update mechanism visualization
        armLigament.setAngle(getAngle().getDegrees());

        // Log data
        SmartDashboard.putNumber("CoralArm/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("CoralArm/Velocity", absoluteEncoder.getVelocity());
        SmartDashboard.putBoolean("CoralArm/AtGoal", atGoalRotation());
        SmartDashboard.putString("CoralArm/Command", getCurrentCommandName());

        if (goalRotation != null) {
            SmartDashboard.putNumber("CoralArm/GoalAngle", goalRotation.getDegrees());
        }
    }

    /**
     * Set the arm to a specific angle.
     * 0 degrees is horizontal, facing toward the front of the robot.
     *
     * @param angle Counter-clockwise positive angle
     */
    public void setAngle(Rotation2d angle) {
        goalRotation = angle;
        controller.setReference(
            angle.getDegrees() + encoderOffset,
            ControlType.kPosition
        );
    }

    /**
     * Set the motor duty cycle directly.
     *
     * @param output The duty cycle (-1 to 1)
     */
    public void setDutyCycle(double output) {
        motor.set(output);
    }

    /**
     * Set the motor voltage directly.
     *
     * @param volts The voltage to apply
     */
    public void setVoltage(double volts) {
        controller.setReference(volts, ControlType.kVoltage);
    }

    /**
     * Check if the arm is at a specific rotation.
     *
     * @param rotation The rotation to check
     * @return True if at the target rotation within tolerance
     */
    public boolean atRotation(Rotation2d rotation) {
        double diff = Math.abs(getAngle().getRadians() - rotation.getRadians());
        return diff < CoralArmConstants.ARM_TOLERANCE.getRadians();
    }

    /**
     * Check if the arm is at the goal rotation.
     *
     * @return True if at the goal rotation
     */
    public boolean atGoalRotation() {
        return goalRotation != null && atRotation(goalRotation);
    }

    /**
     * Get the current angle of the arm.
     *
     * @return The current angle as a Rotation2d
     */
    public Rotation2d getAngle() {
        double degrees = absoluteEncoder.getPosition() - encoderOffset;
        return Rotation2d.fromDegrees(degrees);
    }

    /**
     * Get the name of the current command.
     *
     * @return The current command name or empty string
     */
    public String getCurrentCommandName() {
        Command current = getCurrentCommand();
        return current != null ? current.getName() : "";
    }

    // ==================== Commands ====================

    /**
     * Command to set the arm to a specific angle.
     *
     * @param angle The target angle
     * @return A command that moves the arm to the angle
     */
    public Command setAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), this)
            .until(this::atGoalRotation)
            .withName("SetAngle(" + angle.getDegrees() + ")");
    }

    /**
     * SysId quasistatic command.
     *
     * @param direction The direction to run
     * @return The SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * SysId dynamic command.
     *
     * @param direction The direction to run
     * @return The SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
