package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
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
import frc.robot.Constants.AlgaeArmConstants;

/**
 * Algae arm subsystem for secondary manipulation.
 * Uses a SparkFlex motor with an absolute encoder.
 */
public class AlgaeArm extends SubsystemBase {
    // Motor and encoder
    private final SparkFlex motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController controller;

    /**
     * Creates a new AlgaeArm subsystem.
     */
    public AlgaeArm() {
        // Initialize motor
        motor = new SparkFlex(AlgaeArmConstants.MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        configureMotor();
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
            .velocityConversionFactor(360.0 / 60.0)
            .inverted(true);

        // Configure relative encoder
        config.encoder
            .positionConversionFactor(360.0 / AlgaeArmConstants.GEAR_RATIO)
            .velocityConversionFactor(360.0 / AlgaeArmConstants.GEAR_RATIO / 60.0);

        // Configure closed-loop control
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(AlgaeArmConstants.kP)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeArm/Angle", getAngle().getDegrees());
    }

    /**
     * Set the arm to a specific angle.
     * 0 degrees is horizontal, facing toward the front of the robot.
     *
     * @param angle Counter-clockwise positive angle
     */
    public void setAngle(Rotation2d angle) {
        controller.setReference(angle.getDegrees(), ControlType.kPosition);
    }

    /**
     * Get the current angle of the arm.
     *
     * @return The current angle as a Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
    }

    /**
     * Set the motor duty cycle directly.
     *
     * @param output The duty cycle (-1 to 1)
     */
    public void setDutyCycle(double output) {
        motor.set(output);
    }

    // ==================== Commands ====================

    /**
     * Command to set the arm to a specific angle.
     *
     * @param angle The target angle
     * @return A command that moves the arm to the angle
     */
    public Command setAngleCommand(Rotation2d angle) {
        return Commands.runOnce(() -> setAngle(angle), this)
            .withName("SetAlgaeAngle(" + angle.getDegrees() + ")");
    }
}
