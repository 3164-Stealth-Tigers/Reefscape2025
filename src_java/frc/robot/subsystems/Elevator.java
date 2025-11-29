package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Elevator subsystem for vertical lift mechanism.
 * Uses two NEO motors in leader/follower configuration.
 */
public class Elevator extends SubsystemBase {
    // Motors
    private final SparkMax leader;
    private final SparkMax follower;

    // Encoder and controller
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    // Limit switch for homing
    private final DigitalInput limitSwitch;

    // Feedforward
    private final ElevatorFeedforward feedforward;

    // Motion profile
    private final TrapezoidProfile profile;

    // Mechanism visualization
    private final Mechanism2d mechanism;
    private final MechanismLigament2d elevatorLigament;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    // Goal tracking
    private Double goalHeight = null;
    private Double goalVelocity = null;
    private String goalLevelName = null;

    /**
     * Creates a new Elevator subsystem.
     */
    public Elevator() {
        // Initialize motors
        leader = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        follower = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();

        // Initialize limit switch
        limitSwitch = new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_ID);

        // Initialize feedforward
        feedforward = new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD_CONSTANTS[0],  // kS
            ElevatorConstants.FEEDFORWARD_CONSTANTS[1],  // kG
            ElevatorConstants.FEEDFORWARD_CONSTANTS[2],  // kV
            ElevatorConstants.FEEDFORWARD_CONSTANTS[3]   // kA
        );

        // Initialize motion profile
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY,
                ElevatorConstants.MAX_ACCELERATION
            )
        );

        // Configure motors
        configureMotors();
        reset(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT);

        // Setup mechanism visualization
        mechanism = new Mechanism2d(3, 4);
        MechanismRoot2d root = mechanism.getRoot("elevator", 2, 0);
        elevatorLigament = root.append(new MechanismLigament2d("carriage", getCarriageHeight(), 90));
        SmartDashboard.putData("Elevator Mechanism", mechanism);

        // Setup SysId routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.3).per(Seconds.of(1)),
                Volts.of(2),
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                voltage -> setVoltage(voltage.in(Volts)),
                log -> {
                    log.motor("elevator")
                        .voltage(Volts.of(leader.getAppliedOutput() * leader.getBusVoltage()))
                        .linearPosition(Meters.of(encoder.getPosition()))
                        .linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
                },
                this
            )
        );
    }

    /**
     * Configure both motors with appropriate settings.
     */
    private void configureMotors() {
        // Common configuration
        double positionFactor = 2 * (1.0 / ElevatorConstants.GEAR_RATIO)
            * (ElevatorConstants.SPROCKET_PITCH_DIAMETER * Math.PI);
        double velocityFactor = positionFactor / 60.0;

        // Leader configuration
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(ElevatorConstants.INVERT_LEFT_MOTOR);

        leaderConfig.encoder
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(velocityFactor);

        // Position control (Slot 0)
        leaderConfig.closedLoop
            .p(ElevatorConstants.POSITION_kP, ClosedLoopSlot.kSlot0)
            .d(ElevatorConstants.POSITION_kD, ClosedLoopSlot.kSlot0)
            .outputRange(-0.5, ElevatorConstants.MAX_OUT_UP, ClosedLoopSlot.kSlot0);

        // Velocity control (Slot 1)
        leaderConfig.closedLoop
            .p(ElevatorConstants.VELOCITY_kP, ClosedLoopSlot.kSlot1)
            .outputRange(ElevatorConstants.MAX_OUT_DOWN, ElevatorConstants.MAX_OUT_UP, ClosedLoopSlot.kSlot1);

        // Smart Motion configuration
        leaderConfig.closedLoop.smartMotion
            .maxVelocity(ElevatorConstants.MAX_VELOCITY, ClosedLoopSlot.kSlot1)
            .minOutputVelocity(ElevatorConstants.MAX_VELOCITY / 2, ClosedLoopSlot.kSlot1)
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(ElevatorConstants.HEIGHT_TOLERANCE, ClosedLoopSlot.kSlot1);

        // Soft limits
        leaderConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT)
            .reverseSoftLimit(ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower configuration
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .follow(ElevatorConstants.LEFT_MOTOR_ID, ElevatorConstants.INVERT_RIGHT_MOTOR);

        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Update mechanism visualization
        elevatorLigament.setLength(getCarriageHeight());

        // Log data
        SmartDashboard.putNumber("Elevator/Height", getCarriageHeight());
        SmartDashboard.putNumber("Elevator/Velocity", encoder.getVelocity());
        SmartDashboard.putBoolean("Elevator/LimitSwitch", lowerLimit());
        SmartDashboard.putNumber("Elevator/Voltage", leader.getAppliedOutput() * leader.getBusVoltage());
        SmartDashboard.putBoolean("Elevator/AtGoal", atGoalHeight());

        if (goalHeight != null) {
            SmartDashboard.putNumber("Elevator/GoalHeight", goalHeight);
            SmartDashboard.putNumber("Elevator/HeightError", heightError());
        }
    }

    /**
     * Reset the encoder to a specific height.
     *
     * @param height The height to reset to in meters
     */
    public void reset(double height) {
        encoder.setPosition(height);
    }

    /**
     * Set the target height using position control.
     *
     * @param height The target height in meters
     */
    public void setHeight(double height) {
        goalHeight = height;
        controller.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * Set the target velocity using velocity control.
     *
     * @param velocity The target velocity in m/s
     */
    public void setVelocity(double velocity) {
        goalVelocity = velocity;
        controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1, 1.0 / 473.0);
    }

    /**
     * Set the motor duty cycle directly.
     *
     * @param output The duty cycle (-1 to 1)
     */
    public void setDutyCycle(double output) {
        leader.set(output);
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
     * Enable or disable soft limits.
     *
     * @param enable True to enable soft limits
     */
    public void enableSoftLimits(boolean enable) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.softLimit
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimitEnabled(enable);
        leader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Get the current carriage height.
     *
     * @return The carriage height in meters
     */
    public double getCarriageHeight() {
        return encoder.getPosition();
    }

    /**
     * Get the current carriage height in inches.
     *
     * @return The carriage height in inches
     */
    public double getCarriageHeightInches() {
        return getCarriageHeight() * 39.37;
    }

    /**
     * Check if the lower limit switch is triggered.
     *
     * @return True if the limit switch is triggered
     */
    public boolean lowerLimit() {
        // Hall Effect sensor returns false when magnet is detected
        return !limitSwitch.get();
    }

    /**
     * Check if the elevator is at a specific height.
     *
     * @param height The height to check in meters
     * @return True if at the target height within tolerance
     */
    public boolean atHeight(double height) {
        if (RobotBase.isSimulation()) {
            return true;
        }
        return Math.abs(getCarriageHeight() - height) < ElevatorConstants.HEIGHT_TOLERANCE;
    }

    /**
     * Check if the elevator is at the goal height.
     *
     * @return True if at the goal height
     */
    public boolean atGoalHeight() {
        return goalHeight != null && atHeight(goalHeight);
    }

    /**
     * Get the height error from the goal.
     *
     * @return The height error in meters
     */
    public double heightError() {
        return goalHeight != null ? goalHeight - getCarriageHeight() : 0;
    }

    /**
     * Get the velocity error from the goal.
     *
     * @return The velocity error in m/s
     */
    public double velocityError() {
        return goalVelocity != null ? goalVelocity - encoder.getVelocity() : 0;
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
     * Command to set the elevator to a specific height.
     *
     * @param height The target height in meters
     * @return A command that moves to the height
     */
    public Command setHeightCommand(double height) {
        return new SetProfiledHeightCommand(height, this)
            .andThen(Commands.runOnce(() -> setHeight(height), this));
    }

    /**
     * Command to home the elevator using the limit switch.
     *
     * @return A command that homes the elevator
     */
    public Command homeElevatorCommand() {
        return Commands.sequence(
            // Disable soft limits
            Commands.runOnce(() -> enableSoftLimits(false)),
            // If limit switch is triggered, move up first
            Commands.run(() -> setVoltage(1.5)).onlyWhile(this::lowerLimit),
            Commands.waitSeconds(0.2),
            // Move down until limit switch triggers
            Commands.run(() -> setVoltage(-1)).until(this::lowerLimit),
            // Reset position
            Commands.runOnce(() -> reset(ElevatorConstants.LIMIT_SWITCH_HEIGHT)),
            // Stop motors
            Commands.runOnce(() -> setVoltage(0)),
            // Re-enable soft limits
            Commands.runOnce(() -> enableSoftLimits(true))
        ).withName("Home Elevator");
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

    // ==================== Inner Command Class ====================

    /**
     * Command that moves the elevator to a height using a trapezoidal profile.
     */
    public static class SetProfiledHeightCommand extends Command {
        private final double goalPosition;
        private final Elevator elevator;
        private final TrapezoidProfile profile;
        private final Timer timer = new Timer();

        private TrapezoidProfile.State setpoint;
        private TrapezoidProfile.State goal;

        public SetProfiledHeightCommand(double height, Elevator elevator) {
            this.goalPosition = height;
            this.elevator = elevator;
            this.profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.MAX_VELOCITY,
                    ElevatorConstants.MAX_ACCELERATION
                )
            );
            addRequirements(elevator);
            setName("ProfiledHeightCommand");
        }

        @Override
        public void initialize() {
            setpoint = new TrapezoidProfile.State(
                elevator.getCarriageHeight(),
                elevator.encoder.getVelocity()
            );
            goal = new TrapezoidProfile.State(goalPosition, 0);
            elevator.goalHeight = goalPosition;
            timer.restart();
        }

        @Override
        public void execute() {
            elevator.setHeight(setpoint.position);
            setpoint = profile.calculate(0.02, setpoint, goal);
        }

        @Override
        public void end(boolean interrupted) {
            timer.stop();
        }

        @Override
        public boolean isFinished() {
            return profile.isFinished(timer.get()) && elevator.atGoalHeight();
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(elevator);
        }
    }
}
