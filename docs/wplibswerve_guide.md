# WPILib Native Swerve Implementation Guide

A comprehensive guide to implementing swerve drive using WPILib's native swerve classes for FRC robots in Java.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Configuration Step-by-Step](#configuration-step-by-step)
4. [SwerveModule Class](#swervemodule-class)
5. [SwerveDrive Subsystem](#swervedrive-subsystem)
6. [Teleop Control](#teleop-control)
7. [Autonomous Integration](#autonomous-integration)
8. [Tuning Guide](#tuning-guide)
9. [Troubleshooting](#troubleshooting)

---

## Overview

WPILib provides native swerve drive classes that handle all the mathematics for swerve drive control. This guide covers implementing swerve drive using:

- **SwerveDriveKinematics** - Converts chassis speeds to module states
- **SwerveDriveOdometry** - Tracks robot position from wheel movements
- **SwerveDrivePoseEstimator** - Combines odometry with vision measurements
- **SwerveModuleState** - Represents a module's speed and angle
- **SwerveModulePosition** - Represents a module's distance and angle

### Key Concepts

| Term | Definition |
|------|------------|
| **Swerve Module** | A wheel assembly with two motors: one drives the wheel, one steers it |
| **Drive Motor** | The motor that spins the wheel (provides velocity) |
| **Azimuth Motor** | The motor that rotates/steers the wheel (provides angle) |
| **Field-Relative** | Robot moves relative to the field (forward is always toward opposing alliance) |
| **Robot-Relative** | Robot moves relative to itself (forward is where the robot is facing) |
| **Open-Loop** | Motor runs at a percentage without feedback |
| **Closed-Loop** | Motor uses encoder feedback to maintain exact velocity |
| **CANCoder** | Absolute encoder that knows wheel angle even after power cycle |

### Comparison with swervepy (Python)

| Feature | swervepy (Python) | WPILib Native (Java) |
|---------|-------------------|---------------------|
| Language | Python | Java |
| Kinematics | Internal | SwerveDriveKinematics |
| Pose Estimation | Internal | SwerveDrivePoseEstimator |
| Module Abstraction | CoaxialSwerveModule | Custom SwerveModule class |
| Units | Pint library (`u.inch`, `u.m`) | WPILib Units class |
| Configuration | TypicalDriveComponentParameters | Constants class + SparkMaxConfig |

---

## Architecture

```
src_java/frc/robot/
├── Constants.java                    # All swerve constants
├── subsystems/
│   └── swerve/
│       ├── SwerveDrive.java         # Main swerve subsystem
│       └── SwerveModule.java        # Individual module control
└── commands/
    └── SwerveCommands.java          # Swerve-related commands
```

### Component Hierarchy

```
SwerveDrive (SubsystemBase)
├── Pigeon2 Gyro (CAN ID 0)
├── SwerveDriveKinematics
├── SwerveDrivePoseEstimator
└── SwerveModules (4x SwerveModule)
    ├── SparkMax Drive Motor (NEO)
    │   └── Integrated Encoder
    ├── SparkMax Azimuth Motor (NEO)
    │   └── Integrated Encoder
    └── CANCoder (Absolute Position)
```

### WPILib Swerve Classes

```
+-----------------------------------+
|     SwerveDriveKinematics        |
|-----------------------------------|
| • Converts ChassisSpeeds to      |
|   SwerveModuleState[]            |
| • Converts module states to      |
|   ChassisSpeeds                  |
| • Desaturates wheel speeds       |
+-----------------------------------+
           |
           v
+-----------------------------------+
|     SwerveDrivePoseEstimator     |
|-----------------------------------|
| • Combines odometry + vision     |
| • Tracks robot position          |
| • Returns Pose2d                 |
+-----------------------------------+
           |
           v
+-----------------------------------+
|     SwerveModuleState            |
|-----------------------------------|
| • speedMetersPerSecond           |
| • angle (Rotation2d)             |
| • optimize() - reduces rotation  |
+-----------------------------------+
```

---

## Configuration Step-by-Step

### Step 1: Define Physical Constants

In `Constants.java`, create a `SwerveConstants` class:

```java
public final class Constants {
    public static final class SwerveConstants {
        // Physical dimensions (in METERS)
        public static final double TRACK_WIDTH = Units.inchesToMeters(17.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(29.75);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0 * Math.PI);

        // Gear ratios (motor rotations per wheel rotation)
        public static final double DRIVE_GEAR_RATIO = 6.75;     // SDS Mk4i L2
        public static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;  // SDS Mk4i

        // Max speeds
        public static final double MAX_SPEED = 4.2;  // meters per second
        public static final double MAX_ANGULAR_VELOCITY = 9.547;  // radians per second
    }
}
```

### Step 2: Define Motor CAN IDs

```java
public static final class SwerveConstants {
    // ... physical constants above ...

    // Pigeon Gyro
    public static final int PIGEON_ID = 0;

    // Front Left Module
    public static final int FL_DRIVE_ID = 7;
    public static final int FL_AZIMUTH_ID = 8;
    public static final int FL_CANCODER_ID = 1;

    // Front Right Module
    public static final int FR_DRIVE_ID = 5;
    public static final int FR_AZIMUTH_ID = 6;
    public static final int FR_CANCODER_ID = 2;

    // Rear Left Module
    public static final int RL_DRIVE_ID = 3;
    public static final int RL_AZIMUTH_ID = 4;
    public static final int RL_CANCODER_ID = 3;

    // Rear Right Module
    public static final int RR_DRIVE_ID = 1;
    public static final int RR_AZIMUTH_ID = 2;
    public static final int RR_CANCODER_ID = 4;
}
```

### Step 3: Define Encoder Offsets

Each module needs an offset so the wheel knows which way is "forward":

```java
public static final class SwerveConstants {
    // ... previous constants ...

    // Encoder offsets (in DEGREES)
    // To find: rotate wheel forward, read CANCoder value
    public static final double FL_ENCODER_OFFSET = 199.072266;
    public static final double FR_ENCODER_OFFSET = 89.208984;
    public static final double RL_ENCODER_OFFSET = 64.863281;
    public static final double RR_ENCODER_OFFSET = 37.529297;
}
```

**How to calibrate offsets:**
1. Put robot on blocks (wheels off ground)
2. Manually rotate each wheel to point forward (toward front of robot)
3. Read the CANCoder value from Phoenix Tuner or SmartDashboard
4. Use that value as the offset

### Step 4: Define PID and Feedforward Gains

```java
public static final class SwerveConstants {
    // ... previous constants ...

    // Drive motor PID (tune with SysId)
    public static final double DRIVE_kP = 0.064395;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;

    // Drive motor feedforward (tune with SysId)
    public static final double DRIVE_kS = 0.18656;  // Static friction
    public static final double DRIVE_kV = 2.5833;   // Velocity
    public static final double DRIVE_kA = 0.40138;  // Acceleration

    // Azimuth motor PID
    public static final double AZIMUTH_kP = 0.01;
    public static final double AZIMUTH_kI = 0.0;
    public static final double AZIMUTH_kD = 0.0;

    // Current limits (Amps)
    public static final int DRIVE_CURRENT_LIMIT = 60;
    public static final int AZIMUTH_CURRENT_LIMIT = 30;

    // Ramp rates (seconds to full power)
    public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;
}
```

---

## SwerveModule Class

Each swerve module controls one wheel assembly.

### Class Structure

```java
public class SwerveModule {
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

    // Offset
    private final Rotation2d encoderOffset;
}
```

### Constructor

```java
public SwerveModule(int moduleNumber, int driveMotorId, int azimuthMotorId,
                    int canCoderId, double encoderOffset) {
    this.encoderOffset = Rotation2d.fromDegrees(encoderOffset);

    // Initialize motors
    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    azimuthMotor = new SparkMax(azimuthMotorId, MotorType.kBrushless);

    // Get encoders and controllers
    driveEncoder = driveMotor.getEncoder();
    azimuthEncoder = azimuthMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    azimuthController = azimuthMotor.getClosedLoopController();

    // Initialize CANCoder
    canCoder = new CANcoder(canCoderId);

    // Configure feedforward
    driveFeedforward = new SimpleMotorFeedforward(
        SwerveConstants.DRIVE_kS,
        SwerveConstants.DRIVE_kV,
        SwerveConstants.DRIVE_kA
    );

    configureMotors();
    resetToAbsolute();
}
```

### Motor Configuration

```java
private void configureMotors() {
    // Drive motor configuration
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT)
        .openLoopRampRate(SwerveConstants.DRIVE_OPEN_LOOP_RAMP);

    // Convert rotations to meters
    double drivePositionFactor = SwerveConstants.WHEEL_CIRCUMFERENCE
                                 / SwerveConstants.DRIVE_GEAR_RATIO;
    double driveVelocityFactor = drivePositionFactor / 60.0;

    driveConfig.encoder
        .positionConversionFactor(drivePositionFactor)
        .velocityConversionFactor(driveVelocityFactor);

    driveConfig.closedLoop
        .p(SwerveConstants.DRIVE_kP)
        .i(SwerveConstants.DRIVE_kI)
        .d(SwerveConstants.DRIVE_kD);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters,
                         PersistMode.kPersistParameters);

    // Azimuth motor configuration
    SparkMaxConfig azimuthConfig = new SparkMaxConfig();
    azimuthConfig
        .idleMode(IdleMode.kBrake)  // Always brake for steering
        .smartCurrentLimit(SwerveConstants.AZIMUTH_CURRENT_LIMIT)
        .inverted(true);

    // Convert rotations to degrees
    double azimuthPositionFactor = 360.0 / SwerveConstants.AZIMUTH_GEAR_RATIO;

    azimuthConfig.encoder
        .positionConversionFactor(azimuthPositionFactor)
        .velocityConversionFactor(azimuthPositionFactor / 60.0);

    azimuthConfig.closedLoop
        .p(SwerveConstants.AZIMUTH_kP)
        .i(SwerveConstants.AZIMUTH_kI)
        .d(SwerveConstants.AZIMUTH_kD)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(360);

    azimuthMotor.configure(azimuthConfig, ResetMode.kResetSafeParameters,
                           PersistMode.kPersistParameters);
}
```

### Key Methods

```java
// Get absolute angle from CANCoder (survives power cycle)
public Rotation2d getAbsoluteAngle() {
    double angle = canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    return Rotation2d.fromDegrees(angle).minus(encoderOffset);
}

// Reset integrated encoder to absolute position
public void resetToAbsolute() {
    double absolutePosition = getAbsoluteAngle().getDegrees();
    azimuthEncoder.setPosition(absolutePosition);
}

// Get current state (velocity + angle)
public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
                                  Rotation2d.fromDegrees(azimuthEncoder.getPosition()));
}

// Get current position (distance + angle) for odometry
public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(),
                                     Rotation2d.fromDegrees(azimuthEncoder.getPosition()));
}

// Set desired state with optimization
public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
    // Optimize to avoid rotating more than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, getAngle());

    // Set azimuth position
    azimuthController.setReference(desiredState.angle.getDegrees(),
                                    SparkMax.ControlType.kPosition);

    // Set drive velocity
    if (openLoop) {
        double percentOutput = desiredState.speedMetersPerSecond
                               / SwerveConstants.MAX_SPEED;
        driveMotor.set(percentOutput);
    } else {
        double ff = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        driveController.setReference(desiredState.speedMetersPerSecond,
                                      SparkMax.ControlType.kVelocity, 0, ff);
    }
}
```

---

## SwerveDrive Subsystem

The main subsystem that coordinates all four modules.

### Class Structure

```java
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] modules;
    private final Pigeon2 gyro;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean fieldRelative = true;
}
```

### Constructor

```java
public SwerveDrive() {
    // Initialize gyro
    gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
    gyro.reset();

    // Define module positions relative to robot center
    // +X = forward, +Y = left
    Translation2d[] moduleLocations = {
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),   // FL: front-left
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // FR: front-right
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  // RL: rear-left
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)  // RR: rear-right
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(moduleLocations);

    // Initialize modules
    modules = new SwerveModule[] {
        new SwerveModule(0, FL_DRIVE_ID, FL_AZIMUTH_ID, FL_CANCODER_ID, FL_ENCODER_OFFSET),
        new SwerveModule(1, FR_DRIVE_ID, FR_AZIMUTH_ID, FR_CANCODER_ID, FR_ENCODER_OFFSET),
        new SwerveModule(2, RL_DRIVE_ID, RL_AZIMUTH_ID, RL_CANCODER_ID, RL_ENCODER_OFFSET),
        new SwerveModule(3, RR_DRIVE_ID, RR_AZIMUTH_ID, RR_CANCODER_ID, RR_ENCODER_OFFSET)
    };

    // Initialize pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getYaw(),
        getModulePositions(),
        new Pose2d()  // Starting pose
    );
}
```

### Drive Methods

```java
/**
 * Drive the robot with given velocities.
 * @param translation Translation velocity (x forward, y left) in m/s
 * @param rotation Rotational velocity in rad/s
 * @param fieldRelative Whether to drive field-relative
 * @param openLoop Whether to use open-loop control
 */
public void drive(Translation2d translation, double rotation,
                  boolean fieldRelative, boolean openLoop) {
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
 * Drive with chassis speeds directly.
 */
public void drive(ChassisSpeeds speeds, boolean openLoop) {
    // Discretize to reduce drift at high speeds
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert to module states
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    // Desaturate wheel speeds (keeps ratios, limits max speed)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);

    // Set module states
    for (int i = 0; i < modules.length; i++) {
        modules[i].setDesiredState(states[i], openLoop);
    }
}
```

### X-Lock (Ski Stop)

```java
/**
 * Set modules to X pattern to resist pushing.
 */
public void setX() {
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
}

/*
   X-Lock Pattern:

      FL  \  /  FR
           \/
           /\
      RL  /  \  RR
*/
```

### Odometry and Pose

```java
@Override
public void periodic() {
    // Update pose estimate every loop
    poseEstimator.update(getYaw(), getModulePositions());
}

public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
}

public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
}

public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(pose, timestampSeconds);
}

public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
}
```

---

## Teleop Control

### Basic Teleop Command

```java
public Command teleopCommand(DoubleSupplier forward, DoubleSupplier strafe,
                             DoubleSupplier turn) {
    return new RunCommand(() -> {
        // Convert joystick inputs to velocities
        double vx = forward.getAsDouble() * SwerveConstants.MAX_SPEED;
        double vy = strafe.getAsDouble() * SwerveConstants.MAX_SPEED;
        double omega = turn.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;

        // Flip for red alliance (field-relative)
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            vx = -vx;
            vy = -vy;
        }

        drive(new Translation2d(vx, vy), omega, fieldRelative, true);
    }, this);
}
```

### Setting Up in RobotContainer

```java
public class RobotContainer {
    private final SwerveDrive swerve = new SwerveDrive();
    private final CommandXboxController driver = new CommandXboxController(0);

    public RobotContainer() {
        // Set default command with joystick input
        swerve.setDefaultCommand(
            swerve.teleopCommand(
                () -> -driver.getLeftY(),    // Forward (inverted because Y is up)
                () -> -driver.getLeftX(),    // Strafe (inverted for field coords)
                () -> -driver.getRightX()    // Turn (inverted for CCW+)
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset gyro to 180 degrees (facing away from driver)
        driver.start().onTrue(
            Commands.runOnce(() -> swerve.resetYaw(Rotation2d.fromDegrees(180)))
        );

        // Toggle field-relative mode
        driver.back().onTrue(
            Commands.runOnce(swerve::toggleFieldRelative)
        );

        // X-lock while held
        driver.y().whileTrue(swerve.skiStopCommand());
    }
}
```

### Input Shaping (Recommended)

Apply exponential curves for smoother control:

```java
private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
        return 0.0;
    }
    return Math.signum(value) * ((Math.abs(value) - deadband) / (1.0 - deadband));
}

private double squareInput(double value) {
    return Math.copySign(value * value, value);
}

// In RobotContainer:
swerve.setDefaultCommand(
    swerve.teleopCommand(
        () -> squareInput(applyDeadband(-driver.getLeftY(), 0.08)),
        () -> squareInput(applyDeadband(-driver.getLeftX(), 0.08)),
        () -> squareInput(applyDeadband(-driver.getRightX(), 0.08))
    )
);
```

---

## Autonomous Integration

### PathPlanner Setup

```java
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;

public class SwerveDrive extends SubsystemBase {

    public SwerveDrive() {
        // ... existing initialization ...

        configurePathPlanner();
    }

    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,                 // Pose supplier
                this::resetPose,               // Pose reset consumer
                this::getRobotRelativeSpeeds,  // ChassisSpeeds supplier
                (speeds, feedforwards) -> drive(speeds, false),  // Drive consumer
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),  // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)   // Rotation PID
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent()
                           && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config", e.getStackTrace());
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
}
```

### DriveToPose Command

```java
public class DriveToPoseCommand extends Command {
    private final SwerveDrive swerve;
    private final Pose2d targetPose;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public DriveToPoseCommand(SwerveDrive swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        xController = new PIDController(2.0, 0, 0);
        yController = new PIDController(2.0, 0, 0);
        thetaController = new PIDController(4.0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        swerve.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
                                                   currentPose.getRotation()),
            false
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d current = swerve.getPose();
        double posError = current.getTranslation().getDistance(targetPose.getTranslation());
        double rotError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());
        return posError < 0.05 && rotError < Math.toRadians(3);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
```

---

## Tuning Guide

### 1. Encoder Offsets

**Process:**
1. Put robot on blocks (wheels off ground)
2. Rotate each wheel so it points forward
3. Read CANCoder value from Phoenix Tuner or SmartDashboard
4. Use that value as the offset
5. Deploy and verify all wheels point same direction when driving forward

### 2. Drive Motor PID (Use SysId)

| Gain | Starting Value | Adjust If... |
|------|---------------|--------------|
| kP | 0.05-0.1 | Robot sluggish (increase) or oscillates (decrease) |
| kI | 0 | Usually not needed |
| kD | 0 | Add small amount if oscillating |

### 3. Drive Motor Feedforward (Use SysId)

| Gain | Description | Typical Range |
|------|-------------|---------------|
| kS | Static friction (voltage to start moving) | 0.1-0.3 |
| kV | Velocity gain (voltage per m/s) | 2.0-3.0 |
| kA | Acceleration gain (voltage per m/s^2) | 0.2-0.5 |

### 4. Azimuth Motor PID

| Gain | Starting Value | Adjust If... |
|------|---------------|--------------|
| kP | 0.01-0.05 | Wheels slow to turn (increase) or vibrate (decrease) |
| kI | 0 | Usually not needed |
| kD | 0-0.001 | Add if oscillating around target |

### 5. Running SysId

Add button bindings for characterization:

```java
// In RobotContainer
CommandXboxController sysIdController = new CommandXboxController(3);

sysIdController.y().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
sysIdController.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
sysIdController.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
sysIdController.x().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
```

In SwerveDrive, create SysId routines:

```java
private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        voltage -> {
            for (SwerveModule module : modules) {
                module.setDriveVoltage(voltage.in(Volts));
            }
        },
        log -> {
            for (int i = 0; i < modules.length; i++) {
                log.motor("drive-" + i)
                    .voltage(Volts.of(modules[i].getDriveVoltage()))
                    .linearPosition(Meters.of(modules[i].getPosition()))
                    .linearVelocity(MetersPerSecond.of(modules[i].getVelocity()));
            }
        },
        this
    )
);

public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
}
```

---

## Troubleshooting

### Wheels Point Wrong Direction

**Symptom:** Wheels don't all point the same way when driving forward

**Cause:** Encoder offsets are wrong

**Fix:**
1. Put robot on blocks
2. Rotate each wheel to point forward
3. Read CANCoder values
4. Update offsets in Constants.java
5. Redeploy and test

### Robot Drifts When Driving Straight

**Symptom:** Robot curves instead of going straight

**Possible Causes:**
- Gyro not calibrated (let robot sit still for 10 seconds after power on)
- Drive motor inversions inconsistent
- One wheel has wrong offset

**Fix:** Check gyro calibration and verify all wheels respond identically

### Robot Spins Uncontrollably

**Symptom:** Robot spins when you try to drive forward

**Cause:** Azimuth motors fighting each other (opposite inversions)

**Fix:** Check azimuth `inverted` settings - some modules may need opposite direction

### Wheels Jitter/Vibrate

**Symptom:** Wheels oscillate rapidly when stopped

**Cause:** Azimuth kP too high

**Fix:** Reduce azimuth kP (try halving it)

### Robot Doesn't Follow Auto Paths

**Symptom:** Robot drifts off path in autonomous

**Cause:** Auto PID gains too low

**Fix:** Increase translation and rotation PID gains in PathPlanner config

### Pose Jumps Around

**Symptom:** Robot position jumps erratically on dashboard

**Cause:** Vision measurements not filtered properly

**Fix:** Add standard deviation filtering to vision measurements

```java
// Filter vision by distance
public void addVisionMeasurement(Pose2d pose, double timestamp, double distance) {
    if (distance > 4.0) return;  // Ignore far measurements

    // Increase uncertainty with distance
    double xyStdDev = 0.5 * distance;
    double thetaStdDev = Math.toRadians(5) * distance;

    poseEstimator.addVisionMeasurement(
        pose, timestamp,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
    );
}
```

### Motor Brownout

**Symptom:** Robot loses power during aggressive maneuvers

**Cause:** Current limits too high

**Fix:** Reduce `DRIVE_CURRENT_LIMIT` (try 40A instead of 60A)

---

## Quick Reference Card

### Coordinate System

```
        +X (Forward)
            ^
            |
   +Y <-----+-----> -Y
  (Left)    |     (Right)
            v
        -X (Back)

Rotation: Counter-clockwise positive (CCW+)
```

### Module Positions

```java
// Standard 4-module layout relative to robot center
FL = new Translation2d(+WHEEL_BASE/2, +TRACK_WIDTH/2);  // Front Left
FR = new Translation2d(+WHEEL_BASE/2, -TRACK_WIDTH/2);  // Front Right
RL = new Translation2d(-WHEEL_BASE/2, +TRACK_WIDTH/2);  // Rear Left
RR = new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2);  // Rear Right
```

### Common Gear Ratios

| Module Type | Drive Ratio | Azimuth Ratio |
|-------------|-------------|---------------|
| SDS Mk4i L1 | 8.14:1 | 150/7 (21.43:1) |
| SDS Mk4i L2 | 6.75:1 | 150/7 (21.43:1) |
| SDS Mk4i L3 | 6.12:1 | 150/7 (21.43:1) |
| SDS Mk4 L2 | 6.75:1 | 12.8:1 |
| MAXSwerve | 4.71:1 | Direct drive |

### Unit Conversions

```java
import edu.wpi.first.math.util.Units;

// Length
double meters = Units.inchesToMeters(24);    // inches -> meters
double inches = Units.metersToInches(0.6);   // meters -> inches

// Angles
double radians = Units.degreesToRadians(90); // degrees -> radians
double degrees = Units.radiansToDegrees(Math.PI); // radians -> degrees
```

### Essential Imports

```java
// WPILib swerve classes
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

// Geometry
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

// Control
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

// Hardware
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.CANcoder;
```

---

## Additional Resources

- [WPILib Swerve Documentation](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
- [PathPlanner Documentation](https://pathplanner.dev/home.html)
- [REV Robotics SparkMAX Documentation](https://docs.revrobotics.com/brushless)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)
- [SDS Mk4i Module Specs](https://www.swervedrivespecialties.com/products/mk4i-swerve-module)

---

*Last Updated: November 2025*
*Team 3164 Stealth Tigers*
