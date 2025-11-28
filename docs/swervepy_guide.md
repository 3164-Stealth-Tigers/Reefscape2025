# Swervepy Implementation Guide

A comprehensive guide to implementing swerve drive using the swervepy library for FRC robots.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Installation](#installation)
4. [Configuration Step-by-Step](#configuration-step-by-step)
5. [Component Reference](#component-reference)
6. [SwerveDrive Subsystem API](#swervedrive-subsystem-api)
7. [Teleop Control](#teleop-control)
8. [Autonomous Integration](#autonomous-integration)
9. [Tuning Guide](#tuning-guide)
10. [Troubleshooting](#troubleshooting)

---

## Overview

**swervepy** is a modular Python library for FRC swerve drive systems. It provides:

- Abstract base classes for extensibility
- Pre-built implementations for common hardware (NEO, Falcon 500, CANCoder, Pigeon)
- Built-in teleop commands
- PathPlanner integration for autonomous
- SysId characterization support
- Simulation capabilities

**Repository**: [github.com/EWall25/swervepy](https://github.com/EWall25/swervepy)

### Key Concepts

| Term | Definition |
|------|------------|
| **Coaxial Module** | A swerve module where one motor drives the wheel and another rotates it (most common type) |
| **Drive Component** | The motor that spins the wheel (provides velocity) |
| **Azimuth Component** | The motor that rotates/steers the wheel (provides angle) |
| **Field-Relative** | Robot moves relative to the field (forward is always toward opposing alliance) |
| **Robot-Relative** | Robot moves relative to itself (forward is where the robot is facing) |
| **Open-Loop** | Motor runs at a percentage without feedback |
| **Closed-Loop** | Motor uses encoder feedback to maintain exact velocity |

---

## Architecture

```
swervepy/
├── __init__.py          # Exports: SwerveDrive, TrajectoryFollowerParameters, u
├── subsystem.py         # SwerveDrive class (main subsystem)
├── conversions.py       # Unit conversion utilities
├── abstract/            # Abstract base classes
│   ├── motor.py         # CoaxialDriveComponent, CoaxialAzimuthComponent
│   ├── sensor.py        # Gyro, AbsoluteEncoder
│   └── system.py        # SwerveModule
└── impl/                # Concrete implementations
    ├── motor.py         # NEO, Falcon500 drive/azimuth components
    ├── sensor.py        # CANCoder, DutyCycleEncoder, Pigeon, Pigeon2
    └── swerve_module.py # CoaxialSwerveModule
```

### Component Hierarchy

```
SwerveDrive (Subsystem)
├── Gyro (Pigeon2Gyro)
└── SwerveModules (4x CoaxialSwerveModule)
    ├── DriveComponent (NEOCoaxialDriveComponent)
    ├── AzimuthComponent (NEOCoaxialAzimuthComponent)
    │   └── AbsoluteEncoder (AbsoluteCANCoder)
    └── Placement (Translation2d)
```

---

## Installation

swervepy is typically vendored (copied) into your project rather than installed via pip.

### Option 1: Clone the Repository

```bash
git clone https://github.com/EWall25/swervepy.git
cp -r swervepy/swervepy your_robot/src/swervepy
```

### Option 2: Git Submodule

```bash
git submodule add https://github.com/EWall25/swervepy.git
```

### Required Dependencies

Add to your `pyproject.toml`:

```toml
[tool.robotpy]
robotpy_version = "2025.3.1.1"
robotpy_extras = [
    "commands2",
    "phoenix6",      # For Pigeon2, CANCoder
    "rev",           # For NEO motors with SparkMax
]
requires = ["Pint"]  # Unit handling
```

---

## Configuration Step-by-Step

### Step 1: Create Configuration File

Create `swerve_config.py` in your `src/` directory:

```python
import math
from wpimath.geometry import Translation2d, Rotation2d
from swervepy import u, TrajectoryFollowerParameters
from swervepy.impl import (
    TypicalDriveComponentParameters,
    TypicalAzimuthComponentParameters,
    NeutralMode,
    NEOCoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
    AbsoluteCANCoder,
    Pigeon2Gyro,
    CoaxialSwerveModule,
)
```

### Step 2: Define Physical Dimensions

```python
# Robot dimensions (convert to meters for internal use)
TRACK_WIDTH = (17.75 * u.inch).m_as(u.m)   # Distance between left and right wheels
WHEEL_BASE = (29.75 * u.inch).m_as(u.m)    # Distance between front and back wheels

# Speed limits
MAX_VELOCITY = 4.2 * (u.m / u.s)           # Maximum translational speed
MAX_ANGULAR_VELOCITY = 9.547 * (u.rad / u.s)  # Maximum rotational speed
```

### Step 3: Define Encoder Offsets

Each swerve module needs an encoder offset so the wheel knows which way is "forward". These are measured in degrees.

```python
# Encoder offsets (calibrated per-robot)
# To find these: rotate each wheel to face forward, read the CANCoder value
FL_ENCODER_OFFSET = 19.072266 + 180   # Front Left
FR_ENCODER_OFFSET = 269.208984 - 180  # Front Right
RL_ENCODER_OFFSET = 244.863281 - 180  # Rear Left
RR_ENCODER_OFFSET = 217.529297 - 180  # Rear Right
```

**How to calibrate offsets:**
1. Put robot on blocks (wheels off ground)
2. Manually rotate each wheel so it points forward
3. Read the CANCoder value from Phoenix Tuner or SmartDashboard
4. Use that value as the offset (may need +/- 180 depending on motor direction)

### Step 4: Configure Drive Motor Parameters

```python
DRIVE_PARAMS = TypicalDriveComponentParameters(
    # Physical specifications
    wheel_circumference=4 * math.pi * u.inch,  # 4" diameter wheel
    gear_ratio=6.75 / 1,                        # SDS Mk4i L2 ratio
    max_speed=MAX_VELOCITY,

    # Ramp rates (seconds to full power)
    open_loop_ramp_rate=0.25,   # Teleop smoothing
    closed_loop_ramp_rate=0,    # Auto (no smoothing for precision)

    # Current limits (amps)
    continuous_current_limit=60,
    peak_current_limit=80,
    peak_current_duration=0.01,

    # Neutral behavior
    neutral_mode=NeutralMode.COAST,  # COAST or BRAKE

    # PID gains (tune with SysId)
    kP=0.064395,
    kI=0,
    kD=0,

    # Feedforward gains (tune with SysId)
    kS=0.18656,   # Static friction
    kV=2.5833,    # Velocity
    kA=0.40138,   # Acceleration

    # Motor direction
    invert_motor=False,
)
```

### Step 5: Configure Azimuth Motor Parameters

```python
AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
    gear_ratio=150 / 7,           # SDS Mk4i steering ratio
    max_angular_velocity=MAX_ANGULAR_VELOCITY,
    ramp_rate=0,                  # No smoothing for steering

    # Current limits
    continuous_current_limit=30,
    peak_current_limit=40,
    peak_current_duration=0.01,

    # Always BRAKE so wheels hold position
    neutral_mode=NeutralMode.BRAKE,

    # PID gains
    kP=0.01,
    kI=0,
    kD=0,

    # Motor direction
    invert_motor=True,  # Often inverted for correct steering direction
)
```

### Step 6: Create the Gyro

```python
GYRO = Pigeon2Gyro(
    can_id=0,       # CAN ID of the Pigeon 2.0
    invert=False    # Set True if gyro reads backwards
)
```

### Step 7: Create Swerve Modules

Each module needs:
- Drive motor (CAN ID)
- Azimuth motor (CAN ID + encoder offset + CANCoder)
- Placement (position relative to robot center)

```python
SWERVE_MODULES = (
    # Front Left
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(7, DRIVE_PARAMS),    # Drive motor ID: 7
        NEOCoaxialAzimuthComponent(
            8,                                          # Azimuth motor ID: 8
            Rotation2d.fromDegrees(FL_ENCODER_OFFSET),
            AZIMUTH_PARAMS,
            AbsoluteCANCoder(1)                         # CANCoder ID: 1
        ),
        Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),  # +X (front), +Y (left)
    ),

    # Front Right
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(5, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(
            6,
            Rotation2d.fromDegrees(FR_ENCODER_OFFSET),
            AZIMUTH_PARAMS,
            AbsoluteCANCoder(2)
        ),
        Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  # +X (front), -Y (right)
    ),

    # Rear Left
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(3, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(
            4,
            Rotation2d.fromDegrees(RL_ENCODER_OFFSET),
            AZIMUTH_PARAMS,
            AbsoluteCANCoder(3)
        ),
        Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  # -X (back), +Y (left)
    ),

    # Rear Right
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(1, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(
            2,
            Rotation2d.fromDegrees(RR_ENCODER_OFFSET),
            AZIMUTH_PARAMS,
            AbsoluteCANCoder(4)
        ),
        Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),  # -X (back), -Y (right)
    ),
)
```

### Step 8: Configure Autonomous Parameters

```python
AUTONOMOUS_PARAMS = TrajectoryFollowerParameters(
    theta_kP=4,           # Rotation P gain for path following
    xy_kP=2,              # Translation P gain for path following
    drive_open_loop=True  # Use open-loop during auto (usually True)
)
```

---

## Component Reference

### TypicalDriveComponentParameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `wheel_circumference` | Quantity | Wheel diameter * pi (e.g., `4 * math.pi * u.inch`) |
| `gear_ratio` | float | Motor rotations per wheel rotation (e.g., `6.75` for Mk4i L2) |
| `max_speed` | Quantity | Max velocity (e.g., `4.2 * (u.m / u.s)`) |
| `open_loop_ramp_rate` | float | Seconds to reach full power (teleop) |
| `closed_loop_ramp_rate` | float | Seconds to reach full power (auto) |
| `continuous_current_limit` | int | Steady-state current limit (amps) |
| `peak_current_limit` | int | Burst current limit (amps) |
| `peak_current_duration` | float | How long peak current allowed (seconds) |
| `neutral_mode` | NeutralMode | `COAST` or `BRAKE` when stopped |
| `kP`, `kI`, `kD` | float | PID gains for velocity control |
| `kS`, `kV`, `kA` | float | Feedforward gains |
| `invert_motor` | bool | Reverse motor direction |

### TypicalAzimuthComponentParameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `gear_ratio` | float | Motor rotations per wheel rotation (e.g., `150/7` for Mk4i) |
| `max_angular_velocity` | Quantity | Max rotation speed |
| `ramp_rate` | float | Seconds to reach full power |
| `continuous_current_limit` | int | Steady-state current limit |
| `peak_current_limit` | int | Burst current limit |
| `peak_current_duration` | float | How long peak current allowed |
| `neutral_mode` | NeutralMode | Should be `BRAKE` to hold position |
| `kP`, `kI`, `kD` | float | PID gains for position control |
| `invert_motor` | bool | Reverse motor direction |

### Common Gear Ratios

| Module Type | Drive Ratio | Azimuth Ratio |
|-------------|-------------|---------------|
| SDS Mk4i L1 | 8.14:1 | 150/7 |
| SDS Mk4i L2 | 6.75:1 | 150/7 |
| SDS Mk4i L3 | 6.12:1 | 150/7 |
| SDS Mk4 L1 | 8.14:1 | 12.8:1 |
| SDS Mk4 L2 | 6.75:1 | 12.8:1 |
| MAXSwerve | 4.71:1 | N/A (direct) |

---

## SwerveDrive Subsystem API

### Constructor

```python
from swervepy import SwerveDrive

swerve = SwerveDrive(
    modules=SWERVE_MODULES,           # Tuple of 4 CoaxialSwerveModule
    gyro=GYRO,                        # Pigeon2Gyro instance
    max_velocity=MAX_VELOCITY,        # Max translational speed
    max_angular_velocity=MAX_ANGULAR_VELOCITY,  # Max rotational speed
    path_following_params=AUTONOMOUS_PARAMS,    # Optional: for auto
    vision_pose_callback=vision.get_pose_estimation,  # Optional: for vision
)
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pose` | Pose2d | Current robot pose on field |
| `heading` | Rotation2d | Current robot heading |
| `module_states` | Tuple[SwerveModuleState] | Current velocity/angle of each module |
| `module_positions` | Tuple[SwerveModulePosition] | Current position of each module |
| `robot_relative_speeds` | ChassisSpeeds | Current speeds relative to robot |

### Drive Methods

```python
# Option 1: Translation2d + rotation (radians/sec)
swerve.drive(
    translation=Translation2d(vx, vy),  # m/s in x and y
    rotation=omega,                      # rad/s
    field_relative=True,                 # Field or robot relative
    open_loop=True                       # Open or closed loop
)

# Option 2: ChassisSpeeds
swerve.drive(
    speeds=ChassisSpeeds(vx, vy, omega),
    open_loop=True
)

# Set exact module states (for ski stop, etc.)
swerve.desire_module_states(states)  # Tuple of 4 SwerveModuleState
```

### Odometry Methods

```python
swerve.reset_odometry(Pose2d(x, y, rotation))  # Set current pose
swerve.zero_heading(degrees=0)                  # Reset gyro heading
swerve.reset_modules()                          # Reset drive encoders
swerve.reset_odometry_to_vision()               # Sync with vision pose
```

### Characterization

```python
# For SysId motor characterization
swerve.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
swerve.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
swerve.sys_id_dynamic(SysIdRoutine.Direction.kForward)
swerve.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
```

---

## Teleop Control

### Basic Setup

```python
# In container.py
from swervepy import SwerveDrive
import swerve_config

class RobotContainer:
    def __init__(self):
        self.driver = CommandXboxController(0)

        self.swerve = SwerveDrive(
            swerve_config.SWERVE_MODULES,
            swerve_config.GYRO,
            swerve_config.MAX_VELOCITY,
            swerve_config.MAX_ANGULAR_VELOCITY,
            swerve_config.AUTONOMOUS_PARAMS,
        )

        # Create teleop command
        self.teleop_command = self.swerve.teleop_command(
            translation=lambda: -self.driver.getLeftY(),   # Forward/back
            strafe=lambda: -self.driver.getLeftX(),        # Left/right
            rotation=lambda: -self.driver.getRightX(),     # Turn
            field_relative=True,
            drive_open_loop=True,
        )

        # Set as default command
        self.swerve.setDefaultCommand(self.teleop_command)
```

### Input Shaping (Recommended)

Add exponential curves for better control feel:

```python
def shape_input(value, exponent=2):
    """Apply exponential curve while preserving sign"""
    return (abs(value) ** exponent) * (1 if value > 0 else -1)

self.teleop_command = self.swerve.teleop_command(
    translation=lambda: shape_input(-self.driver.getLeftY(), 2),
    strafe=lambda: shape_input(-self.driver.getLeftX(), 2),
    rotation=lambda: shape_input(-self.driver.getRightX(), 2),
    field_relative=True,
    drive_open_loop=True,
)
```

### Ski Stop Command

Lock wheels in an X pattern to resist pushing:

```python
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

def SkiStopCommand(swerve: SwerveDrive):
    angles = (45, 315, 315, 45)  # FL, FR, RL, RR
    states = tuple(
        SwerveModuleState(0, Rotation2d.fromDegrees(angle))
        for angle in angles
    )
    return commands2.RunCommand(
        lambda: swerve.desire_module_states(states),
        swerve
    )
```

### Button Bindings

```python
def configure_button_bindings(self):
    # Reset gyro
    self.driver.start().onTrue(
        InstantCommand(lambda: self.swerve.zero_heading(180))
    )

    # Toggle field-relative
    self.driver.back().onTrue(
        InstantCommand(self.teleop_command.toggle_field_relative)
    )

    # Ski stop
    self.driver.y().whileTrue(
        SkiStopCommand(self.swerve)
    )
```

---

## Autonomous Integration

### PathPlanner Setup

swervepy integrates with PathPlanner for autonomous paths.

```python
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants

# Configure AutoBuilder in RobotContainer.__init__
AutoBuilder.configure(
    pose_supplier=lambda: self.swerve.pose,
    reset_pose=self.swerve.reset_odometry,
    robot_relative_speeds_supplier=lambda: self.swerve.robot_relative_speeds,
    drive_robot_relative=lambda speeds: self.swerve.drive(speeds, open_loop=False),
    HolonomicPathFollowerConfig(
        PIDConstants(5.0, 0, 0),      # Translation PID
        PIDConstants(5.0, 0, 0),      # Rotation PID
        swerve_config.MAX_VELOCITY.m_as(u.m / u.s),
        math.hypot(swerve_config.WHEEL_BASE/2, swerve_config.TRACK_WIDTH/2),
    ),
    should_flip_path=lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
    self.swerve,
)
```

### DriveToPose Command

Drive to a specific field position:

```python
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.trajectory import PathPlannerTrajectoryState

class DriveToPoseCommand(commands2.Command):
    def __init__(self, swerve, target: Pose2d, params: TrajectoryFollowerParameters):
        super().__init__()
        self.swerve = swerve
        self.target = PathPlannerTrajectoryState(pose=target)
        self.controller = PPHolonomicDriveController(
            PIDConstants(params.xy_kP),
            PIDConstants(params.theta_kP)
        )

    def initialize(self):
        self.controller.reset(self.swerve.pose, self.swerve.robot_relative_speeds)

    def execute(self):
        output = self.controller.calculateRobotRelativeSpeeds(
            self.swerve.pose, self.target
        )
        self.swerve.drive(output, open_loop=True)

    def end(self, interrupted):
        self.swerve.drive(ChassisSpeeds(), open_loop=True)

    def getRequirements(self):
        return {self.swerve}
```

---

## Tuning Guide

### 1. Encoder Offsets

1. Put robot on blocks
2. Manually rotate each wheel to point forward (toward front of robot)
3. Read CANCoder values in Phoenix Tuner
4. Use those values as offsets
5. Deploy and verify all wheels point same direction

### 2. Drive Motor PID (SysId Recommended)

Start with these values and adjust:

| Gain | Starting Value | Adjust If... |
|------|---------------|--------------|
| kP | 0.05-0.1 | Robot sluggish (increase) or oscillates (decrease) |
| kI | 0 | Usually not needed |
| kD | 0 | Add small amount if oscillating |
| kS | 0.1-0.3 | Find via SysId (static friction) |
| kV | 2.0-3.0 | Find via SysId (velocity feedforward) |
| kA | 0.2-0.5 | Find via SysId (acceleration feedforward) |

### 3. Azimuth Motor PID

| Gain | Starting Value | Adjust If... |
|------|---------------|--------------|
| kP | 0.01-0.05 | Wheels slow to turn (increase) or vibrate (decrease) |
| kI | 0 | Usually not needed |
| kD | 0 | Add if oscillating around target |

### 4. Autonomous PID

| Gain | Purpose | Starting Value |
|------|---------|---------------|
| xy_kP | Translation tracking | 2-5 |
| theta_kP | Rotation tracking | 2-5 |

Increase if robot drifts off path. Decrease if robot overshoots.

### 5. Running SysId

```python
# Add these button bindings for characterization
self.sysid_controller = CommandXboxController(3)

self.sysid_controller.y().whileTrue(
    self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
)
self.sysid_controller.a().whileTrue(
    self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
)
self.sysid_controller.b().whileTrue(
    self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kForward)
)
self.sysid_controller.x().whileTrue(
    self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
)
```

After running, analyze logs in SysId tool to get kS, kV, kA values.

---

## Troubleshooting

### Wheels Point Wrong Direction

- **Symptom**: Wheels don't all point the same way when driving forward
- **Cause**: Encoder offsets are wrong
- **Fix**: Re-calibrate encoder offsets (see Tuning Guide)

### Robot Drifts When Driving Straight

- **Symptom**: Robot curves when you push forward
- **Cause**: Gyro not calibrated or motor inversions wrong
- **Fix**:
  1. Let robot sit still for 10 seconds after power on (gyro calibration)
  2. Check `invert_motor` settings in drive params

### Robot Spins Uncontrollably

- **Symptom**: Robot spins when you try to drive
- **Cause**: Azimuth motors fighting each other
- **Fix**: Check azimuth `invert_motor` settings; some modules may need opposite direction

### Wheels Jitter/Vibrate

- **Symptom**: Wheels oscillate rapidly when stopped
- **Cause**: Azimuth kP too high
- **Fix**: Reduce azimuth kP (try halving it)

### Robot Sluggish in Auto

- **Symptom**: Robot doesn't follow paths accurately
- **Cause**: Auto PID gains too low
- **Fix**: Increase xy_kP and theta_kP in AUTONOMOUS_PARAMS

### Vision Pose Jumping

- **Symptom**: Robot pose jumps around on dashboard
- **Cause**: Vision measurement trust too high
- **Fix**: Filter vision updates by distance or stddev

### Motor Brownout

- **Symptom**: Robot loses power during aggressive maneuvers
- **Cause**: Current limits too high
- **Fix**: Reduce continuous_current_limit (try 40A instead of 60A)

---

## Quick Reference Card

### Imports

```python
from swervepy import SwerveDrive, TrajectoryFollowerParameters, u
from swervepy.impl import (
    TypicalDriveComponentParameters,
    TypicalAzimuthComponentParameters,
    NeutralMode,
    NEOCoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
    AbsoluteCANCoder,
    Pigeon2Gyro,
    CoaxialSwerveModule,
)
```

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

```python
# Standard 4-module layout
FL = Translation2d(+WHEEL_BASE/2, +TRACK_WIDTH/2)  # Front Left
FR = Translation2d(+WHEEL_BASE/2, -TRACK_WIDTH/2)  # Front Right
RL = Translation2d(-WHEEL_BASE/2, +TRACK_WIDTH/2)  # Rear Left
RR = Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2)  # Rear Right
```

### Units Cheat Sheet

```python
from swervepy import u

# Length
distance_m = (24 * u.inch).m_as(u.m)   # inches to meters
distance_in = (0.6 * u.m).m_as(u.inch) # meters to inches

# Velocity
speed = 4.2 * (u.m / u.s)              # meters per second
speed_fps = 14 * (u.ft / u.s)          # feet per second

# Angular velocity
omega = 360 * (u.deg / u.s)            # degrees per second
omega_rad = 6.28 * (u.rad / u.s)       # radians per second
```

---

## Additional Resources

- [swervepy GitHub Repository](https://github.com/EWall25/swervepy)
- [WPILib Swerve Documentation](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
- [PathPlanner Documentation](https://pathplanner.dev/home.html)
- [REV Robotics SparkMAX Documentation](https://docs.revrobotics.com/sparkmax/)
- [CTRE Phoenix Documentation](https://v6.docs.ctr-electronics.com/)

---

*Last Updated: November 2025*
*Team 3164 Stealth Tigers*
