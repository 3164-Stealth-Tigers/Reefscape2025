# Reefscape 2025 Java - Complete Codebase Documentation

## 🤖 What is This Project?

This is the robot code for Team 3164 Stealth Tigers' 2025 FRC (FIRST Robotics Competition) robot! The game this year is called **REEFSCAPE**, where robots compete to place CORAL pieces on a hexagonal REEF structure and perform other tasks. Think of it like a really advanced remote-controlled robot that can drive itself and score game pieces!

This is the **Java version** of the codebase, using WPILib's native swerve implementation.

## 📁 Project Structure Overview

```
src_java/
└── frc/
    └── robot/
        ├── Robot.java              # Main entry point
        ├── RobotContainer.java     # Command center
        ├── Constants.java          # All tunable values
        ├── Superstructure.java     # Coordinated movements
        ├── OI.java                 # Operator interface
        ├── subsystems/
        │   ├── swerve/
        │   │   ├── SwerveDrive.java
        │   │   └── SwerveModule.java
        │   ├── Elevator.java
        │   ├── CoralArm.java
        │   ├── Claw.java
        │   ├── Climber.java
        │   ├── AlgaeArm.java
        │   ├── AutoAlign.java
        │   └── Vision.java
        ├── commands/
        │   └── SwerveCommands.java
        └── util/
            └── FieldUtil.java
```

## 🎮 How the Robot Works - The Big Picture

### The Robot's Brain Structure

```
+------------------+     Commands     +---------------+
| Driver Station   | ----------------> | Robot.java   |
| Xbox Controllers |                   | (Main Brain) |
+------------------+                   +---------------+
                                              |
                                              v
                                    +-------------------+
                                    | RobotContainer    |
                                    | (Command Central) |
                                    +-------------------+
                                              |
                    +------------+------------+------------+
                    |            |            |            |
                    v            v            v            v
              +---------+  +---------+  +---------+  +---------+
              | Swerve  |  |Elevator |  |  Arm    |  |  Claw   |
              +---------+  +---------+  +---------+  +---------+
                    |            |            |            |
                    v            v            v            v
              +---------+  +---------+  +---------+  +---------+
              | Motors  |  | Motors  |  | Motor   |  | Motor   |
              +---------+  +---------+  +---------+  +---------+
```

## 📂 Detailed File Structure

### Core Files

#### 1. **Robot.java** - The Main Robot Brain
- **Location:** `src_java/frc/robot/Robot.java`
- **Purpose:** This is the starting point! It's like the robot's main brain that controls everything
- **What it does:**
  - Starts the robot when you turn it on
  - Switches between different modes (autonomous, teleop, test)
  - Manages data logging for debugging
  - Creates the RobotContainer that holds all subsystems

```java
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        DataLogManager.start();
        robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
}
```

#### 2. **RobotContainer.java** - Command Central
- **Location:** `src_java/frc/robot/RobotContainer.java`
- **Purpose:** This is where ALL the robot's parts come together!
- **What it does:**
  - Creates all subsystems (elevator, arms, drivetrain, etc.)
  - Maps controller buttons to robot actions
  - Builds autonomous routines
  - Manages the "superstructure" (coordinated movements)

### 🚗 Subsystems (Robot Parts)

Each subsystem is a separate class that represents a robot mechanism:

```
+-----------------------------------------------------------+
|                     MOVEMENT SYSTEMS                       |
+-----------------------------------------------------------+
|  SwerveDrive.java + SwerveModule.java                     |
|  - 4 wheel modules                                         |
|  - Can move any direction                                  |
|  - Uses WPILib native swerve (SwerveDriveKinematics)      |
+-----------------------------------------------------------+

+-----------------------------------------------------------+
|                     SCORING SYSTEMS                        |
+-----------------------------------------------------------+
|  Elevator.java          | CoralArm.java     | Claw.java   |
|  - Moves up/down        | - Rotates angle   | - Grabs     |
|  - 30-80 inches         | - Places pieces   | - Releases  |
|  - SparkMax motors      | - SparkFlex motor | - NEO 550   |
+-----------------------------------------------------------+

+-----------------------------------------------------------+
|                     SUPPORT SYSTEMS                        |
+-----------------------------------------------------------+
|  Climber.java   | Vision.java    | AutoAlign.java         |
|  - Lifts robot  | - Camera track | - Auto positioning     |
|  - End game     | - AprilTags    | - Collision detection  |
+-----------------------------------------------------------+
```

### Detailed Subsystem Breakdown

#### 1. **Swerve Drive System** (`subsystems/swerve/`)

The coolest part! This robot can move in ANY direction without turning!

```
+------------------------------------------+
|           SWERVE MODULE LAYOUT           |
+------------------------------------------+
|                                          |
|   [FL]-------------------------[FR]      |
|   Motors: 7,8                Motors: 5,6 |
|   CANCoder: 1                CANCoder: 2 |
|                                          |
|                                          |
|                                          |
|   [RL]-------------------------[RR]      |
|   Motors: 3,4                Motors: 1,2 |
|   CANCoder: 3                CANCoder: 4 |
|                                          |
+------------------------------------------+

Each Module Contains:
+------------------+
| Drive Motor      | <-- Moves wheel forward/back
+------------------+
| Azimuth Motor    | <-- Rotates wheel direction
+------------------+
| CANCoder         | <-- Tracks absolute position
+------------------+
```

**Key Classes:**
- `SwerveDrive.java` - Main subsystem using `SwerveDriveKinematics` and `SwerveDrivePoseEstimator`
- `SwerveModule.java` - Individual module control with SparkMax motors

**Key Specifications:**
- Max Speed: 4.2 meters/second
- Wheel Base: 29.75 inches
- Track Width: 17.75 inches
- Each wheel can rotate 360° independently!

#### 2. **Elevator System** (`subsystems/Elevator.java`)

Lifts the scoring mechanism up and down:

```
Height Levels:
                                           78.5" ─┬─ Level 4
                                                  │
                                                  │
                                           45.25" ─┬─ Level 3
                                                  │
                                           29.75" ─┬─ Level 2
                                           31.0"  ─┼─ Level 1
                                           30.5"  ─┴─ Level 0 (Loading)
```

**Features:**
- Two NEO motors (IDs 9, 10) in leader/follower configuration
- Soft limits prevent overextension
- Hall effect sensor for homing
- Range: 30.5" to 78.5" height
- Trapezoidal motion profiling

#### 3. **Coral Arm** (`subsystems/CoralArm.java`)

Rotates to place game pieces at different angles:

```
Arm Positions:
    90°  ╭─────
         │      Extended (for intake)
         │
    0°  ─────── Horizontal
         │
   -35° ─────╮  Scoring angle (L1-L4)
              ╰
   -50°        Minimum limit
```

**Specifications:**
- Motor ID: 12 (NEO Vortex)
- Gear Ratio: 125:1
- Arm Length: 18 inches
- Uses absolute encoder for position

#### 4. **Claw System** (`subsystems/Claw.java`)

Grabs and releases CORAL game pieces:

```
State Machine:
+--------+    Intake    +----------+    Current>15A    +---------------+
|  IDLE  | -----------> | INTAKING | ----------------> | HAS_POSSESSION|
+--------+              +----------+                   +---------------+
    ^                                                          |
    |                                                          |
    +------------------------- Outtake ------------------------+
```

**Detection Method:**
- Monitors motor current
- High current + low speed = piece grabbed!
- Threshold: 15 Amps

#### 5. **Climber** (`subsystems/Climber.java`)

End-game climbing mechanism:
- Two SparkFlex motors (IDs: 13, 14)
- Synchronized movement in leader/follower
- Soft limits for safety

### 🎮 Control System

#### Controller Mapping

```
+--------------------------------------------------+
|             DRIVER CONTROLLER (Port 0)            |
+--------------------------------------------------+
| Left Stick    | Forward/Strafe movement          |
| Right Stick   | Rotation                         |
| Start Button  | Reset gyro to 180°               |
| Back Button   | Toggle field-relative            |
| X Button      | Toggle speed (normal/slow)       |
| Y Button      | Ski stop (X pattern)             |
+--------------------------------------------------+

+--------------------------------------------------+
|            OPERATOR CONTROLLER (Port 1)           |
+--------------------------------------------------+
| A Button      | Level 1                           |
| X Button      | Level 2                           |
| B Button      | Level 3                           |
| Y Button      | Level 4                           |
| Right Trigger | Loading position + intake         |
| Left Trigger  | Outtake                           |
| Left Bumper   | Outtake                           |
| POV Up/Down   | Climber up/down                   |
| Start Button  | Home elevator                     |
| Back Button   | Toggle automation                 |
+--------------------------------------------------+

+--------------------------------------------------+
|             BUTTON BOARD (Port 2)                 |
+--------------------------------------------------+
| Buttons 1-12  | Reef positions A-L               |
| Stick Left    | Left coral station               |
| Stick Right   | Right coral station              |
+--------------------------------------------------+
```

#### OI Architecture (OI.java)

The OI (Operator Interface) uses interfaces and implementations:

```java
// Interface defines what controls are needed
public interface DriverActionSet {
    double forward();
    double strafe();
    double turn();
    Trigger resetGyro();
    Trigger skiStop();
    // ...
}

// Implementation for Xbox controller
public class XboxDriver implements DriverActionSet {
    private final CommandXboxController stick;

    @Override
    public double forward() {
        return deadband(-stick.getLeftY(), 0.08);
    }
    // ...
}
```

### 🤖 Autonomous Routines

The robot can run pre-programmed routines:

```
Autonomous Selection:
+-------------------+
|  Drive Forward    | <-- Simple 2-second drive
+-------------------+
        |
        v
+-------------------+
|      RP Auto      | <-- Score 2 pieces for ranking point
+-------------------+
        |
        v
+-------------------+
|     Speed 1       | <-- Complex 3-4 piece auto (top reef)
+-------------------+
        |
        v
+-------------------+
|     Speed 2       | <-- Alternative route (bottom reef)
+-------------------+
```

### 📊 Constants and Configuration

All constants are in `Constants.java` with nested classes:

```java
public final class Constants {
    public static final class ElevatorConstants {
        public static final int LEFT_MOTOR_ID = 9;
        public static final double MAX_VELOCITY = 3.81;  // m/s
        // ...
    }

    public static final class SwerveConstants {
        public static final double MAX_SPEED = 4.2;  // m/s
        public static final double FL_ENCODER_OFFSET = 199.072266;
        // ...
    }

    public static final class ScoringConstants {
        public static final double L4_HEIGHT = 1.9939;  // meters
        // ...
    }
}
```

### 🔧 Key WPILib Classes Used

| Class | Purpose |
|-------|---------|
| `SubsystemBase` | Base class for all subsystems |
| `Command` | Base class for all commands |
| `Commands` | Factory for common command patterns |
| `Trigger` | Button binding class |
| `SwerveDriveKinematics` | Swerve math calculations |
| `SwerveDrivePoseEstimator` | Position tracking with vision |
| `SparkMax` / `SparkFlex` | REV motor controllers |

### 💡 How to Make Changes

#### Adding a New Button Function

1. Open `OI.java`
2. Add method to interface
3. Implement in controller class
4. Open `RobotContainer.java`
5. Add binding in `configureButtonBindings()`

```java
// In OI.java interface
Trigger newButton();

// In OI.java XboxDriver class
@Override
public Trigger newButton() {
    return stick.b();
}

// In RobotContainer.java
driverJoystick.newButton().onTrue(
    Commands.runOnce(() -> System.out.println("Button pressed!"))
);
```

#### Adjusting Robot Speeds

1. Open `Constants.java`
2. Find the relevant constant
3. Change the value
4. Rebuild and deploy

```java
// Before
public static final double MAX_SPEED = 4.2;

// After
public static final double MAX_SPEED = 3.0;  // Slower for testing
```

#### Modifying Autonomous

1. Open `RobotContainer.java`
2. Create new auto method
3. Add to SendableChooser

```java
private void buildMyAuto() {
    Command auto = Commands.sequence(
        elevator.setHeightCommand(ScoringConstants.L4_HEIGHT),
        claw.outtakeCommand().withTimeout(1.0),
        elevator.setHeightCommand(ScoringConstants.LOADING_HEIGHT)
    );
    autoChooser.addOption("My Auto", auto);
}
```

### 🎯 Key Concepts for Beginners

#### Commands vs Subsystems

- **Subsystem**: A physical part of the robot (like the elevator)
- **Command**: An action that uses subsystems (like "move elevator to level 2")

```
+----------------------------+        +-------------------+
| Command: SetHeightCommand  | --Uses--> | Subsystem: Elevator |
+----------------------------+        +-------------------+
                                              |
                                         Controls
                                              v
                                      +---------------+
                                      | Hardware: Motors |
                                      +---------------+
```

#### Command Composition

Commands can be combined:

```java
// Sequential - one after another
Command sequence = Commands.sequence(
    driveForward,
    raiseElevator,
    score
);

// Parallel - all at once
Command parallel = Commands.parallel(
    driveToTarget,
    prepareElevator,
    spinUpShooter
);

// Race - until one finishes
Command race = Commands.race(
    intake,
    Commands.waitSeconds(3.0)  // Timeout
);

// Deadline - until deadline finishes
Command deadline = Commands.deadline(
    driveToTarget,    // When this finishes, stop the others
    runIntake,
    blinkLights
);
```

### 🐛 Debugging Tips

1. **SmartDashboard**: Shows live data from robot
   ```java
   SmartDashboard.putNumber("Elevator/Height", getHeight());
   SmartDashboard.putBoolean("Claw/HasPiece", hasPossession());
   ```

2. **DataLogManager**: Records all NT values
   ```java
   DataLogManager.start();
   ```

3. **Print Commands**: Quick debugging
   ```java
   Commands.print("Reached this point!")
   ```

4. **Shuffleboard/AdvantageScope**: Advanced visualization

### 📈 Performance Metrics

- **Loop Time**: 20ms (50Hz)
- **CAN Bus Usage**: Monitor in Driver Station
- **Battery Voltage**: Keep above 11V
- **Motor Temperatures**: Watch for overheating

## 🚀 Getting Started with Code Changes

### Basic Workflow

1. Make changes in your editor (VS Code recommended)
2. Build: `./gradlew build`
3. Deploy to robot: `./gradlew deploy`
4. Test in disabled mode first
5. Enable and test carefully
6. Check SmartDashboard for errors

### Safety First!

- Always have someone ready to disable robot
- Test new code with robot on blocks
- Start with low speeds
- Check motor directions before full test

## 📚 Where to Learn More

- **WPILib Docs**: https://docs.wpilib.org
- **REV Robotics Docs**: https://docs.revrobotics.com
- **PathPlanner Docs**: https://pathplanner.dev
- **Team Documentation**: This folder!
- **Code Comments**: Read the Javadoc documentation

---

*Remember: Every expert programmer started as a beginner. Don't be afraid to ask questions and experiment (safely)!*
