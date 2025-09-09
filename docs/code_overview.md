# Reefscape 2025 - Complete Codebase Documentation

## 🤖 What is This Project?

This is the robot code for Team 3164 Stealth Tigers' 2025 FRC (FIRST Robotics Competition) robot! The game this year is called **REEFSCAPE**, where robots compete to place CORAL pieces on a hexagonal REEF structure and perform other tasks. Think of it like a really advanced remote-controlled robot that can drive itself and score game pieces!

## 📁 Project Structure Overview

```mermaid
graph TD
    A[Reefscape2025 Root] --> B[src/]
    A --> C[docs/]
    A --> D[downloads/]
    
    B --> E[robot.py<br/>Main Robot Code]
    B --> F[container.py<br/>Command Center]
    B --> G[subsystems/]
    B --> H[commands/]
    B --> I[Configuration Files]
    
    G --> J[Drive System]
    G --> K[Scoring Mechanisms]
    G --> L[Support Systems]
    
    style A fill:#f9f,stroke:#333,stroke-width:4px
    style E fill:#bbf,stroke:#333,stroke-width:2px
    style F fill:#bbf,stroke:#333,stroke-width:2px
```

## 🎮 How the Robot Works - The Big Picture

### The Robot's Brain Structure

```mermaid
graph LR
    A[Driver Station<br/>Xbox Controllers] -->|Commands| B[Robot Brain<br/>robot.py]
    B --> C[Container<br/>Command Central]
    C --> D[Subsystems]
    D --> E[Motors & Sensors]
    E -->|Feedback| B
    
    style A fill:#fbb,stroke:#333,stroke-width:2px
    style B fill:#bbf,stroke:#333,stroke-width:2px
    style C fill:#bfb,stroke:#333,stroke-width:2px
```

## 📂 Detailed File Structure

### Core Files

#### 1. **robot.py** - The Main Robot Brain
- **Location:** `src/robot.py`
- **Purpose:** This is the starting point! It's like the robot's main brain that controls everything
- **What it does:**
  - Starts the robot when you turn it on
  - Switches between different modes (autonomous, teleop, test)
  - Manages data logging for debugging
  - Creates the RobotContainer that holds all subsystems

```mermaid
sequenceDiagram
    participant DS as Driver Station
    participant R as robot.py
    participant RC as RobotContainer
    participant S as Subsystems
    
    DS->>R: Power On
    R->>RC: Create Container
    RC->>S: Initialize Subsystems
    R->>DS: Ready!
    
    loop Every 20ms
        DS->>R: Send Commands
        R->>RC: Process Commands
        RC->>S: Execute Actions
        S->>R: Return Status
    end
```

#### 2. **container.py** - Command Central
- **Location:** `src/container.py`
- **Purpose:** This is where ALL the robot's parts come together!
- **What it does:**
  - Creates all subsystems (elevator, arms, drivetrain, etc.)
  - Maps controller buttons to robot actions
  - Builds autonomous routines
  - Manages the "superstructure" (coordinated movements)

### 🚗 Subsystems (Robot Parts)

Each subsystem is like a different part of your robot's body:

```mermaid
graph TB
    subgraph "Movement Systems"
        A[Swerve Drive<br/>4 wheel modules<br/>Can move any direction]
    end
    
    subgraph "Scoring Systems"
        B[Elevator<br/>Moves up/down<br/>30-80 inches]
        C[Coral Arm<br/>Rotates to angle<br/>Places game pieces]
        D[Claw<br/>Grabs/releases<br/>CORAL pieces]
    end
    
    subgraph "Support Systems"
        E[Climber<br/>Lifts robot<br/>End game]
        F[Vision<br/>Camera tracking<br/>April Tags]
        G[Auto Align<br/>Automatic positioning<br/>For scoring]
    end
    
    style A fill:#ffd,stroke:#333,stroke-width:2px
    style B fill:#dff,stroke:#333,stroke-width:2px
    style C fill:#dff,stroke:#333,stroke-width:2px
    style D fill:#dff,stroke:#333,stroke-width:2px
```

### Detailed Subsystem Breakdown

#### 1. **Swerve Drive System** (`swerve_config.py` + SwerveDrive library)

The coolest part! This robot can move in ANY direction without turning!

```mermaid
graph TD
    subgraph "Swerve Module Layout"
        FL[Front Left<br/>Motor IDs: 7,8<br/>CANCoder: 1]
        FR[Front Right<br/>Motor IDs: 5,6<br/>CANCoder: 2]
        RL[Rear Left<br/>Motor IDs: 3,4<br/>CANCoder: 3]
        RR[Rear Right<br/>Motor IDs: 1,2<br/>CANCoder: 4]
    end
    
    subgraph "Each Module Has"
        D[Drive Motor<br/>Moves wheel forward/back]
        A[Azimuth Motor<br/>Rotates wheel direction]
        E[Encoder<br/>Tracks position]
    end
```

**Key Specifications:**
- Max Speed: 4.2 meters/second
- Wheel Base: 29.75 inches
- Track Width: 17.75 inches
- Each wheel can rotate 360° independently!

#### 2. **Elevator System** (`subsystems/elevator.py`)

Lifts the scoring mechanism up and down:

```mermaid
graph LR
    A[Controller Input] --> B[SetHeightCommand]
    B --> C[Elevator Motors<br/>IDs: 9, 10]
    C --> D[Carriage Movement]
    D --> E[Height Levels]
    
    E --> F[Level 0: 30.5"]
    E --> G[Level 1: 31"]
    E --> H[Level 2: 29.75"]
    E --> I[Level 3: 45.25"]
    E --> J[Level 4: 78.5"]
```

**Features:**
- Two NEO motors working together
- Soft limits prevent overextension
- Hall effect sensor for homing
- Range: 30.5" to 80" height

#### 3. **Coral Arm** (`subsystems/coral_arm.py`)

Rotates to place game pieces at different angles:

```mermaid
graph TD
    A[Arm Positions] --> B[Level 0: 0°]
    A --> C[Level 1-4: -35°]
    A --> D[Extended: 90°]
    
    E[Motor ID: 12] --> F[125:1 Gear Ratio]
    F --> G[14 inch arm length]
```

#### 4. **Claw System** (`subsystems/claw.py`)

Grabs and releases CORAL game pieces:

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Intaking: Intake Button
    Intaking --> HasPossession: Current > 15A
    HasPossession --> Outtaking: At Scoring Position
    Outtaking --> Idle: Piece Released
```

**Detection Method:**
- Monitors motor current
- High current + low speed = piece grabbed!
- Threshold: 15 Amps

#### 5. **Climber** (`subsystems/climber.py`)

End-game climbing mechanism:

```mermaid
graph LR
    A[Two Motors<br/>IDs: 13, 14] --> B[Synchronized Movement]
    B --> C[Rotation Limits<br/>76° to 153°]
```

### 🎮 Control System

#### Controller Mapping

```mermaid
graph TD
    subgraph "Driver Controller (Port 0)"
        D1[Left Stick: Forward/Strafe]
        D2[Right Stick: Rotation]
        D3[Back: Reset Gyro]
        D4[Start: Toggle Speed]
        D5[X: Ski Stop]
    end
    
    subgraph "Operator Controller (Port 1)"
        O1[A: Level 1]
        O2[X: Level 2]
        O3[B: Level 3]
        O4[Y: Level 4]
        O5[RT: Intake]
        O6[LT: Outtake]
        O7[RB: Loading Position]
    end
    
    subgraph "Button Board (Port 2)"
        B1[12 Reef Positions<br/>Letters A-L]
        B2[2 Loading Stations<br/>Left & Right]
    end
```

### 🤖 Autonomous Routines

The robot can run pre-programmed routines:

```mermaid
graph TD
    A[Autonomous Start] --> B{Which Auto?}
    B --> C[Drive Forward<br/>Simple 2 second drive]
    B --> D[RP Auto<br/>Score 2 pieces]
    B --> E[Speed 1<br/>Complex 3-4 piece auto]
    B --> F[Speed 2<br/>Alternative route]
    
    E --> G[Score at position I]
    G --> H[Get CORAL from station]
    H --> I[Score at position K]
    I --> J[Repeat...]
```

### 📊 Constants and Configuration

#### Important Robot Measurements

```mermaid
graph LR
    subgraph "Robot Dimensions"
        A[Length: 41.5 inches]
        B[Width: 29.5 inches]
        C[Max Height: 80 inches]
    end
    
    subgraph "Game Field"
        D[Field: 17.5m x 8m]
        E[Reef Diameter: 1.66m]
        F[12 Scoring Positions]
    end
```

### 🔧 Configuration Files

1. **constants.py** - All the important numbers
   - Motor IDs
   - Physical measurements
   - Speed limits
   - PID values

2. **swerve_config.py** - Drive system setup
   - Module positions
   - Encoder offsets
   - Speed parameters

3. **field.py** - Game field information
   - Scoring positions
   - Alliance flipping
   - Field dimensions

### 💡 How to Make Changes

#### Adding a New Button Function

1. Open `oi.py`
2. Add button definition
3. Open `container.py`
4. Map button to command in `configure_button_bindings()`

Example:
```python
# In oi.py
self.new_button = self.controller.a()

# In container.py
self.operator_joystick.new_button.onTrue(
    commands2.InstantCommand(lambda: print("Button pressed!"))
)
```

#### Adjusting Robot Speeds

1. Open `constants.py`
2. Find the relevant constant (e.g., `MAX_VELOCITY`)
3. Change the value
4. Redeploy code to robot

#### Modifying Autonomous

1. Open `container.py`
2. Find `build_autos_speed1()` or create new method
3. Use command groups to sequence actions
4. Add to auto chooser

### 🎯 Key Concepts for Beginners

#### Commands vs Subsystems

- **Subsystem**: A physical part of the robot (like the elevator)
- **Command**: An action that uses subsystems (like "move elevator to level 2")

```mermaid
graph LR
    A[Command: SetHeightCommand] -->|Uses| B[Subsystem: Elevator]
    B -->|Controls| C[Hardware: Motors]
```

#### Command Groups

Commands can be combined:
- **Sequential**: Do one thing, then another
- **Parallel**: Do multiple things at once
- **Race**: Do things until one finishes

```mermaid
graph TD
    A[ParallelCommandGroup] --> B[Drive Forward]
    A --> C[Raise Elevator]
    A --> D[Extend Arm]
    
    E[SequentialCommandGroup] --> F[First: Drive]
    F --> G[Then: Score]
    G --> H[Finally: Back up]
```

### 🐛 Debugging Tips

1. **SmartDashboard**: Shows live data from robot
2. **Data Logs**: Recorded in robot.py
3. **Simulation**: Test without physical robot
4. **Print Statements**: Use `commands2.PrintCommand()`

### 📈 Performance Metrics

- **Loop Time**: 20ms (50Hz)
- **CAN Bus Usage**: Monitor in Driver Station
- **Battery Voltage**: Keep above 11V
- **Motor Temperatures**: Watch for overheating

## 🚀 Getting Started with Code Changes

### Basic Workflow

1. Make changes in your editor
2. Deploy to robot (when connected)
3. Test in disabled mode first
4. Enable and test carefully
5. Check SmartDashboard for errors

### Safety First!

- Always have someone ready to disable robot
- Test new code with robot on blocks
- Start with low speeds
- Check motor directions before full test

## 📚 Where to Learn More

- **WPILib Docs**: Official FRC programming guide
- **Team Documentation**: This folder!
- **Code Comments**: Read the inline documentation
- **Ask Mentors**: They're here to help!

---

*Remember: Every expert programmer started as a beginner. Don't be afraid to ask questions and experiment (safely)!*