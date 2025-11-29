# Reefscape 2025 Java Simulator

A web-based simulator for the Team 3164 Stealth Tigers robot code. Run it from the command line and control the robot through your browser.

## Overview

The simulator provides:
- **Field visualization** - Top-down view of the REEFSCAPE field with hexagonal reef
- **Robot simulation** - Swerve drive physics, elevator, arm, claw, and climber
- **Real-time controls** - Keyboard input via WebSocket at 50Hz
- **Subsystem dashboard** - Live status of all robot mechanisms
- **Score tracking** - Points awarded for scoring coral at different levels

## Requirements

- **Java 17+** (OpenJDK recommended)

### Installing Java on macOS

```bash
# Using Homebrew
brew install openjdk@17

# Add to your shell profile (~/.zshrc or ~/.bashrc)
export JAVA_HOME="/opt/homebrew/Cellar/openjdk@17/17.0.17"
export PATH="$JAVA_HOME/bin:$PATH"

# Reload your shell
source ~/.zshrc
```

### Installing Java on Windows

Download from [Adoptium](https://adoptium.net/) and add to PATH.

### Installing Java on Linux

```bash
sudo apt install openjdk-17-jdk  # Debian/Ubuntu
sudo dnf install java-17-openjdk  # Fedora
```

## Building the Simulator

```bash
cd simulator

# Build the fat JAR (includes all dependencies)
./gradlew shadowJar

# The JAR will be at: build/libs/reefscape-sim.jar
```

## Running the Simulator

### Basic Usage

```bash
cd simulator
java -jar build/libs/reefscape-sim.jar
```

This will:
1. Start the simulation server on port 8080
2. Automatically open your browser to `http://localhost:8080`
3. Begin the physics simulation at 50Hz

### Command Line Options

```bash
java -jar build/libs/reefscape-sim.jar [options]

Options:
  --port, -p <port>   Server port (default: 8080)
  --no-browser        Don't auto-open browser
  --help, -h          Show help
```

### Examples

```bash
# Run on a different port
java -jar build/libs/reefscape-sim.jar --port 3000

# Run without opening browser
java -jar build/libs/reefscape-sim.jar --no-browser

# If Java isn't in your PATH
export JAVA_HOME="/opt/homebrew/Cellar/openjdk@17/17.0.17"
export PATH="$JAVA_HOME/bin:$PATH"
java -jar build/libs/reefscape-sim.jar
```

## Controls

### Driving

| Key | Action |
|-----|--------|
| `W` | Drive forward |
| `S` | Drive backward |
| `A` | Strafe left |
| `D` | Strafe right |
| `Q` | Rotate counter-clockwise |
| `E` | Rotate clockwise |
| `X` | Toggle slow mode (30% speed) |
| `C` | Toggle field-relative mode |
| `G` | Reset gyro to 180° |
| `V` | Ski stop (lock wheels in X pattern) |

### Scoring

| Key | Action |
|-----|--------|
| `R` | Loading position (L0) |
| `1` | Level 1 scoring position |
| `2` | Level 2 scoring position |
| `3` | Level 3 scoring position |
| `4` | Level 4 scoring position |
| `Space` | Intake coral |
| `Shift` | Outtake / Score |

### Other

| Key | Action |
|-----|--------|
| `↑` | Climber up |
| `↓` | Climber down |
| `P` | Debug: Instantly pickup coral |
| `Esc` | Reset robot to starting position |

## Scoring Workflow

1. Press `P` to pick up a coral (or drive to station and press `Space`)
2. Press `1`, `2`, `3`, or `4` to set scoring level
3. Drive to the reef
4. Press `Shift` to score

**Point Values:**
- Level 1: 2 points
- Level 2: 3 points
- Level 3: 4 points
- Level 4: 6 points

## User Interface

### Field View
- Blue rectangle: FRC field (54' x 27')
- Cyan hexagon: REEF structure
- Gold circles: Scoring positions A-L
- Red/Green square: Robot (green = holding coral)
- Arrow: Robot heading direction

### Subsystem Panel
- **Elevator**: Height bar with goal indicator (red line)
- **Arm**: Rotating indicator showing current angle
- **Claw**: State indicator (Empty/Intaking/Holding/Outtaking)
- **Climber**: Position bar

### Mode Indicators
- **Field Relative**: Green when active (drive relative to field)
- **Slow Mode**: Green when active (30% speed)

## Architecture

```
simulator/
├── src/main/java/team3164/simulator/
│   ├── Main.java                 # Entry point, CLI handling
│   ├── Constants.java            # Robot/field constants
│   ├── engine/
│   │   ├── SimulationEngine.java # 50Hz physics loop
│   │   ├── RobotState.java       # Complete robot state
│   │   └── InputState.java       # Keyboard input state
│   ├── physics/
│   │   ├── SwervePhysics.java    # Holonomic drive simulation
│   │   ├── ElevatorPhysics.java  # Trapezoidal motion profile
│   │   ├── ArmPhysics.java       # Angular motion simulation
│   │   ├── ClawPhysics.java      # Intake state machine
│   │   └── ClimberPhysics.java   # Simple velocity control
│   └── web/
│       └── SimulatorServer.java  # HTTP + WebSocket server
└── src/main/resources/web/
    ├── index.html                # Main UI
    ├── style.css                 # Dark theme styling
    └── app.js                    # Canvas rendering, controls
```

## Physics Simulation

The simulator models real robot behavior:

### Swerve Drive
- 4-wheel holonomic motion with proper kinematics
- Module angle optimization (shortest path)
- Acceleration limits matching robot constants
- Field-relative transformation

### Elevator
- Trapezoidal motion profile (accelerate → cruise → decelerate)
- Max velocity: 150 in/s
- Max acceleration: 200 in/s²
- Soft limits: 30.5" to 78.5"

### Arm
- Angular motion with trapezoidal profile
- Angle limits: -50° to +90°
- Tolerance: 3°

### Claw
- State machine: EMPTY → INTAKING → HOLDING → OUTTAKING
- Intake time: 0.5s
- Outtake time: 0.3s

## Troubleshooting

### "Port already in use"
Another process is using port 8080. Either:
```bash
# Use a different port
java -jar build/libs/reefscape-sim.jar --port 3000

# Or kill the existing process
pkill -f reefscape-sim.jar
```

### "Unable to locate a Java Runtime"
Java isn't in your PATH. Set it explicitly:
```bash
export JAVA_HOME="/opt/homebrew/Cellar/openjdk@17/17.0.17"
export PATH="$JAVA_HOME/bin:$PATH"
```

### Browser doesn't open
Use `--no-browser` and manually navigate to `http://localhost:8080`

### Robot doesn't move
- Click on the browser window to ensure it has keyboard focus
- Check the WebSocket connection status (top right of UI)

## Development

### Rebuilding After Changes

```bash
./gradlew shadowJar
```

### Running Without Building JAR

```bash
./gradlew run
```

### Project Dependencies

- **Javalin 6.3.0** - HTTP server with WebSocket support
- **Gson 2.11.0** - JSON serialization
- **Jackson 2.17.2** - Object mapping for Javalin
- **SLF4J 2.0.13** - Logging

## Terminal Output

The simulator logs events to the terminal:

```
[00:00:00] Simulation started
[00:00:05] Set level 3 (height=45.3", angle=-35°)
[00:00:08] Field-relative: OFF
[00:00:12] SCORED! Level 3 (+4 pts, total: 4)
[00:00:15] Robot state reset
```

## Related Documentation

- [Java Code Overview](java_code_overview.md) - Understanding the robot code
- [Java How to Make Changes](java_how_to_make_changes.md) - Modifying robot code
- [WPILib Swerve Guide](wplibswerve_guide.md) - Swerve drive concepts
