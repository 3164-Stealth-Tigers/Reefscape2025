# 🛠️ How to Make Changes (Java) - A Beginner's Guide

## 🎯 Quick Start - Your First Change

### The 5-Step Process

```
+------------+    +------------+    +------------+    +------------+    +------------+
| 1. UNDER-  |    | 2. FIND    |    | 3. MAKE    |    | 4. TEST    |    | 5. DEPLOY  |
|   STAND    | -> |   WHERE    | -> |   THE      | -> |   YOUR     | -> |   TO       |
|            |    |            |    |   CHANGE   |    |   CHANGE   |    |   ROBOT    |
+------------+    +------------+    +------------+    +------------+    +------------+
```

## 📚 Common Changes You'll Make

### 1. Changing Robot Speeds

**Scenario:** "The robot is moving too fast/slow"

**Decision Tree:**
```
What Speed to Change?
       |
   +---+---+---+
   |       |   |
   v       v   v
Drive   Turn  Elevator
Speed   Speed  Speed
   |       |      |
   v       v      v
Constants.java > SwerveConstants.MAX_SPEED
Constants.java > SwerveConstants.MAX_ANGULAR_VELOCITY
Constants.java > ElevatorConstants.MAX_VELOCITY
```

#### Example: Slowing Down Drive Speed

**File:** `src_java/frc/robot/Constants.java`

```java
public static final class SwerveConstants {
    // BEFORE - Too fast!
    public static final double MAX_SPEED = 4.2;  // meters per second

    // AFTER - More controllable
    public static final double MAX_SPEED = 3.0;  // meters per second
}
```

**Unit Conversions:**
```java
// Use WPILib Units class for conversions
import edu.wpi.first.math.util.Units;

double meters = Units.inchesToMeters(78.5);    // inches -> meters
double inches = Units.metersToInches(2.0);     // meters -> inches
double radians = Units.degreesToRadians(90);   // degrees -> radians
```

---

### 2. Adjusting Scoring Heights

**Scenario:** "The elevator isn't going to the right height for scoring"

**Process:**
```
1. Open Constants.java
       |
       v
2. Find ScoringConstants class
       |
       v
3. Locate L1_HEIGHT, L2_HEIGHT, etc.
       |
       v
4. Measure actual height needed
       |
       v
5. Update value (in METERS!)
       |
       v
6. Test change
```

#### Example: Adjusting Level 2 Height

**File:** `src_java/frc/robot/Constants.java`

```java
public static final class ScoringConstants {
    // BEFORE - Too high
    public static final double L2_HEIGHT = Units.inchesToMeters(31.25);

    // AFTER - Just right!
    public static final double L2_HEIGHT = Units.inchesToMeters(29.75);
}
```

**Pro Tip:** Measure the actual height needed with a tape measure first!

---

### 3. Adding a New Button Function

**Scenario:** "I want button B to do something new"

**Process:**
```
Step 1: Add to interface (OI.java)
            |
            v
Step 2: Implement in controller class (OI.java)
            |
            v
Step 3: Map to command (RobotContainer.java)
            |
            v
Step 4: Test button
```

#### Step-by-Step Example: Make B Button Do Something

**Step 1 & 2:** Add/Check in `src_java/frc/robot/OI.java`

```java
// In the interface
public interface OperatorActionSet {
    Trigger level3();  // B button already defined
    Trigger newAction();  // Add new method if needed
}

// In the implementation
public class XboxOperator implements OperatorActionSet {
    private final CommandXboxController stick;

    @Override
    public Trigger level3() {
        return stick.b();  // B button
    }

    @Override
    public Trigger newAction() {
        return stick.back();  // Or whatever button
    }
}
```

**Step 3:** Map it in `src_java/frc/robot/RobotContainer.java`

```java
private void configureButtonBindings() {
    // Simple print command
    operatorJoystick.level3().onTrue(
        Commands.print("B Button pressed!")
    );

    // Command that does something
    operatorJoystick.level3().onTrue(
        coralArm.setAngleCommand(Rotation2d.fromDegrees(45))
    );

    // Combined commands
    operatorJoystick.level3().onTrue(
        Commands.parallel(
            Commands.print("Dancing!"),
            coralArm.setAngleCommand(Rotation2d.fromDegrees(45))
        )
    );
}
```

---

### 4. Modifying Autonomous Routines

**Scenario:** "Change the auto to score in different positions"

**Process:**
```
1. Open RobotContainer.java
        |
        v
2. Find build*Auto() methods
        |
        v
3. Identify routine to change
        |
        v
4. Modify command sequence
        |
        v
5. TEST IN SIMULATION FIRST!
        |
        v
6. Test on real robot
```

#### Example: Changing Scoring Position in Auto

**File:** `src_java/frc/robot/RobotContainer.java`

```java
private void buildMyAuto() {
    Command auto = Commands.sequence(
        // BEFORE - Scores at position "I"
        // new AutoAlign.DriveToScoringPositionCommand(autoAlign, "I")

        // AFTER - Scores at position "K" instead
        new AutoAlign.DriveToScoringPositionCommand(autoAlign, "K"),

        claw.outtakeCommand().withTimeout(1.5),
        backupOffReef()
    );

    autoChooser.addOption("My Custom Auto", auto);
}
```

**Reef Positions Map:**
```
        K ---- L
       /        \
      /          \
     J            A
     |            |
     |    REEF    |
     |   CENTER   |
     I            B
      \          /
       \        /
        H ---- C
       /        \
      G          D
       \        /
        F ---- E

Blue side positions: A, C, E, G, I, K (left pipe)
Orange side positions: B, D, F, H, J, L (right pipe)
```

---

### 5. Adjusting Motor Power

**Scenario:** "The claw motor needs more/less power"

#### Example: Changing Claw Intake Speed

**File:** `src_java/frc/robot/subsystems/Claw.java`

```java
public void intake() {
    // BEFORE - Too slow
    motor.set(0.2);  // 20% power

    // AFTER - Faster intake
    motor.set(0.35);  // 35% power
}
```

**Power Scale:**
```
+1.0  ████████████████████  Full forward (100%)
+0.5  ██████████            Half forward (50%)
 0.0  ----------            Stopped
-0.5  ██████████            Half reverse
-1.0  ████████████████████  Full reverse (100%)
```

---

## 🔍 Finding What to Change

### Quick File Reference:

| If you want to change... | Look in this file... | For this... |
|-------------------------|---------------------|-------------|
| Drive speed | `Constants.java` | `SwerveConstants.MAX_SPEED` |
| Elevator heights | `Constants.java` | `ScoringConstants.L*_HEIGHT` |
| Button mappings | `RobotContainer.java` | `configureButtonBindings()` |
| Motor IDs | `Constants.java` | Various `*Constants` classes |
| Auto routines | `RobotContainer.java` | `build*Auto()` methods |
| Claw behavior | `Claw.java` | `intake()`, `outtake()` |
| Arm angles | `Constants.java` | `ScoringConstants.*_ANGLE` |
| Swerve modules | `Constants.java` | `SwerveConstants` |
| PID tuning | `Constants.java` | `*_kP`, `*_kD` values |

### Package Structure:
```
frc/robot/
├── Constants.java          # All numbers/settings
├── RobotContainer.java     # Button bindings, autos
├── Robot.java              # Main robot class
├── OI.java                 # Controller definitions
├── Superstructure.java     # Coordinated movements
├── subsystems/
│   ├── swerve/
│   │   ├── SwerveDrive.java
│   │   └── SwerveModule.java
│   ├── Elevator.java
│   ├── CoralArm.java
│   ├── Claw.java
│   ├── Climber.java
│   └── ...
├── commands/
│   └── SwerveCommands.java
└── util/
    └── FieldUtil.java
```

---

## ⚠️ Safety Checklist

### Before Making Changes:

```
SAFETY CHECKLIST:
[ ] Understand what the code does
[ ] Know the normal/safe values
[ ] Robot on blocks (wheels off ground)
[ ] Someone at E-stop
[ ] Test small changes first
[ ] Save a backup (git commit)
```

### Red Flags - Don't Change These Without Help:

1. **Motor Inversion** (`inverted(true/false)`)
   - Wrong = robot drives backward!

2. **Gear Ratios** (`GEAR_RATIO`)
   - Wrong = incorrect speeds/positions

3. **CAN IDs** (Motor/Sensor IDs)
   - Wrong = motors won't work

4. **PID Values** (`kP`, `kI`, `kD`)
   - Wrong = oscillation or instability

5. **Soft Limits** (`forwardSoftLimit`, `reverseSoftLimit`)
   - Wrong = mechanism crashes into itself

---

## 🧪 Testing Your Changes

### The Testing Pyramid:

```
                    +------------------+
                   /  6. Full Speed    \   <- MOST DANGEROUS
                  /       Test          \
                 +----------------------+
                /  5. Test on Ground     \
               /       SLOWLY             \
              +--------------------------+
             /  4. Test on Blocks         \
            +------------------------------+
           /  3. Test Disabled on Robot     \
          +----------------------------------+
         /  2. Test in Simulation             \
        +--------------------------------------+
       /  1. Check Code Compiles               \  <- SAFEST
      +------------------------------------------+
```

### How to Test Safely:

#### 1. **Build the Code** (No robot needed!)
```bash
# In terminal, from project root
./gradlew build

# Or in VS Code: Ctrl+Shift+P -> "WPILib: Build Robot Code"
```

#### 2. **Simulation Testing**
```bash
# In terminal
./gradlew simulateJava

# Or in VS Code: Ctrl+Shift+P -> "WPILib: Simulate Robot Code"
```

#### 3. **On-Blocks Testing**
- Put robot on blocks (wheels off ground)
- Deploy code
- Enable in teleop
- Test at low speeds first

#### 4. **SmartDashboard/Shuffleboard Monitoring**
Check these values while testing:
- Motor temperatures
- Current draw
- Battery voltage
- Error messages

---

## 💻 Deploying Code to Robot

### Step-by-Step Deployment:

```
1. Save all files
       |
       v
2. Connect to robot WiFi (team number: 3164)
       |
       v
3. Build: ./gradlew build
       |
       v
4. Deploy: ./gradlew deploy
       |
       v
5. Watch for errors in terminal
       |
       v
6. Open Driver Station, check code status
       |
       v
7. Enable robot CAREFULLY!
```

### VS Code Shortcuts:
- Build: `Ctrl+Shift+P` -> "WPILib: Build Robot Code"
- Deploy: `Ctrl+Shift+P` -> "WPILib: Deploy Robot Code"

### Common Deployment Issues:

| Problem | Solution |
|---------|----------|
| Can't connect to robot | Check WiFi - should be "3164" network |
| Build fails | Check terminal for error messages |
| Robot disabled itself | Check Driver Station for errors |
| Changes don't appear | Did you save and redeploy? |
| "No robot code" | Check if code deployed successfully |

---

## 📝 Code Patterns to Copy

### Pattern 1: Creating a Simple Command

```java
// Instant command (runs once)
Command myCommand = Commands.runOnce(() -> {
    subsystem.doSomething();
}, subsystem);

// Run command (runs continuously)
Command myRunCommand = Commands.run(() -> {
    subsystem.doSomethingRepeatedly();
}, subsystem);
```

### Pattern 2: Sequential Actions

```java
// Do multiple things in order
Command sequence = Commands.sequence(
    firstAction(),           // Do this first
    Commands.waitSeconds(1), // Wait 1 second
    secondAction()           // Then do this
);
```

### Pattern 3: Parallel Actions

```java
// Do multiple things at once
Command parallel = Commands.parallel(
    elevator.setHeightCommand(1.5),  // These happen
    coralArm.setAngleCommand(angle), // at the same
    claw.intakeCommand()             // time!
);
```

### Pattern 4: Conditional Commands

```java
// Do something based on a condition
Command conditional = Commands.either(
    doIfTrue(),              // If condition is true
    doIfFalse(),             // If condition is false
    () -> checkSomething()   // The condition to check
);
```

### Pattern 5: Button Bindings

```java
// When pressed (once)
button.onTrue(command);

// While held
button.whileTrue(command);

// Toggle on/off
button.toggleOnTrue(command);

// When released
button.onFalse(command);
```

### Pattern 6: Command with Timeout

```java
Command withTimeout = myCommand.withTimeout(2.0);  // Stops after 2 seconds
```

### Pattern 7: Command with End Condition

```java
Command untilFinished = Commands.run(() -> elevator.setHeight(1.5), elevator)
    .until(() -> elevator.atGoalHeight());
```

---

## 🎓 Learning Resources

### Start Here:
1. **Read existing code** - Best way to learn patterns
2. **Make small changes** - Change one number at a time
3. **Ask questions** - No question is too simple!
4. **Use print statements** - `Commands.print("Debug info")`

### Useful Debug Commands:

```java
// Print to console
Commands.print("Robot reached position!")

// Print a value
Commands.runOnce(() -> {
    System.out.println("Elevator height: " + elevator.getCarriageHeight());
})

// Log to SmartDashboard
SmartDashboard.putNumber("My Value", 123);
SmartDashboard.putBoolean("Is Ready", true);
SmartDashboard.putString("Status", "Running");
```

### Key WPILib Documentation:
- Commands: https://docs.wpilib.org/en/stable/docs/software/commandbased/
- SmartDashboard: https://docs.wpilib.org/en/stable/docs/software/dashboards/
- Motor Controllers: https://docs.revrobotics.com/brushless

---

## 🚀 Your First Project Ideas

### Beginner Projects:
- [ ] Adjust a scoring height by 1 inch
- [ ] Change drive speed for practice
- [ ] Add a print statement when scoring
- [ ] Add a value to SmartDashboard

### Intermediate Projects:
- [ ] Create a new button combination
- [ ] Modify an auto routine path
- [ ] Add SmartDashboard display for a subsystem
- [ ] Create a simple command method

### Advanced Projects:
- [ ] Create a new autonomous routine
- [ ] Add a new control mode
- [ ] Implement LED status indicators
- [ ] Add sensor feedback loops

---

## ❓ FAQ

**Q: I broke something! What do I do?**
A: Don't panic! Use git to revert:
```bash
git checkout -- filename.java    # Revert one file
git checkout .                   # Revert all changes
```

**Q: How do I know what values are safe?**
A: Check the current values first, make small changes (10-20%)

**Q: The robot isn't doing what I expect?**
A: Add print statements, check SmartDashboard, verify deployment worked

**Q: Can I test without the robot?**
A: Yes! Use simulation: `./gradlew simulateJava`

**Q: Where are the errors showing?**
A: Check the terminal (build errors) and Driver Station (runtime errors)

**Q: How do I undo my changes?**
A: Use git:
```bash
git status              # See what changed
git diff                # See the differences
git checkout -- file    # Undo changes to one file
```

---

*Remember: Every expert was once a beginner. Start small, test often, and don't be afraid to ask for help! 🤖*
