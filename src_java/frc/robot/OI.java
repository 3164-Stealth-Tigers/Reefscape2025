package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * ============================================================================
 * OI.JAVA - Operator Interface (Controller Definitions)
 * ============================================================================
 *
 * WHAT THIS FILE DOES:
 * This file defines HOW controllers work. It maps physical buttons to
 * logical actions like "forward", "strafe", "level1", etc.
 *
 * WHY IT'S ORGANIZED THIS WAY:
 * We use INTERFACES to define what a controller should do, then
 * IMPLEMENTATIONS to say which buttons do those things. This lets us
 * easily swap controllers without changing other code!
 *
 * ============================================================================
 * FILE STRUCTURE
 * ============================================================================
 *
 * 1. INTERFACES (What controls are needed)
 *    - DriverActionSet: Controls for driving the robot
 *    - OperatorActionSet: Controls for mechanisms
 *    - ScoringPositionsActionSet: Controls for reef position buttons
 *
 * 2. IMPLEMENTATIONS (Which buttons do what)
 *    - XboxDriver: Xbox controller for driver
 *    - XboxOperator: Xbox controller for operator
 *    - PS4Driver: PS4 controller for driver
 *    - T16000MDriver: Flight stick for driver
 *    - ArcadeScoringPositions: Button board for reef
 *    - PS4ScoringPositions: PS4 for reef
 *    - KeyboardScoringPositions: Xbox as keyboard for reef
 *
 * ============================================================================
 * HOW TO ADD A NEW BUTTON
 * ============================================================================
 *
 * 1. Add the method to the INTERFACE:
 *    Trigger myNewButton();
 *
 * 2. Add the implementation to EACH controller class:
 *    @Override
 *    public Trigger myNewButton() {
 *        return stick.a();  // Or whatever button
 *    }
 *
 * 3. Use it in RobotContainer.java:
 *    operatorJoystick.myNewButton().onTrue(myCommand);
 *
 * ============================================================================
 * CONTROLLER BUTTON MAPS
 * ============================================================================
 *
 * XBOX CONTROLLER:
 * ┌─────────────────────────────────────────┐
 * │    LB          [≡]  [☰]          RB    │
 * │    LT                            RT    │
 * │         [LS]              (Y)         │
 * │                       (X)   (B)       │
 * │              [D-PAD]     (A)          │
 * │                    [RS]               │
 * └─────────────────────────────────────────┘
 *
 * LS = Left Stick (also button when pressed)
 * RS = Right Stick (also button when pressed)
 * LT/RT = Triggers (analog 0-1)
 * LB/RB = Bumpers (digital)
 * D-PAD = POV (up/down/left/right)
 * [≡] = Back button
 * [☰] = Start button
 *
 * PS4 CONTROLLER:
 * Similar layout but different names:
 * - L1/R1 = Bumpers
 * - L2/R2 = Triggers
 * - Options = Start
 * - Share = Back
 * - Triangle/Square/Cross/Circle = Y/X/A/B
 *
 * ============================================================================
 */
public final class OI {

    // Private constructor prevents instantiation
    // This class is just a container for interfaces and implementations
    private OI() {
    }

    // ========================================================================
    // UTILITY METHODS
    // ========================================================================

    /**
     * Apply deadband to joystick input.
     *
     * WHAT IS DEADBAND?
     * Joysticks rarely return exactly 0 when released. They might return
     * 0.02 or -0.01 due to small physical imperfections. Deadband treats
     * any value within a small range around 0 as actually being 0.
     *
     * @param value The raw joystick value (-1 to 1)
     * @param band The deadband threshold (typically 0.05 to 0.1)
     * @return 0 if within deadband, otherwise the original value
     *
     * EXAMPLE:
     *   deadband(0.03, 0.1) returns 0.0  (within deadband)
     *   deadband(0.5, 0.1) returns 0.5   (outside deadband)
     */
    public static double deadband(double value, double band) {
        return Math.abs(value) > band ? value : 0;
    }

    // ========================================================================
    // ========================================================================
    //
    //                        INTERFACE DEFINITIONS
    //
    // ========================================================================
    // ========================================================================
    // These interfaces define WHAT controls a controller needs.
    // Think of them as a "contract" - any controller that wants to be
    // a driver controller must provide all these methods.
    // ========================================================================

    /**
     * DRIVER CONTROLLER INTERFACE
     *
     * The driver controls the robot's movement (drivetrain).
     *
     * REQUIRED CONTROLS:
     * - forward/strafe/turn: Analog axes for driving
     * - resetGyro: Re-zeros the "forward" direction
     * - toggleSpeed: Switch between fast and slow mode
     * - toggleFieldRelative: Switch between field and robot relative
     * - skiStop: Lock wheels to resist being pushed
     */
    public interface DriverActionSet {
        /**
         * Movement along the X axis (forward/backward).
         * @return Value from -1 (full backward) to 1 (full forward)
         */
        double forward();

        /**
         * Movement along the Y axis (left/right strafe).
         * @return Value from -1 (full left) to 1 (full right)
         */
        double strafe();

        /**
         * Rotation around the Z axis (turning).
         * @return Value from -1 (full clockwise) to 1 (full counter-clockwise)
         */
        double turn();

        /** Reset the gyroscope to know which way is "forward" */
        Trigger resetGyro();

        /** Toggle between fast and slow driving modes */
        Trigger toggleSpeed();

        /** Toggle between field-relative and robot-relative control */
        Trigger toggleFieldRelative();

        /** Lock wheels in X pattern to resist pushing (ski stop) */
        Trigger skiStop();

        /**
         * Check if the driver is commanding any movement.
         * Used to release ski stop when driver wants to move.
         * @return true if any movement axis is non-zero
         */
        boolean isMovementCommanded();
    }

    /**
     * OPERATOR CONTROLLER INTERFACE
     *
     * The operator controls the robot's mechanisms (elevator, arm, claw, etc.)
     *
     * REQUIRED CONTROLS:
     * - Level buttons: Move to preset scoring positions
     * - Intake/outtake: Grab and release game pieces
     * - Climber controls: Raise and lower climber
     * - Manual controls: Direct control of elevator/arm
     * - Utility: Homing, automation toggle
     */
    public interface OperatorActionSet {
        // --------------------------------------------------------------------
        // LEVEL SELECTION - Preset positions for scoring
        // --------------------------------------------------------------------
        Trigger loadingLevel();  // Level 0: Pickup position
        Trigger level1();        // Level 1: Lowest scoring
        Trigger level2();        // Level 2
        Trigger level3();        // Level 3
        Trigger level4();        // Level 4: Highest scoring

        // --------------------------------------------------------------------
        // CLIMBER CONTROLS - End-game robot lifting
        // --------------------------------------------------------------------
        Trigger climberUp();     // Extend climber
        Trigger climberDown();   // Retract climber

        // --------------------------------------------------------------------
        // ALGAE ARM CONTROLS - Secondary game piece mechanism
        // --------------------------------------------------------------------
        Trigger algaeArmStowed();    // Tuck arm away
        Trigger algaeArmExtended();  // Extend arm

        // --------------------------------------------------------------------
        // INTAKE CONTROLS - Grabbing and releasing pieces
        // --------------------------------------------------------------------
        Trigger intake();   // Spin intake to grab piece
        Trigger outtake();  // Spin intake to release piece

        // --------------------------------------------------------------------
        // MANUAL CONTROLS - Direct operator control (bypasses presets)
        // --------------------------------------------------------------------
        double elevator();  // Manual elevator speed (-1 to 1)
        double coralArm();  // Manual arm speed (-1 to 1)

        // --------------------------------------------------------------------
        // UTILITY CONTROLS
        // --------------------------------------------------------------------
        Trigger homeElevator();  // Run homing sequence to recalibrate
        Trigger autoToggle();    // Enable/disable automatic scoring
    }

    /**
     * SCORING POSITIONS INTERFACE
     *
     * The button board provides quick access to all reef positions.
     *
     * The reef has 12 scoring locations (A through L) arranged in a hexagon,
     * plus left/right coral station buttons.
     */
    public interface ScoringPositionsActionSet {
        // Reef positions A through L
        Trigger reefA();
        Trigger reefB();
        Trigger reefC();
        Trigger reefD();
        Trigger reefE();
        Trigger reefF();
        Trigger reefG();
        Trigger reefH();
        Trigger reefI();
        Trigger reefJ();
        Trigger reefK();
        Trigger reefL();

        // Coral station positions
        Trigger stationLeft();
        Trigger stationRight();
    }

    // ========================================================================
    // ========================================================================
    //
    //                    CONTROLLER IMPLEMENTATIONS
    //
    // ========================================================================
    // ========================================================================
    // These classes implement the interfaces above for specific controllers.
    // They map physical buttons to the logical actions.
    // ========================================================================

    // ========================================================================
    // XBOX DRIVER IMPLEMENTATION
    // ========================================================================

    /**
     * Xbox controller setup for the DRIVER.
     *
     * BUTTON MAPPING:
     * ┌─────────────────────────────────────────────────────┐
     * │  Control          │  Button/Axis                   │
     * ├─────────────────────────────────────────────────────┤
     * │  Forward/Back     │  Left Stick Y                  │
     * │  Strafe L/R       │  Left Stick X                  │
     * │  Turn             │  Right Stick X (70% speed)     │
     * │  Reset Gyro       │  Start Button                  │
     * │  Toggle Speed     │  X Button                      │
     * │  Field Relative   │  Back Button                   │
     * │  Ski Stop         │  Y Button                      │
     * └─────────────────────────────────────────────────────┘
     */
    public static class XboxDriver implements DriverActionSet {
        private final CommandXboxController stick;

        public XboxDriver(int port) {
            this.stick = new CommandXboxController(port);
        }

        @Override
        public double forward() {
            // Negative because Y axis is inverted on controllers
            return deadband(-stick.getLeftY(), 0.08);
        }

        @Override
        public double strafe() {
            // Negative to match field coordinate system (left = positive Y)
            return deadband(-stick.getLeftX(), 0.08);
        }

        @Override
        public double turn() {
            // 70% speed multiplier for more controlled turning
            return deadband(-stick.getRightX(), 0.08) * 0.7;
        }

        @Override
        public Trigger toggleSpeed() {
            return stick.x();
        }

        @Override
        public Trigger resetGyro() {
            return stick.start();
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.back();
        }

        @Override
        public Trigger skiStop() {
            return stick.y();
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

    // ========================================================================
    // XBOX OPERATOR IMPLEMENTATION
    // ========================================================================

    /**
     * Xbox controller setup for the OPERATOR.
     *
     * BUTTON MAPPING:
     * ┌─────────────────────────────────────────────────────┐
     * │  Control          │  Button                        │
     * ├─────────────────────────────────────────────────────┤
     * │  Loading Level    │  Right Trigger                 │
     * │  Level 1          │  A Button                      │
     * │  Level 2          │  X Button                      │
     * │  Level 3          │  B Button                      │
     * │  Level 4          │  Y Button                      │
     * │  Climber Up       │  D-Pad Up                      │
     * │  Climber Down     │  D-Pad Down                    │
     * │  Algae Extend     │  D-Pad Left                    │
     * │  Intake           │  Left Trigger                  │
     * │  Outtake          │  Left Bumper                   │
     * │  Manual Elevator  │  Left Stick Y                  │
     * │  Manual Arm       │  Right Stick Y                 │
     * │  Home Elevator    │  Start Button                  │
     * │  Auto Toggle      │  Back Button                   │
     * └─────────────────────────────────────────────────────┘
     */
    public static class XboxOperator implements OperatorActionSet {
        private final CommandXboxController stick;

        public XboxOperator(int port) {
            this.stick = new CommandXboxController(port);
        }

        @Override
        public Trigger loadingLevel() {
            return stick.rightTrigger();
        }

        @Override
        public Trigger level1() {
            return stick.a();
        }

        @Override
        public Trigger level2() {
            return stick.x();
        }

        @Override
        public Trigger level3() {
            return stick.b();
        }

        @Override
        public Trigger level4() {
            return stick.y();
        }

        @Override
        public Trigger climberUp() {
            return stick.povUp();
        }

        @Override
        public Trigger climberDown() {
            return stick.povDown();
        }

        @Override
        public Trigger algaeArmStowed() {
            // Not mapped on Xbox - returns a trigger that's always false
            return new Trigger(() -> false);
        }

        @Override
        public Trigger algaeArmExtended() {
            return stick.povLeft();
        }

        @Override
        public double elevator() {
            return deadband(-stick.getLeftY(), 0.08);
        }

        @Override
        public double coralArm() {
            return deadband(-stick.getRightY(), 0.08);
        }

        @Override
        public Trigger intake() {
            return stick.leftTrigger();
        }

        @Override
        public Trigger outtake() {
            return stick.leftBumper();
        }

        @Override
        public Trigger homeElevator() {
            return stick.start();
        }

        @Override
        public Trigger autoToggle() {
            return stick.back();
        }
    }

    // ========================================================================
    // PS4 DRIVER IMPLEMENTATION
    // ========================================================================

    /**
     * PS4 controller setup for the DRIVER.
     *
     * Similar to Xbox but with PS4 button names.
     */
    public static class PS4Driver implements DriverActionSet {
        private final CommandPS4Controller stick;

        public PS4Driver(int port) {
            this.stick = new CommandPS4Controller(port);
        }

        @Override
        public double forward() {
            return deadband(-stick.getLeftY(), 0.08);
        }

        @Override
        public double strafe() {
            return deadband(-stick.getLeftX(), 0.08);
        }

        @Override
        public double turn() {
            // 60% speed for PS4 (slightly slower than Xbox)
            return deadband(-stick.getRightX(), 0.08) * 0.6;
        }

        @Override
        public Trigger resetGyro() {
            return stick.options();  // "Start" on PS4
        }

        @Override
        public Trigger toggleSpeed() {
            // Not mapped on PS4
            return new Trigger(() -> false);
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.share();  // "Back" on PS4
        }

        @Override
        public Trigger skiStop() {
            return stick.triangle();  // "Y" equivalent
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

    // ========================================================================
    // T16000M FLIGHT STICK DRIVER IMPLEMENTATION
    // ========================================================================

    /**
     * Thrustmaster T.16000M flight stick setup for the DRIVER.
     *
     * Some drivers prefer flight sticks for precise control.
     */
    public static class T16000MDriver implements DriverActionSet {
        private final CommandJoystick stick;

        public T16000MDriver(int port) {
            this.stick = new CommandJoystick(port);
        }

        @Override
        public double forward() {
            // Very small deadband for flight stick precision
            return deadband(-stick.getRawAxis(1), 0.001);
        }

        @Override
        public double strafe() {
            return deadband(-stick.getRawAxis(0), 0.001);
        }

        @Override
        public double turn() {
            // Twist axis for rotation
            return deadband(-stick.getRawAxis(2), 0.01) * 0.6;
        }

        @Override
        public Trigger resetGyro() {
            return stick.button(8);
        }

        @Override
        public Trigger toggleSpeed() {
            return new Trigger(() -> false);
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.button(9);
        }

        @Override
        public Trigger skiStop() {
            return stick.trigger();  // Main trigger button
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

    // ========================================================================
    // ARCADE BUTTON BOARD - SCORING POSITIONS
    // ========================================================================

    /**
     * Arcade-style button board for reef scoring positions.
     *
     * This is typically a custom board with 12 arcade buttons
     * for positions A-L and a joystick for station selection.
     *
     * BUTTON MAPPING:
     * - Buttons 1-12: Reef positions A through L
     * - Joystick Left/Right: Coral stations
     */
    public static class ArcadeScoringPositions implements ScoringPositionsActionSet {
        private final CommandJoystick stick;

        public ArcadeScoringPositions(int port) {
            this.stick = new CommandJoystick(port);
        }

        @Override
        public Trigger stationRight() {
            return stick.axisGreaterThan(0, 0.5);
        }

        @Override
        public Trigger stationLeft() {
            return stick.axisLessThan(0, -0.5);
        }

        // Reef positions mapped to buttons 1-12
        @Override public Trigger reefA() { return stick.button(1); }
        @Override public Trigger reefB() { return stick.button(2); }
        @Override public Trigger reefC() { return stick.button(3); }
        @Override public Trigger reefD() { return stick.button(4); }
        @Override public Trigger reefE() { return stick.button(5); }
        @Override public Trigger reefF() { return stick.button(6); }
        @Override public Trigger reefG() { return stick.button(7); }
        @Override public Trigger reefH() { return stick.button(8); }
        @Override public Trigger reefI() { return stick.button(9); }
        @Override public Trigger reefJ() { return stick.button(10); }
        @Override public Trigger reefK() { return stick.button(11); }
        @Override public Trigger reefL() { return stick.button(12); }
    }

    // ========================================================================
    // PS4 CONTROLLER - SCORING POSITIONS
    // ========================================================================

    /**
     * PS4 controller as a backup for scoring positions.
     *
     * Maps reef positions to PS4 buttons and D-pad.
     * Useful when the button board isn't available.
     */
    public static class PS4ScoringPositions implements ScoringPositionsActionSet {
        private final CommandPS4Controller stick;

        public PS4ScoringPositions(int port) {
            this.stick = new CommandPS4Controller(port);
        }

        // D-pad positions
        @Override public Trigger reefA() { return stick.povUp(); }
        @Override public Trigger reefB() { return stick.povLeft(); }
        @Override public Trigger reefC() { return stick.povDown(); }
        @Override public Trigger reefD() { return stick.povRight(); }

        // Face buttons
        @Override public Trigger reefE() { return stick.triangle(); }
        @Override public Trigger reefF() { return stick.square(); }
        @Override public Trigger reefG() { return stick.cross(); }
        @Override public Trigger reefH() { return stick.circle(); }

        // Shoulder buttons
        @Override public Trigger reefI() { return stick.L1(); }
        @Override public Trigger reefJ() { return stick.L2(); }
        @Override public Trigger reefK() { return stick.R1(); }
        @Override public Trigger reefL() { return stick.R2(); }

        // Stick buttons for stations
        @Override public Trigger stationLeft() { return stick.L3(); }
        @Override public Trigger stationRight() { return stick.R3(); }
    }

    // ========================================================================
    // XBOX CONTROLLER - SCORING POSITIONS (Keyboard Style)
    // ========================================================================

    /**
     * Xbox controller as a backup for scoring positions.
     *
     * Maps reef positions to Xbox buttons and D-pad.
     * Called "Keyboard" style because it's an alternate mapping.
     */
    public static class KeyboardScoringPositions implements ScoringPositionsActionSet {
        private final CommandXboxController stick;

        public KeyboardScoringPositions(int port) {
            this.stick = new CommandXboxController(port);
        }

        // D-pad positions
        @Override public Trigger reefA() { return stick.povUp(); }
        @Override public Trigger reefB() { return stick.povRight(); }
        @Override public Trigger reefC() { return stick.povLeft(); }
        @Override public Trigger reefD() { return stick.povDown(); }

        // Face buttons
        @Override public Trigger reefE() { return stick.x(); }
        @Override public Trigger reefF() { return stick.a(); }
        @Override public Trigger reefG() { return stick.b(); }
        @Override public Trigger reefH() { return stick.y(); }

        // Triggers and bumpers
        @Override public Trigger reefI() { return stick.rightTrigger(); }
        @Override public Trigger reefJ() { return stick.rightBumper(); }
        @Override public Trigger reefK() { return stick.leftTrigger(); }
        @Override public Trigger reefL() { return stick.leftBumper(); }

        // Stations not mapped on this controller
        @Override public Trigger stationLeft() { return new Trigger(() -> false); }
        @Override public Trigger stationRight() { return new Trigger(() -> false); }
    }
}
