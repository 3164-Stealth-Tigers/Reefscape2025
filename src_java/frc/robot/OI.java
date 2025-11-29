package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The OI (Operator Interface) module contains Action Set interfaces and Control Scheme implementations.
 *
 * An Action Set defines the inputs a controller should have. For example, a driver's controller
 * needs an input for driving forwards and backwards.
 *
 * A Control Scheme implements an Action Set and defines which physical buttons or joysticks
 * on a controller perform each action.
 */
public final class OI {

    private OI() {
        // Utility class - prevent instantiation
    }

    /**
     * Apply deadband to a value.
     *
     * @param value The input value
     * @param band The deadband threshold
     * @return 0 if within deadband, otherwise the original value
     */
    public static double deadband(double value, double band) {
        return Math.abs(value) > band ? value : 0;
    }

    // ==================== Action Set Interfaces ====================

    /**
     * Interface defining driver control inputs.
     */
    public interface DriverActionSet {
        /** Movement along the X axis, from -1 to 1 */
        double forward();

        /** Movement along the Y axis, from -1 to 1 */
        double strafe();

        /** Rotation around the Z axis, from -1 (CW) to 1 (CCW) */
        double turn();

        /** Reset the gyroscope */
        Trigger resetGyro();

        /** Toggle between normal and slow speed */
        Trigger toggleSpeed();

        /** Toggle field-relative control */
        Trigger toggleFieldRelative();

        /** Turn wheels to X pattern (ski stop) */
        Trigger skiStop();

        /** Return true if any movement is commanded */
        boolean isMovementCommanded();
    }

    /**
     * Interface defining operator control inputs.
     */
    public interface OperatorActionSet {
        // Level selection
        Trigger loadingLevel();
        Trigger level1();
        Trigger level2();
        Trigger level3();
        Trigger level4();

        // Climber controls
        Trigger climberUp();
        Trigger climberDown();

        // Algae arm controls
        Trigger algaeArmStowed();
        Trigger algaeArmExtended();

        // Intake controls
        Trigger intake();
        Trigger outtake();

        // Manual controls
        double elevator();
        double coralArm();

        // Utility
        Trigger homeElevator();
        Trigger autoToggle();
    }

    /**
     * Interface defining scoring position inputs.
     */
    public interface ScoringPositionsActionSet {
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
        Trigger stationLeft();
        Trigger stationRight();
    }

    // ==================== Control Scheme Implementations ====================

    /**
     * Xbox controller implementation for driver controls.
     */
    public static class XboxDriver implements DriverActionSet {
        private final CommandXboxController stick;

        public XboxDriver(int port) {
            this.stick = new CommandXboxController(port);
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

    /**
     * Xbox controller implementation for operator controls.
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

    /**
     * PS4 controller implementation for driver controls.
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
            return deadband(-stick.getRightX(), 0.08) * 0.6;
        }

        @Override
        public Trigger resetGyro() {
            return stick.options();
        }

        @Override
        public Trigger toggleSpeed() {
            return new Trigger(() -> false);
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.share();
        }

        @Override
        public Trigger skiStop() {
            return stick.triangle();
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

    /**
     * T.16000M flight stick implementation for driver controls.
     */
    public static class T16000MDriver implements DriverActionSet {
        private final CommandJoystick stick;

        public T16000MDriver(int port) {
            this.stick = new CommandJoystick(port);
        }

        @Override
        public double forward() {
            return deadband(-stick.getRawAxis(1), 0.001);
        }

        @Override
        public double strafe() {
            return deadband(-stick.getRawAxis(0), 0.001);
        }

        @Override
        public double turn() {
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
            return stick.trigger();
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

    /**
     * Arcade-style button board for scoring positions.
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

        @Override
        public Trigger reefA() { return stick.button(1); }
        @Override
        public Trigger reefB() { return stick.button(2); }
        @Override
        public Trigger reefC() { return stick.button(3); }
        @Override
        public Trigger reefD() { return stick.button(4); }
        @Override
        public Trigger reefE() { return stick.button(5); }
        @Override
        public Trigger reefF() { return stick.button(6); }
        @Override
        public Trigger reefG() { return stick.button(7); }
        @Override
        public Trigger reefH() { return stick.button(8); }
        @Override
        public Trigger reefI() { return stick.button(9); }
        @Override
        public Trigger reefJ() { return stick.button(10); }
        @Override
        public Trigger reefK() { return stick.button(11); }
        @Override
        public Trigger reefL() { return stick.button(12); }
    }

    /**
     * PS4 controller for scoring positions.
     */
    public static class PS4ScoringPositions implements ScoringPositionsActionSet {
        private final CommandPS4Controller stick;

        public PS4ScoringPositions(int port) {
            this.stick = new CommandPS4Controller(port);
        }

        @Override
        public Trigger reefA() { return stick.povUp(); }
        @Override
        public Trigger reefB() { return stick.povLeft(); }
        @Override
        public Trigger reefC() { return stick.povDown(); }
        @Override
        public Trigger reefD() { return stick.povRight(); }
        @Override
        public Trigger reefE() { return stick.triangle(); }
        @Override
        public Trigger reefF() { return stick.square(); }
        @Override
        public Trigger reefG() { return stick.cross(); }
        @Override
        public Trigger reefH() { return stick.circle(); }
        @Override
        public Trigger reefI() { return stick.L1(); }
        @Override
        public Trigger reefJ() { return stick.L2(); }
        @Override
        public Trigger reefK() { return stick.R1(); }
        @Override
        public Trigger reefL() { return stick.R2(); }
        @Override
        public Trigger stationLeft() { return stick.L3(); }
        @Override
        public Trigger stationRight() { return stick.R3(); }
    }

    /**
     * Xbox controller as keyboard emulation for scoring positions.
     */
    public static class KeyboardScoringPositions implements ScoringPositionsActionSet {
        private final CommandXboxController stick;

        public KeyboardScoringPositions(int port) {
            this.stick = new CommandXboxController(port);
        }

        @Override
        public Trigger reefA() { return stick.povUp(); }
        @Override
        public Trigger reefB() { return stick.povRight(); }
        @Override
        public Trigger reefC() { return stick.povLeft(); }
        @Override
        public Trigger reefD() { return stick.povDown(); }
        @Override
        public Trigger reefE() { return stick.x(); }
        @Override
        public Trigger reefF() { return stick.a(); }
        @Override
        public Trigger reefG() { return stick.b(); }
        @Override
        public Trigger reefH() { return stick.y(); }
        @Override
        public Trigger reefI() { return stick.rightTrigger(); }
        @Override
        public Trigger reefJ() { return stick.rightBumper(); }
        @Override
        public Trigger reefK() { return stick.leftTrigger(); }
        @Override
        public Trigger reefL() { return stick.leftBumper(); }
        @Override
        public Trigger stationLeft() { return new Trigger(() -> false); }
        @Override
        public Trigger stationRight() { return new Trigger(() -> false); }
    }
}
