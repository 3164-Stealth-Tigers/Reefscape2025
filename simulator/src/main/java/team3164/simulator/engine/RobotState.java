package team3164.simulator.engine;

import team3164.simulator.Constants;

/**
 * Complete state of the simulated robot.
 * This is the single source of truth for all robot data.
 */
public class RobotState {

    // ========================================================================
    // POSE (Position on field)
    // ========================================================================
    public double x;         // meters from origin (blue alliance wall)
    public double y;         // meters from origin (right side of field)
    public double heading;   // radians, 0 = facing red alliance, CCW positive

    // ========================================================================
    // VELOCITY
    // ========================================================================
    public double vx;        // m/s in field X direction
    public double vy;        // m/s in field Y direction
    public double omega;     // rad/s angular velocity

    // ========================================================================
    // SWERVE MODULES (FL, FR, RL, RR)
    // ========================================================================
    public double[] moduleAngles = new double[4];    // radians
    public double[] moduleSpeeds = new double[4];    // m/s

    // ========================================================================
    // ELEVATOR
    // ========================================================================
    public double elevatorHeight;       // meters
    public double elevatorVelocity;     // m/s
    public double elevatorGoal;         // target height in meters
    public boolean elevatorAtGoal;

    // ========================================================================
    // CORAL ARM
    // ========================================================================
    public double armAngle;             // radians
    public double armVelocity;          // rad/s
    public double armGoal;              // target angle in radians
    public boolean armAtGoal;

    // ========================================================================
    // CLAW
    // ========================================================================
    public ClawState clawState = ClawState.EMPTY;
    public boolean hasCoral;
    public double clawTimer;            // timer for intake/outtake animation

    // ========================================================================
    // CLIMBER
    // ========================================================================
    public double climberPosition;      // meters
    public double climberVelocity;      // m/s

    // ========================================================================
    // CONTROL MODE
    // ========================================================================
    public boolean fieldRelative = true;
    public boolean slowMode = false;
    public String currentCommand = "";
    public int currentLevel = 0;        // 0 = loading, 1-4 = scoring levels

    // ========================================================================
    // GAME STATE
    // ========================================================================
    public int score = 0;
    public double matchTime = 0;
    public boolean isEnabled = true;

    public enum ClawState {
        EMPTY,
        INTAKING,
        HOLDING,
        OUTTAKING
    }

    /**
     * Initialize robot to starting position.
     */
    public RobotState() {
        reset();
    }

    /**
     * Reset to default starting state.
     */
    public void reset() {
        // Start near blue alliance coral station
        x = 2.0;
        y = Constants.Field.WIDTH / 2.0;
        heading = 0;

        vx = 0;
        vy = 0;
        omega = 0;

        for (int i = 0; i < 4; i++) {
            moduleAngles[i] = 0;
            moduleSpeeds[i] = 0;
        }

        elevatorHeight = Constants.Elevator.MIN_HEIGHT;
        elevatorVelocity = 0;
        elevatorGoal = Constants.Elevator.MIN_HEIGHT;
        elevatorAtGoal = true;

        armAngle = 0;
        armVelocity = 0;
        armGoal = 0;
        armAtGoal = true;

        clawState = ClawState.EMPTY;
        hasCoral = false;
        clawTimer = 0;

        climberPosition = 0;
        climberVelocity = 0;

        fieldRelative = true;
        slowMode = false;
        currentCommand = "";
        currentLevel = 0;

        score = 0;
        matchTime = 0;
        isEnabled = true;
    }

    /**
     * Check if robot is ready to score (elevator and arm at goal).
     */
    public boolean isReadyToScore() {
        return elevatorAtGoal && armAtGoal && hasCoral && currentLevel > 0;
    }

    /**
     * Get height in inches for display.
     */
    public double getElevatorHeightInches() {
        return elevatorHeight * 39.37;
    }

    /**
     * Get arm angle in degrees for display.
     */
    public double getArmAngleDegrees() {
        return Math.toDegrees(armAngle);
    }

    /**
     * Get heading in degrees for display.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }
}
