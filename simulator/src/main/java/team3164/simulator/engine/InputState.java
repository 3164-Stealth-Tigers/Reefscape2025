package team3164.simulator.engine;

/**
 * Current state of all user inputs.
 * Updated from keyboard/gamepad via WebSocket.
 */
public class InputState {

    // ========================================================================
    // DRIVE INPUTS (continuous, -1 to 1)
    // ========================================================================
    public double forward;    // W/S or left stick Y
    public double strafe;     // A/D or left stick X
    public double turn;       // Q/E or right stick X

    // ========================================================================
    // BUTTON INPUTS (discrete, true/false)
    // ========================================================================
    // Subsystem controls
    public boolean level0;    // Loading position (R key or RT)
    public boolean level1;    // Level 1 (1 key or A)
    public boolean level2;    // Level 2 (2 key or X)
    public boolean level3;    // Level 3 (3 key or B)
    public boolean level4;    // Level 4 (4 key or Y)

    public boolean intake;    // Space or LT
    public boolean outtake;   // Shift or LB

    public boolean climberUp;    // Up arrow or POV Up
    public boolean climberDown;  // Down arrow or POV Down

    // Mode toggles
    public boolean toggleSpeed;      // X key
    public boolean toggleFieldRel;   // C key
    public boolean resetGyro;        // G key
    public boolean skiStop;          // V key
    public boolean resetRobot;       // Escape key

    // Reef position shortcuts (for button board simulation)
    public String reefPosition;      // "A" through "L" or null

    /**
     * Reset all inputs to neutral.
     */
    public void reset() {
        forward = 0;
        strafe = 0;
        turn = 0;

        level0 = false;
        level1 = false;
        level2 = false;
        level3 = false;
        level4 = false;

        intake = false;
        outtake = false;

        climberUp = false;
        climberDown = false;

        toggleSpeed = false;
        toggleFieldRel = false;
        resetGyro = false;
        skiStop = false;
        resetRobot = false;

        reefPosition = null;
    }

    /**
     * Check if any level button is pressed.
     */
    public int getRequestedLevel() {
        if (level4) return 4;
        if (level3) return 3;
        if (level2) return 2;
        if (level1) return 1;
        if (level0) return 0;
        return -1;  // No level requested
    }

    /**
     * Check if any drive input is active.
     */
    public boolean hasDriveInput() {
        return Math.abs(forward) > 0.05 ||
               Math.abs(strafe) > 0.05 ||
               Math.abs(turn) > 0.05;
    }

    @Override
    public String toString() {
        return String.format("Input[fwd=%.2f, str=%.2f, turn=%.2f, level=%d]",
            forward, strafe, turn, getRequestedLevel());
    }
}
