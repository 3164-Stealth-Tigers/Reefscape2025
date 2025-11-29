package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.RobotState;
import team3164.simulator.engine.RobotState.ClawState;

/**
 * Simulates claw intake/outtake behavior.
 *
 * The claw can intake coral pieces and hold them until outtake is triggered.
 * Uses a simple state machine with timers for intake/outtake animations.
 */
public class ClawPhysics {

    /**
     * Update claw state based on inputs.
     *
     * @param state Current robot state (modified in place)
     * @param input Current input state
     * @param dt Time step in seconds
     * @return true if coral was just scored
     */
    public static boolean update(RobotState state, InputState input, double dt) {
        boolean scored = false;

        switch (state.clawState) {
            case EMPTY:
                if (input.intake) {
                    // Start intaking
                    state.clawState = ClawState.INTAKING;
                    state.clawTimer = Constants.Claw.INTAKE_TIME;
                    state.currentCommand = "Intaking";
                }
                break;

            case INTAKING:
                state.clawTimer -= dt;
                if (state.clawTimer <= 0) {
                    // Finished intaking - now holding coral
                    state.clawState = ClawState.HOLDING;
                    state.hasCoral = true;
                    state.currentCommand = "Holding Coral";
                } else if (!input.intake) {
                    // Cancelled intake
                    state.clawState = ClawState.EMPTY;
                    state.currentCommand = "";
                }
                break;

            case HOLDING:
                if (input.outtake) {
                    // Start outtaking
                    state.clawState = ClawState.OUTTAKING;
                    state.clawTimer = Constants.Claw.OUTTAKE_TIME;
                    state.currentCommand = "Outtaking";
                }
                break;

            case OUTTAKING:
                state.clawTimer -= dt;
                if (state.clawTimer <= 0) {
                    // Finished outtaking
                    state.clawState = ClawState.EMPTY;
                    state.hasCoral = false;
                    state.currentCommand = "";

                    // Check if this was a score
                    if (state.currentLevel > 0 && state.isReadyToScore()) {
                        scored = true;
                    }
                }
                break;
        }

        return scored;
    }

    /**
     * Force intake a coral (for testing/initialization).
     */
    public static void pickupCoral(RobotState state) {
        state.clawState = ClawState.HOLDING;
        state.hasCoral = true;
    }

    /**
     * Force drop coral (for testing).
     */
    public static void dropCoral(RobotState state) {
        state.clawState = ClawState.EMPTY;
        state.hasCoral = false;
    }
}
