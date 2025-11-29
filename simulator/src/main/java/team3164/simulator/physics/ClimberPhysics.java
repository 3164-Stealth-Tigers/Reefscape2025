package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.RobotState;

/**
 * Simulates climber mechanism behavior.
 *
 * The climber extends/retracts based on button input.
 * Simple velocity control with position limits.
 */
public class ClimberPhysics {

    /**
     * Update climber state based on inputs.
     *
     * @param state Current robot state (modified in place)
     * @param input Current input state
     * @param dt Time step in seconds
     */
    public static void update(RobotState state, InputState input, double dt) {
        double targetVelocity = 0;

        if (input.climberUp) {
            targetVelocity = Constants.Climber.MAX_VELOCITY;
        } else if (input.climberDown) {
            targetVelocity = -Constants.Climber.MAX_VELOCITY;
        }

        // Smooth velocity change
        double velocityDiff = targetVelocity - state.climberVelocity;
        double maxChange = Constants.Climber.MAX_VELOCITY * 2 * dt;  // Ramp rate

        if (Math.abs(velocityDiff) <= maxChange) {
            state.climberVelocity = targetVelocity;
        } else {
            state.climberVelocity += Math.signum(velocityDiff) * maxChange;
        }

        // Update position
        state.climberPosition += state.climberVelocity * dt;

        // Clamp to limits
        if (state.climberPosition < Constants.Climber.MIN_POSITION) {
            state.climberPosition = Constants.Climber.MIN_POSITION;
            state.climberVelocity = 0;
        } else if (state.climberPosition > Constants.Climber.MAX_POSITION) {
            state.climberPosition = Constants.Climber.MAX_POSITION;
            state.climberVelocity = 0;
        }
    }
}
