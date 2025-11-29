package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.RobotState;

/**
 * Simulates elevator motion using trapezoidal motion profile.
 *
 * The elevator uses a trapezoidal velocity profile for smooth movement:
 * - Accelerates to max velocity
 * - Cruises at max velocity
 * - Decelerates to stop at target
 */
public class ElevatorPhysics {

    /**
     * Update elevator state toward goal height.
     *
     * @param state Current robot state (modified in place)
     * @param dt Time step in seconds
     */
    public static void update(RobotState state, double dt) {
        double error = state.elevatorGoal - state.elevatorHeight;
        double absError = Math.abs(error);

        // Calculate stopping distance at current velocity
        double stoppingDistance = (state.elevatorVelocity * state.elevatorVelocity) /
                                  (2 * Constants.Elevator.MAX_ACCELERATION);

        double desiredVelocity;

        if (absError < Constants.Elevator.TOLERANCE) {
            // At goal - stop
            desiredVelocity = 0;
            state.elevatorAtGoal = true;
        } else if (absError <= stoppingDistance) {
            // Need to decelerate
            desiredVelocity = Math.signum(error) *
                Math.sqrt(2 * Constants.Elevator.MAX_ACCELERATION * absError);
            state.elevatorAtGoal = false;
        } else {
            // Accelerate or cruise
            desiredVelocity = Math.signum(error) * Constants.Elevator.MAX_VELOCITY;
            state.elevatorAtGoal = false;
        }

        // Apply acceleration limit
        double velocityError = desiredVelocity - state.elevatorVelocity;
        double maxVelocityChange = Constants.Elevator.MAX_ACCELERATION * dt;

        if (Math.abs(velocityError) <= maxVelocityChange) {
            state.elevatorVelocity = desiredVelocity;
        } else {
            state.elevatorVelocity += Math.signum(velocityError) * maxVelocityChange;
        }

        // Update position
        state.elevatorHeight += state.elevatorVelocity * dt;

        // Clamp to limits
        if (state.elevatorHeight < Constants.Elevator.MIN_HEIGHT) {
            state.elevatorHeight = Constants.Elevator.MIN_HEIGHT;
            state.elevatorVelocity = 0;
        } else if (state.elevatorHeight > Constants.Elevator.MAX_HEIGHT) {
            state.elevatorHeight = Constants.Elevator.MAX_HEIGHT;
            state.elevatorVelocity = 0;
        }

        // Final at-goal check
        if (Math.abs(state.elevatorHeight - state.elevatorGoal) < Constants.Elevator.TOLERANCE &&
            Math.abs(state.elevatorVelocity) < 0.01) {
            state.elevatorAtGoal = true;
            state.elevatorVelocity = 0;
        }
    }

    /**
     * Set the elevator goal to a specific height.
     *
     * @param state Robot state
     * @param height Target height in meters
     */
    public static void setGoal(RobotState state, double height) {
        // Clamp to valid range
        height = Math.max(Constants.Elevator.MIN_HEIGHT,
                 Math.min(Constants.Elevator.MAX_HEIGHT, height));

        state.elevatorGoal = height;
        state.elevatorAtGoal = false;
    }

    /**
     * Set elevator to a scoring level.
     *
     * @param state Robot state
     * @param level 0=loading, 1-4=scoring levels
     */
    public static void setLevel(RobotState state, int level) {
        double height;
        switch (level) {
            case 0: height = Constants.Scoring.LOADING_HEIGHT; break;
            case 1: height = Constants.Scoring.L1_HEIGHT; break;
            case 2: height = Constants.Scoring.L2_HEIGHT; break;
            case 3: height = Constants.Scoring.L3_HEIGHT; break;
            case 4: height = Constants.Scoring.L4_HEIGHT; break;
            default: return;
        }
        setGoal(state, height);
        state.currentLevel = level;
    }
}
