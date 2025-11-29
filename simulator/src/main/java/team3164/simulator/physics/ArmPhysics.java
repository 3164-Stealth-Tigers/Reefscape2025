package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.RobotState;

/**
 * Simulates coral arm rotational motion.
 *
 * The arm rotates to position the claw at the correct angle for scoring.
 * Uses similar trapezoidal profile as the elevator but for angular motion.
 */
public class ArmPhysics {

    /**
     * Update arm state toward goal angle.
     *
     * @param state Current robot state (modified in place)
     * @param dt Time step in seconds
     */
    public static void update(RobotState state, double dt) {
        double error = state.armGoal - state.armAngle;
        double absError = Math.abs(error);

        // Calculate stopping angle at current velocity
        double stoppingAngle = (state.armVelocity * state.armVelocity) /
                               (2 * Constants.Arm.MAX_ACCELERATION);

        double desiredVelocity;

        if (absError < Constants.Arm.TOLERANCE) {
            // At goal - stop
            desiredVelocity = 0;
            state.armAtGoal = true;
        } else if (absError <= stoppingAngle) {
            // Need to decelerate
            desiredVelocity = Math.signum(error) *
                Math.sqrt(2 * Constants.Arm.MAX_ACCELERATION * absError);
            state.armAtGoal = false;
        } else {
            // Accelerate or cruise
            desiredVelocity = Math.signum(error) * Constants.Arm.MAX_VELOCITY;
            state.armAtGoal = false;
        }

        // Apply acceleration limit
        double velocityError = desiredVelocity - state.armVelocity;
        double maxVelocityChange = Constants.Arm.MAX_ACCELERATION * dt;

        if (Math.abs(velocityError) <= maxVelocityChange) {
            state.armVelocity = desiredVelocity;
        } else {
            state.armVelocity += Math.signum(velocityError) * maxVelocityChange;
        }

        // Update angle
        state.armAngle += state.armVelocity * dt;

        // Clamp to limits
        if (state.armAngle < Constants.Arm.MIN_ANGLE) {
            state.armAngle = Constants.Arm.MIN_ANGLE;
            state.armVelocity = 0;
        } else if (state.armAngle > Constants.Arm.MAX_ANGLE) {
            state.armAngle = Constants.Arm.MAX_ANGLE;
            state.armVelocity = 0;
        }

        // Final at-goal check
        if (Math.abs(state.armAngle - state.armGoal) < Constants.Arm.TOLERANCE &&
            Math.abs(state.armVelocity) < 0.01) {
            state.armAtGoal = true;
            state.armVelocity = 0;
        }
    }

    /**
     * Set the arm goal to a specific angle.
     *
     * @param state Robot state
     * @param angle Target angle in radians
     */
    public static void setGoal(RobotState state, double angle) {
        // Clamp to valid range
        angle = Math.max(Constants.Arm.MIN_ANGLE,
                Math.min(Constants.Arm.MAX_ANGLE, angle));

        state.armGoal = angle;
        state.armAtGoal = false;
    }

    /**
     * Set arm to angle for a scoring level.
     *
     * @param state Robot state
     * @param level 0=loading, 1-4=scoring levels
     */
    public static void setLevel(RobotState state, int level) {
        double angle;
        switch (level) {
            case 0: angle = Constants.Scoring.LOADING_ANGLE; break;
            case 1: angle = Constants.Scoring.L1_ANGLE; break;
            case 2: angle = Constants.Scoring.L2_ANGLE; break;
            case 3: angle = Constants.Scoring.L3_ANGLE; break;
            case 4: angle = Constants.Scoring.L4_ANGLE; break;
            default: return;
        }
        setGoal(state, angle);
    }
}
