package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.RobotState;

/**
 * Simulates swerve drive kinematics and dynamics.
 *
 * This models the holonomic (omnidirectional) motion of a 4-wheel
 * swerve drive robot, including acceleration limits and field-relative control.
 */
public class SwervePhysics {

    // Module positions relative to robot center (FL, FR, RL, RR)
    private static final double[][] MODULE_POSITIONS = {
        { Constants.Swerve.WHEEL_BASE / 2,  Constants.Swerve.TRACK_WIDTH / 2},  // FL
        { Constants.Swerve.WHEEL_BASE / 2, -Constants.Swerve.TRACK_WIDTH / 2},  // FR
        {-Constants.Swerve.WHEEL_BASE / 2,  Constants.Swerve.TRACK_WIDTH / 2},  // RL
        {-Constants.Swerve.WHEEL_BASE / 2, -Constants.Swerve.TRACK_WIDTH / 2}   // RR
    };

    /**
     * Update swerve drive state based on inputs.
     *
     * @param state Current robot state (modified in place)
     * @param input Current input state
     * @param dt Time step in seconds
     */
    public static void update(RobotState state, InputState input, double dt) {
        if (!state.isEnabled) {
            // Robot disabled - coast to stop
            state.vx *= 0.95;
            state.vy *= 0.95;
            state.omega *= 0.95;
            updatePose(state, dt);
            return;
        }

        // Handle ski stop
        if (input.skiStop) {
            // Set modules to X pattern and stop
            state.moduleAngles[0] = Math.toRadians(45);
            state.moduleAngles[1] = Math.toRadians(-45);
            state.moduleAngles[2] = Math.toRadians(-45);
            state.moduleAngles[3] = Math.toRadians(45);

            for (int i = 0; i < 4; i++) {
                state.moduleSpeeds[i] = 0;
            }

            state.vx = 0;
            state.vy = 0;
            state.omega = 0;
            return;
        }

        // Get speed multiplier
        double speedMult = state.slowMode ? 0.3 : 1.0;

        // Calculate desired velocities from input
        double desiredVx = input.forward * Constants.Swerve.MAX_SPEED * speedMult;
        double desiredVy = input.strafe * Constants.Swerve.MAX_SPEED * speedMult;
        double desiredOmega = input.turn * Constants.Swerve.MAX_ANGULAR_VELOCITY * speedMult;

        // Apply field-relative transformation
        if (state.fieldRelative) {
            double cos = Math.cos(state.heading);
            double sin = Math.sin(state.heading);

            double tempVx = desiredVx * cos + desiredVy * sin;
            double tempVy = -desiredVx * sin + desiredVy * cos;

            desiredVx = tempVx;
            desiredVy = tempVy;
        }

        // Apply acceleration limits (smooth response)
        state.vx = applyAccelLimit(state.vx, desiredVx, Constants.Swerve.MAX_ACCELERATION, dt);
        state.vy = applyAccelLimit(state.vy, desiredVy, Constants.Swerve.MAX_ACCELERATION, dt);
        state.omega = applyAccelLimit(state.omega, desiredOmega, Constants.Swerve.MAX_ANGULAR_ACCELERATION, dt);

        // Calculate module states using inverse kinematics
        calculateModuleStates(state);

        // Update robot pose
        updatePose(state, dt);

        // Apply field boundaries
        applyFieldBoundaries(state);
    }

    /**
     * Apply acceleration limit to smoothly change velocity.
     */
    private static double applyAccelLimit(double current, double desired, double maxAccel, double dt) {
        double diff = desired - current;
        double maxChange = maxAccel * dt;

        if (Math.abs(diff) <= maxChange) {
            return desired;
        }
        return current + Math.signum(diff) * maxChange;
    }

    /**
     * Calculate individual module angles and speeds from chassis speeds.
     * This is the inverse kinematics calculation.
     */
    private static void calculateModuleStates(RobotState state) {
        for (int i = 0; i < 4; i++) {
            double modX = MODULE_POSITIONS[i][0];
            double modY = MODULE_POSITIONS[i][1];

            // Module velocity = robot velocity + rotation contribution
            double moduleVx = state.vx - state.omega * modY;
            double moduleVy = state.vy + state.omega * modX;

            // Calculate speed and angle
            double speed = Math.hypot(moduleVx, moduleVy);
            double angle = Math.atan2(moduleVy, moduleVx);

            // Optimize angle (take shortest path, possibly reversing direction)
            double currentAngle = state.moduleAngles[i];
            double angleDiff = normalizeAngle(angle - currentAngle);

            if (Math.abs(angleDiff) > Math.PI / 2) {
                // Reverse direction is shorter
                angle = normalizeAngle(angle + Math.PI);
                speed = -speed;
            }

            state.moduleAngles[i] = angle;
            state.moduleSpeeds[i] = speed;
        }
    }

    /**
     * Update robot pose based on current velocities.
     */
    private static void updatePose(RobotState state, double dt) {
        // Simple Euler integration
        state.x += state.vx * dt;
        state.y += state.vy * dt;
        state.heading += state.omega * dt;

        // Normalize heading to [-PI, PI]
        state.heading = normalizeAngle(state.heading);
    }

    /**
     * Keep robot within field boundaries.
     */
    private static void applyFieldBoundaries(RobotState state) {
        double halfRobot = Constants.Robot.LENGTH_WITH_BUMPERS / 2;

        // X boundaries
        if (state.x < halfRobot) {
            state.x = halfRobot;
            state.vx = 0;
        } else if (state.x > Constants.Field.LENGTH - halfRobot) {
            state.x = Constants.Field.LENGTH - halfRobot;
            state.vx = 0;
        }

        // Y boundaries
        if (state.y < halfRobot) {
            state.y = halfRobot;
            state.vy = 0;
        } else if (state.y > Constants.Field.WIDTH - halfRobot) {
            state.y = Constants.Field.WIDTH - halfRobot;
            state.vy = 0;
        }
    }

    /**
     * Normalize angle to [-PI, PI].
     */
    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Reset gyro to specified angle.
     */
    public static void resetGyro(RobotState state, double angleDegrees) {
        state.heading = Math.toRadians(angleDegrees);
    }
}
