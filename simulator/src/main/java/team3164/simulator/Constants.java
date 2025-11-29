package team3164.simulator;

/**
 * Simulation constants mirroring the robot's actual configuration.
 * Values taken from src_java/frc/robot/Constants.java
 */
public final class Constants {

    private Constants() {}

    // ========================================================================
    // FIELD DIMENSIONS (meters)
    // ========================================================================
    public static final class Field {
        public static final double LENGTH = 16.4592;  // 54 feet
        public static final double WIDTH = 8.2296;    // 27 feet

        // Reef structure (hexagonal, center of field)
        public static final double REEF_CENTER_X = LENGTH / 2.0;
        public static final double REEF_CENTER_Y = WIDTH / 2.0;
        public static final double REEF_RADIUS = 0.9144;  // 36 inches

        // Coral stations
        public static final double STATION_X = 0.5;
        public static final double STATION_LEFT_Y = WIDTH - 1.0;
        public static final double STATION_RIGHT_Y = 1.0;
    }

    // ========================================================================
    // ROBOT DIMENSIONS (meters)
    // ========================================================================
    public static final class Robot {
        public static final double LENGTH = 0.7556;      // 29.75 inches
        public static final double WIDTH = 0.7556;       // 29.75 inches
        public static final double BUMPER_THICKNESS = 0.0889;  // 3.5 inches

        public static final double LENGTH_WITH_BUMPERS = LENGTH + 2 * BUMPER_THICKNESS;
        public static final double WIDTH_WITH_BUMPERS = WIDTH + 2 * BUMPER_THICKNESS;
    }

    // ========================================================================
    // SWERVE DRIVE
    // ========================================================================
    public static final class Swerve {
        public static final double TRACK_WIDTH = 0.4509;   // 17.75 inches
        public static final double WHEEL_BASE = 0.7556;    // 29.75 inches

        public static final double MAX_SPEED = 4.2;              // m/s
        public static final double MAX_ANGULAR_VELOCITY = 9.547; // rad/s

        // Acceleration limits for simulation
        public static final double MAX_ACCELERATION = 8.0;         // m/s^2
        public static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2
    }

    // ========================================================================
    // ELEVATOR
    // ========================================================================
    public static final class Elevator {
        public static final double MIN_HEIGHT = 0.7747;   // 30.5 inches
        public static final double MAX_HEIGHT = 1.9939;   // 78.5 inches

        public static final double MAX_VELOCITY = 3.81;        // 150 inches/s
        public static final double MAX_ACCELERATION = 5.08;    // 200 inches/s^2

        public static final double TOLERANCE = 0.0127;    // 0.5 inches
    }

    // ========================================================================
    // CORAL ARM
    // ========================================================================
    public static final class Arm {
        public static final double MIN_ANGLE = Math.toRadians(-50);
        public static final double MAX_ANGLE = Math.toRadians(90);

        public static final double MAX_VELOCITY = Math.toRadians(180);      // deg/s
        public static final double MAX_ACCELERATION = Math.toRadians(360);  // deg/s^2

        public static final double TOLERANCE = Math.toRadians(3);
        public static final double LENGTH = 0.4572;  // 18 inches
    }

    // ========================================================================
    // CLAW
    // ========================================================================
    public static final class Claw {
        public static final double INTAKE_TIME = 0.5;   // seconds to intake coral
        public static final double OUTTAKE_TIME = 0.3;  // seconds to outtake coral
    }

    // ========================================================================
    // CLIMBER
    // ========================================================================
    public static final class Climber {
        public static final double MIN_POSITION = 0.0;
        public static final double MAX_POSITION = 0.6096;  // 24 inches

        public static final double MAX_VELOCITY = 0.3;     // m/s
    }

    // ========================================================================
    // SCORING POSITIONS
    // ========================================================================
    public static final class Scoring {
        // Heights in meters
        public static final double LOADING_HEIGHT = 0.7747;  // 30.5 inches
        public static final double L1_HEIGHT = 0.7874;       // 31 inches
        public static final double L2_HEIGHT = 0.7556;       // 29.75 inches
        public static final double L3_HEIGHT = 1.1494;       // 45.25 inches
        public static final double L4_HEIGHT = 1.9939;       // 78.5 inches

        // Angles in radians
        public static final double LOADING_ANGLE = 0.0;
        public static final double L1_ANGLE = Math.toRadians(-35);
        public static final double L2_ANGLE = Math.toRadians(-35);
        public static final double L3_ANGLE = Math.toRadians(-35);
        public static final double L4_ANGLE = Math.toRadians(-35);
    }

    // ========================================================================
    // SIMULATION
    // ========================================================================
    public static final class Simulation {
        public static final double TICK_RATE = 50.0;         // Hz (20ms per tick)
        public static final double DT = 1.0 / TICK_RATE;     // seconds per tick
        public static final int BROADCAST_RATE = 30;         // Hz for WebSocket updates
    }
}
