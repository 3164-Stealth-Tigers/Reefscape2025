package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the robot. All units are in SI (meters, radians, seconds) unless otherwise noted.
 * Conversion is done at definition time using Units utility methods.
 */
public final class Constants {

    /**
     * Driving-related constants for teleop and autonomous control.
     */
    public static final class DrivingConstants {
        public static final boolean OPEN_LOOP = true;
        public static final double NORMAL_SPEED_MULTIPLIER = 1.0;
        public static final double SLOW_SPEED_MULTIPLIER = 0.3;
    }

    /**
     * Elevator subsystem constants.
     */
    public static final class ElevatorConstants {
        // Motor CAN IDs
        public static final int LEFT_MOTOR_ID = 9;
        public static final int RIGHT_MOTOR_ID = 10;

        // Motor inversion settings
        public static final boolean INVERT_LEFT_MOTOR = false;
        public static final boolean INVERT_RIGHT_MOTOR = true;

        // Digital IO
        public static final int LOWER_LIMIT_SWITCH_ID = 0;

        // Physical characteristics
        public static final double GEAR_RATIO = 12.0;  // 12:1 reduction
        public static final double SPROCKET_PITCH_DIAMETER = Units.inchesToMeters(1.751);
        public static final double CARRIAGE_MASS = Units.lbsToKilograms(15.0);  // kg

        // Height limits (meters)
        public static final double MINIMUM_CARRIAGE_HEIGHT = Units.inchesToMeters(30.5);
        public static final double MAXIMUM_CARRIAGE_HEIGHT = Units.inchesToMeters(78.5);
        public static final double LIMIT_SWITCH_HEIGHT = Units.inchesToMeters(30.0);

        // Control parameters
        public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(0.5);
        public static final double MAX_VELOCITY = Units.inchesToMeters(150.0);  // m/s
        public static final double MAX_ACCELERATION = Units.inchesToMeters(200.0);  // m/s^2

        // PID constants
        public static final double POSITION_kP = 20.0;
        public static final double POSITION_kD = 0.0;
        public static final double VELOCITY_kP = 0.3;

        // Output limits
        public static final double MAX_OUT_UP = 1.0;
        public static final double MAX_OUT_DOWN = -0.5;

        // Feedforward constants (kS, kG, kV, kA)
        public static final double[] FEEDFORWARD_CONSTANTS = {0.0, 0.5, 10.0, 0.0};
    }

    /**
     * Claw subsystem constants.
     */
    public static final class ClawConstants {
        public static final int MOTOR_ID = 11;
        public static final int TOF_SENSOR_ID = 1;
        public static final double CURRENT_LIMIT_AMPS = 20.0;
        public static final double THRESHOLD_CURRENT = 15.0;
    }

    /**
     * Coral arm subsystem constants.
     */
    public static final class CoralArmConstants {
        public static final int MOTOR_ID = 12;

        // Physical characteristics
        public static final double GEAR_RATIO = 125.0;
        public static final double ARM_LENGTH = Units.inchesToMeters(18.0);
        public static final double ARM_MASS = Units.lbsToKilograms(5.0);

        // Encoder offset (degrees) - for absolute encoder zeroing
        public static final double ENCODER_OFFSET = 252.0;

        // Angle limits
        public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-50.0);
        public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(90.0);

        // Control parameters
        public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(3.0);
        public static final double kP = 0.02;

        // Feedforward constants (kS, kG, kV, kA)
        public static final double[] FEEDFORWARD_CONSTANTS = {0.0, 0.5, 1.0, 0.0};
    }

    /**
     * Algae arm subsystem constants.
     */
    public static final class AlgaeArmConstants {
        public static final int MOTOR_ID = 15;

        // Physical characteristics
        public static final double GEAR_RATIO = 45.0;
        public static final double ARM_LENGTH = Units.inchesToMeters(12.0);
        public static final double ARM_MASS = Units.lbsToKilograms(3.0);

        // Encoder offset (degrees)
        public static final double ENCODER_OFFSET = 0.0;

        // Angle limits
        public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-30.0);
        public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(120.0);

        // Control parameters
        public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(5.0);
        public static final double kP = 0.015;
    }

    /**
     * Climber subsystem constants.
     */
    public static final class ClimberConstants {
        public static final int LEADER_MOTOR_ID = 14;
        public static final int FOLLOWER_MOTOR_ID = 13;

        // Physical characteristics
        public static final double GEAR_RATIO = 64.0;
        public static final double SPOOL_DIAMETER = Units.inchesToMeters(1.0);

        // Position limits (meters of rope)
        public static final double MINIMUM_POSITION = 0.0;
        public static final double MAXIMUM_POSITION = Units.inchesToMeters(24.0);

        // Control parameters
        public static final double kP = 0.1;
        public static final double CLIMB_SPEED = 1.0;
    }

    /**
     * Vision subsystem constants.
     */
    public static final class VisionConstants {
        // Camera name to robot-to-camera transform mapping
        // Transform3d represents the position and orientation of each camera relative to robot center
        public static final String[] CAMERA_NAMES = {"front_camera", "back_camera"};

        // Front camera transform (x forward, y left, z up from robot center)
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(24.0)),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)  // Tilted down 15 degrees
        );

        // Back camera transform
        public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(24.0)),
            new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(180.0))  // Facing backward
        );
    }

    /**
     * Field-related constants.
     */
    public static final class FieldConstants {
        // Field dimensions (2025 Reefscape)
        public static final double FIELD_LENGTH = Units.feetToMeters(54.0);
        public static final double FIELD_WIDTH = Units.feetToMeters(27.0);

        // Reef structure position (center of field)
        public static final double REEF_CENTER_X = FIELD_LENGTH / 2.0;
        public static final double REEF_CENTER_Y = FIELD_WIDTH / 2.0;
        public static final double REEF_RADIUS = Units.inchesToMeters(36.0);
    }

    /**
     * Robot physical constants.
     */
    public static final class RobotPhysicalConstants {
        public static final double ROBOT_LENGTH = Units.inchesToMeters(29.75);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(29.75);
        public static final double BUMPER_THICKNESS = Units.inchesToMeters(3.5);

        // Total dimensions including bumpers
        public static final double ROBOT_LENGTH_WITH_BUMPERS = ROBOT_LENGTH + 2 * BUMPER_THICKNESS;
        public static final double ROBOT_WIDTH_WITH_BUMPERS = ROBOT_WIDTH + 2 * BUMPER_THICKNESS;
    }

    /**
     * Swerve drive constants.
     */
    public static final class SwerveConstants {
        // Chassis dimensions
        public static final double TRACK_WIDTH = Units.inchesToMeters(17.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(29.75);

        // Speed limits
        public static final double MAX_SPEED = 4.2;  // m/s
        public static final double MAX_ANGULAR_VELOCITY = 9.547;  // rad/s

        // Wheel properties
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // Gear ratios (SDS Mk4i L2)
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

        // Motor CAN IDs
        public static final int FL_DRIVE_ID = 7;
        public static final int FL_AZIMUTH_ID = 8;
        public static final int FR_DRIVE_ID = 5;
        public static final int FR_AZIMUTH_ID = 6;
        public static final int RL_DRIVE_ID = 3;
        public static final int RL_AZIMUTH_ID = 4;
        public static final int RR_DRIVE_ID = 1;
        public static final int RR_AZIMUTH_ID = 2;

        // CANCoder IDs
        public static final int FL_CANCODER_ID = 1;
        public static final int FR_CANCODER_ID = 2;
        public static final int RL_CANCODER_ID = 3;
        public static final int RR_CANCODER_ID = 4;

        // Gyro
        public static final int PIGEON_ID = 0;

        // Encoder offsets (degrees)
        public static final double FL_ENCODER_OFFSET = 19.072266 + 180.0;
        public static final double FR_ENCODER_OFFSET = 269.208984 - 180.0;
        public static final double RL_ENCODER_OFFSET = 244.863281 - 180.0;
        public static final double RR_ENCODER_OFFSET = 217.529297 - 180.0;

        // Drive motor PID
        public static final double DRIVE_kP = 0.064395;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;
        public static final double DRIVE_kS = 0.18656;
        public static final double DRIVE_kV = 2.5833;
        public static final double DRIVE_kA = 0.40138;

        // Azimuth motor PID
        public static final double AZIMUTH_kP = 0.01;
        public static final double AZIMUTH_kI = 0.0;
        public static final double AZIMUTH_kD = 0.0;

        // Current limits
        public static final int DRIVE_CURRENT_LIMIT = 60;
        public static final int AZIMUTH_CURRENT_LIMIT = 30;

        // Ramp rates
        public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
        public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;

        // Autonomous path following
        public static final double AUTO_THETA_kP = 4.0;
        public static final double AUTO_XY_kP = 2.0;
    }

    /**
     * Scoring position heights and angles for each level.
     */
    public static final class ScoringConstants {
        // Level heights (meters)
        public static final double LOADING_HEIGHT = Units.inchesToMeters(30.5);
        public static final double L1_HEIGHT = Units.inchesToMeters(31.0);
        public static final double L2_HEIGHT = Units.inchesToMeters(29.75);
        public static final double L3_HEIGHT = Units.inchesToMeters(45.25);
        public static final double L4_HEIGHT = Units.inchesToMeters(78.5);

        // Arm angles for each level
        public static final Rotation2d LOADING_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d L1_ANGLE = Rotation2d.fromDegrees(-35.0);
        public static final Rotation2d L2_ANGLE = Rotation2d.fromDegrees(-35.0);
        public static final Rotation2d L3_ANGLE = Rotation2d.fromDegrees(-35.0);
        public static final Rotation2d L4_ANGLE = Rotation2d.fromDegrees(-35.0);
    }

    /**
     * Operator interface constants (controller ports and deadbands).
     */
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int BUTTON_BOARD_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.1;
    }
}
