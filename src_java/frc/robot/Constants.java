package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/*
 * ============================================================================
 * CONSTANTS.JAVA - The Robot's Configuration File
 * ============================================================================
 *
 * WHAT THIS FILE DOES:
 * This file contains ALL the numbers that control how the robot behaves.
 * Think of it like a settings menu for the entire robot!
 *
 * WHY IT'S IMPORTANT:
 * Instead of hunting through code to change a speed or height, you come here.
 * All the "magic numbers" are in one place, organized by robot part.
 *
 * HOW TO USE THIS FILE:
 * 1. Find the section for what you want to change (Elevator, Arm, Swerve, etc.)
 * 2. Read the comments to understand what each value does
 * 3. Change the value and redeploy to the robot
 * 4. Test carefully - small changes can have big effects!
 *
 * UNITS WARNING:
 * - Most values are in METERS and RADIANS (SI units)
 * - Use Units.inchesToMeters() to convert inches to meters
 * - Use Rotation2d.fromDegrees() to convert degrees to radians
 *
 * ============================================================================
 * QUICK REFERENCE - Common Things to Change
 * ============================================================================
 *
 * CHANGE DRIVE SPEED:
 *   → SwerveConstants.MAX_SPEED (line ~203)
 *
 * CHANGE SCORING HEIGHTS:
 *   → ScoringConstants.L1_HEIGHT through L4_HEIGHT (lines ~270-274)
 *
 * CHANGE ARM ANGLES:
 *   → ScoringConstants.L1_ANGLE through L4_ANGLE (lines ~277-281)
 *
 * CHANGE ELEVATOR SPEED:
 *   → ElevatorConstants.MAX_VELOCITY (line ~51)
 *
 * ============================================================================
 */
public final class Constants {

    // ========================================================================
    // SECTION: DRIVING BEHAVIOR
    // ========================================================================
    // These settings control how the robot responds to driver input.
    // ========================================================================

    /**
     * Settings that affect how the robot drives during teleop and auto.
     */
    public static final class DrivingConstants {
        /**
         * OPEN_LOOP: Whether to use "open loop" or "closed loop" driving.
         *
         * - true (Open Loop): Motor runs at a percentage. Simpler, good for teleop.
         * - false (Closed Loop): Motor uses encoder to maintain exact speed. More precise.
         *
         * Most teams use open loop for teleop driving.
         */
        public static final boolean OPEN_LOOP = true;

        /**
         * SPEED MULTIPLIERS: Scale the joystick input.
         *
         * NORMAL_SPEED_MULTIPLIER: Full speed (1.0 = 100%)
         * SLOW_SPEED_MULTIPLIER: Slow mode for precise movements (0.3 = 30%)
         *
         * HOW TO CHANGE: If robot feels too fast, reduce NORMAL to 0.8 or 0.7
         */
        public static final double NORMAL_SPEED_MULTIPLIER = 1.0;
        public static final double SLOW_SPEED_MULTIPLIER = 0.3;
    }

    // ========================================================================
    // SECTION: ELEVATOR
    // ========================================================================
    // The elevator lifts game pieces to different scoring levels.
    // It uses two motors working together (leader/follower configuration).
    // ========================================================================

    /**
     * Elevator subsystem configuration.
     *
     * HARDWARE:
     * - 2 NEO motors (IDs 9 and 10)
     * - Hall effect sensor for homing (knows when at bottom)
     * - Sprocket and chain system to lift carriage
     */
    public static final class ElevatorConstants {
        // --------------------------------------------------------------------
        // MOTOR CAN IDs - Don't change unless you rewire the robot!
        // --------------------------------------------------------------------
        public static final int LEFT_MOTOR_ID = 9;   // Left elevator motor
        public static final int RIGHT_MOTOR_ID = 10; // Right elevator motor

        // --------------------------------------------------------------------
        // MOTOR DIRECTION - Which way the motors spin
        // --------------------------------------------------------------------
        // If elevator goes wrong direction, flip these values
        public static final boolean INVERT_LEFT_MOTOR = false;
        public static final boolean INVERT_RIGHT_MOTOR = true;  // Opposite side, so inverted

        // --------------------------------------------------------------------
        // LIMIT SWITCH - Detects when elevator is at the bottom
        // --------------------------------------------------------------------
        public static final int LOWER_LIMIT_SWITCH_ID = 0;  // Digital IO port 0

        // --------------------------------------------------------------------
        // PHYSICAL MEASUREMENTS - Must match the actual robot!
        // --------------------------------------------------------------------
        /**
         * GEAR_RATIO: Motor rotations needed for 1 sprocket rotation.
         * 12:1 means motor spins 12 times for sprocket to spin once.
         */
        public static final double GEAR_RATIO = 12.0;

        /**
         * SPROCKET_PITCH_DIAMETER: Size of the sprocket that drives the chain.
         * This determines how much the elevator moves per motor rotation.
         * Measured in METERS (converted from 1.751 inches).
         */
        public static final double SPROCKET_PITCH_DIAMETER = Units.inchesToMeters(1.751);

        /**
         * CARRIAGE_MASS: Weight of the carriage in kilograms.
         * Used for feedforward calculations to fight gravity.
         */
        public static final double CARRIAGE_MASS = Units.lbsToKilograms(15.0);

        // --------------------------------------------------------------------
        // HEIGHT LIMITS - Prevents crashing into frame!
        // --------------------------------------------------------------------
        /**
         * MINIMUM/MAXIMUM heights the carriage can travel.
         * These are SAFETY LIMITS - the code won't go past these.
         *
         * WARNING: Changing these incorrectly can damage the robot!
         */
        public static final double MINIMUM_CARRIAGE_HEIGHT = Units.inchesToMeters(30.5);
        public static final double MAXIMUM_CARRIAGE_HEIGHT = Units.inchesToMeters(78.5);

        /**
         * LIMIT_SWITCH_HEIGHT: Where the limit switch triggers (for homing).
         */
        public static final double LIMIT_SWITCH_HEIGHT = Units.inchesToMeters(30.0);

        // --------------------------------------------------------------------
        // MOTION CONTROL - How fast and smooth the elevator moves
        // --------------------------------------------------------------------
        /**
         * HEIGHT_TOLERANCE: How close is "close enough" to the target.
         * 0.5 inches means if we're within 0.5" of target, we're done.
         */
        public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(0.5);

        /**
         * MAX_VELOCITY: Top speed of elevator motion (meters per second).
         * Higher = faster but more aggressive.
         */
        public static final double MAX_VELOCITY = Units.inchesToMeters(150.0);

        /**
         * MAX_ACCELERATION: How quickly the elevator speeds up/slows down.
         * Higher = snappier but more jerky.
         */
        public static final double MAX_ACCELERATION = Units.inchesToMeters(200.0);

        // --------------------------------------------------------------------
        // PID CONTROL - Tuning for position accuracy
        // --------------------------------------------------------------------
        // PID = Proportional, Integral, Derivative control
        // These values tell the motor how hard to work to reach the target
        //
        // kP (Proportional): Main tuning knob. Higher = more aggressive
        // kD (Derivative): Dampening. Helps prevent overshoot
        // kI (Integral): Usually 0 for elevators
        // --------------------------------------------------------------------
        public static final double POSITION_kP = 20.0;  // Position control gain
        public static final double POSITION_kD = 0.0;   // Usually leave at 0
        public static final double VELOCITY_kP = 0.3;   // Velocity control gain

        // --------------------------------------------------------------------
        // OUTPUT LIMITS - Prevents motor from going too fast
        // --------------------------------------------------------------------
        public static final double MAX_OUT_UP = 1.0;    // Full power up allowed
        public static final double MAX_OUT_DOWN = -0.5; // Limited power down (gravity helps)

        // --------------------------------------------------------------------
        // FEEDFORWARD - Compensates for gravity and friction
        // --------------------------------------------------------------------
        // [kS, kG, kV, kA] - Don't change unless you run SysId characterization
        public static final double[] FEEDFORWARD_CONSTANTS = {0.0, 0.5, 10.0, 0.0};
    }

    // ========================================================================
    // SECTION: CLAW (Game Piece Intake)
    // ========================================================================
    // The claw grabs and releases CORAL game pieces.
    // ========================================================================

    /**
     * Claw subsystem configuration.
     *
     * HARDWARE:
     * - 1 NEO 550 motor
     * - Time-of-Flight sensor for piece detection
     */
    public static final class ClawConstants {
        public static final int MOTOR_ID = 11;           // Motor CAN ID
        public static final int TOF_SENSOR_ID = 1;       // Time-of-Flight sensor ID

        /**
         * CURRENT_LIMIT_AMPS: Maximum current draw to protect motor.
         */
        public static final double CURRENT_LIMIT_AMPS = 20.0;

        /**
         * THRESHOLD_CURRENT: If motor draws more than this, we probably have a piece.
         * Used for automatic piece detection.
         */
        public static final double THRESHOLD_CURRENT = 15.0;
    }

    // ========================================================================
    // SECTION: CORAL ARM
    // ========================================================================
    // The arm rotates to position game pieces for scoring.
    // ========================================================================

    /**
     * Coral arm subsystem configuration.
     *
     * HARDWARE:
     * - 1 NEO Vortex motor with 125:1 gearbox
     * - Absolute encoder (always knows position, even after restart)
     */
    public static final class CoralArmConstants {
        public static final int MOTOR_ID = 12;  // Motor CAN ID

        // --------------------------------------------------------------------
        // PHYSICAL MEASUREMENTS
        // --------------------------------------------------------------------
        public static final double GEAR_RATIO = 125.0;  // 125:1 reduction
        public static final double ARM_LENGTH = Units.inchesToMeters(18.0);
        public static final double ARM_MASS = Units.lbsToKilograms(5.0);

        /**
         * ENCODER_OFFSET: Calibration value for the absolute encoder.
         * This tells the code what "zero" looks like.
         *
         * HOW TO CALIBRATE:
         * 1. Position arm horizontally (0 degrees)
         * 2. Read the raw encoder value
         * 3. Set this constant to that value
         */
        public static final double ENCODER_OFFSET = 252.0;

        // --------------------------------------------------------------------
        // ANGLE LIMITS - Prevents arm from hitting things
        // --------------------------------------------------------------------
        public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-50.0);
        public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(90.0);

        // --------------------------------------------------------------------
        // CONTROL PARAMETERS
        // --------------------------------------------------------------------
        public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(3.0);
        public static final double kP = 0.02;  // Proportional gain

        // Feedforward constants [kS, kG, kV, kA]
        public static final double[] FEEDFORWARD_CONSTANTS = {0.0, 0.5, 1.0, 0.0};
    }

    // ========================================================================
    // SECTION: ALGAE ARM (Secondary Arm)
    // ========================================================================
    // A smaller arm for handling ALGAE game pieces.
    // ========================================================================

    /**
     * Algae arm subsystem configuration.
     */
    public static final class AlgaeArmConstants {
        public static final int MOTOR_ID = 15;

        public static final double GEAR_RATIO = 45.0;
        public static final double ARM_LENGTH = Units.inchesToMeters(12.0);
        public static final double ARM_MASS = Units.lbsToKilograms(3.0);

        public static final double ENCODER_OFFSET = 0.0;

        public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-30.0);
        public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(120.0);

        public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(5.0);
        public static final double kP = 0.015;
    }

    // ========================================================================
    // SECTION: CLIMBER
    // ========================================================================
    // The climber lifts the robot during end-game.
    // ========================================================================

    /**
     * Climber subsystem configuration.
     *
     * HARDWARE:
     * - 2 NEO motors in leader/follower mode
     * - Winch system with rope/cable
     */
    public static final class ClimberConstants {
        public static final int LEADER_MOTOR_ID = 14;
        public static final int FOLLOWER_MOTOR_ID = 13;

        public static final double GEAR_RATIO = 64.0;
        public static final double SPOOL_DIAMETER = Units.inchesToMeters(1.0);

        // Position limits (how far the climber can extend)
        public static final double MINIMUM_POSITION = 0.0;
        public static final double MAXIMUM_POSITION = Units.inchesToMeters(24.0);

        public static final double kP = 0.1;
        public static final double CLIMB_SPEED = 1.0;  // Full power for climbing
    }

    // ========================================================================
    // SECTION: VISION (Cameras)
    // ========================================================================
    // Camera system for detecting AprilTags and positioning.
    // ========================================================================

    /**
     * Vision subsystem configuration.
     *
     * HARDWARE:
     * - Front camera (looking forward)
     * - Back camera (looking backward)
     */
    public static final class VisionConstants {
        // Camera names (must match what's in PhotonVision)
        public static final String[] CAMERA_NAMES = {"front_camera", "back_camera"};

        /**
         * Camera positions relative to robot center.
         * Transform3d(x, y, z, rotation)
         * - x: forward/back from center (positive = forward)
         * - y: left/right from center (positive = left)
         * - z: up/down from ground (positive = up)
         */
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(24.0)),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)  // Tilted down 15°
        );

        public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(24.0)),
            new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(180.0))  // Facing backward
        );
    }

    // ========================================================================
    // SECTION: FIELD MEASUREMENTS
    // ========================================================================
    // Dimensions of the game field and important positions.
    // ========================================================================

    /**
     * Field geometry constants for 2025 REEFSCAPE.
     */
    public static final class FieldConstants {
        // Field size (standard FRC field)
        public static final double FIELD_LENGTH = Units.feetToMeters(54.0);
        public static final double FIELD_WIDTH = Units.feetToMeters(27.0);

        // Reef structure position (the hexagonal scoring structure)
        public static final double REEF_CENTER_X = FIELD_LENGTH / 2.0;
        public static final double REEF_CENTER_Y = FIELD_WIDTH / 2.0;
        public static final double REEF_RADIUS = Units.inchesToMeters(36.0);
    }

    // ========================================================================
    // SECTION: ROBOT DIMENSIONS
    // ========================================================================
    // Physical size of the robot (needed for collision detection).
    // ========================================================================

    /**
     * Robot physical dimensions.
     *
     * IMPORTANT: Update these if the frame changes!
     */
    public static final class RobotPhysicalConstants {
        public static final double ROBOT_LENGTH = Units.inchesToMeters(29.75);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(29.75);
        public static final double BUMPER_THICKNESS = Units.inchesToMeters(3.5);

        // Total size including bumpers
        public static final double ROBOT_LENGTH_WITH_BUMPERS = ROBOT_LENGTH + 2 * BUMPER_THICKNESS;
        public static final double ROBOT_WIDTH_WITH_BUMPERS = ROBOT_WIDTH + 2 * BUMPER_THICKNESS;
    }

    // ========================================================================
    // SECTION: SWERVE DRIVE
    // ========================================================================
    // Configuration for the swerve drive system (the drivetrain).
    //
    // WHAT IS SWERVE?
    // Swerve drive has 4 wheel modules that can each spin AND rotate.
    // This lets the robot move in any direction without turning!
    // ========================================================================

    /**
     * Swerve drive configuration.
     *
     * HARDWARE (per module):
     * - Drive motor: Spins the wheel (makes robot move)
     * - Azimuth motor: Rotates the wheel (steers direction)
     * - CANCoder: Absolute encoder for wheel angle
     *
     * We have 4 modules: Front-Left (FL), Front-Right (FR),
     * Rear-Left (RL), Rear-Right (RR)
     */
    public static final class SwerveConstants {
        // --------------------------------------------------------------------
        // CHASSIS DIMENSIONS - Distance between wheels
        // --------------------------------------------------------------------
        public static final double TRACK_WIDTH = Units.inchesToMeters(17.75);  // Left to right
        public static final double WHEEL_BASE = Units.inchesToMeters(29.75);   // Front to back

        // --------------------------------------------------------------------
        // SPEED LIMITS
        // --------------------------------------------------------------------
        /**
         * MAX_SPEED: Maximum driving speed in meters per second.
         * 4.2 m/s is about 14 feet per second (pretty fast!)
         *
         * HOW TO CHANGE: Reduce for slower practice, increase for competition
         */
        public static final double MAX_SPEED = 4.2;

        /**
         * MAX_ANGULAR_VELOCITY: Maximum rotation speed in radians per second.
         */
        public static final double MAX_ANGULAR_VELOCITY = 9.547;

        // --------------------------------------------------------------------
        // WHEEL PROPERTIES
        // --------------------------------------------------------------------
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /**
         * GEAR RATIOS: Motor spins : Wheel spins
         * - DRIVE: 6.75:1 means motor spins 6.75 times per wheel rotation
         * - AZIMUTH: 21.4:1 (150/7) for steering
         *
         * These values are for SDS Mk4i L2 modules. Don't change unless
         * you have different modules!
         */
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

        // --------------------------------------------------------------------
        // MOTOR CAN IDs - Which port each motor is plugged into
        // --------------------------------------------------------------------
        // Front Left Module
        public static final int FL_DRIVE_ID = 7;
        public static final int FL_AZIMUTH_ID = 8;

        // Front Right Module
        public static final int FR_DRIVE_ID = 5;
        public static final int FR_AZIMUTH_ID = 6;

        // Rear Left Module
        public static final int RL_DRIVE_ID = 3;
        public static final int RL_AZIMUTH_ID = 4;

        // Rear Right Module
        public static final int RR_DRIVE_ID = 1;
        public static final int RR_AZIMUTH_ID = 2;

        // --------------------------------------------------------------------
        // CANCODER IDs - Absolute encoders for wheel angle
        // --------------------------------------------------------------------
        public static final int FL_CANCODER_ID = 1;
        public static final int FR_CANCODER_ID = 2;
        public static final int RL_CANCODER_ID = 3;
        public static final int RR_CANCODER_ID = 4;

        // --------------------------------------------------------------------
        // GYRO - Tells the robot which way it's facing
        // --------------------------------------------------------------------
        public static final int PIGEON_ID = 0;

        // --------------------------------------------------------------------
        // ENCODER OFFSETS - Calibration for wheel angles
        // --------------------------------------------------------------------
        // These values tell the robot what "forward" looks like for each wheel.
        //
        // HOW TO CALIBRATE:
        // 1. Put robot on blocks (wheels off ground)
        // 2. Rotate each wheel so it points forward
        // 3. Read the CANCoder value in Phoenix Tuner
        // 4. Put that value here (may need to add/subtract 180)
        // --------------------------------------------------------------------
        public static final double FL_ENCODER_OFFSET = 19.072266 + 180.0;
        public static final double FR_ENCODER_OFFSET = 269.208984 - 180.0;
        public static final double RL_ENCODER_OFFSET = 244.863281 - 180.0;
        public static final double RR_ENCODER_OFFSET = 217.529297 - 180.0;

        // --------------------------------------------------------------------
        // DRIVE MOTOR PID - Tuning for wheel speed control
        // --------------------------------------------------------------------
        // These values were found using SysId characterization
        public static final double DRIVE_kP = 0.064395;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        // Feedforward values (physics-based compensation)
        public static final double DRIVE_kS = 0.18656;  // Static friction
        public static final double DRIVE_kV = 2.5833;   // Velocity factor
        public static final double DRIVE_kA = 0.40138;  // Acceleration factor

        // --------------------------------------------------------------------
        // AZIMUTH MOTOR PID - Tuning for wheel angle control
        // --------------------------------------------------------------------
        public static final double AZIMUTH_kP = 0.01;
        public static final double AZIMUTH_kI = 0.0;
        public static final double AZIMUTH_kD = 0.0;

        // --------------------------------------------------------------------
        // CURRENT LIMITS - Protects motors from overheating
        // --------------------------------------------------------------------
        public static final int DRIVE_CURRENT_LIMIT = 60;    // Amps
        public static final int AZIMUTH_CURRENT_LIMIT = 30;  // Amps

        // --------------------------------------------------------------------
        // RAMP RATES - How quickly motors speed up
        // --------------------------------------------------------------------
        // Open loop = teleop driving, Closed loop = auto
        public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;  // Seconds to full power
        public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0; // No ramp for precision

        // --------------------------------------------------------------------
        // AUTONOMOUS PATH FOLLOWING - PID for auto routines
        // --------------------------------------------------------------------
        public static final double AUTO_THETA_kP = 4.0;  // Rotation correction
        public static final double AUTO_XY_kP = 2.0;     // Position correction
    }

    // ========================================================================
    // SECTION: SCORING POSITIONS
    // ========================================================================
    // Heights and angles for each scoring level on the REEF.
    //
    // LEVEL GUIDE:
    // - Level 0: Loading position (picking up pieces)
    // - Level 1: Lowest scoring position (trough)
    // - Level 2: Second level
    // - Level 3: Third level
    // - Level 4: Highest scoring position (most points!)
    // ========================================================================

    /**
     * Scoring heights and arm angles for each level.
     *
     * HOW TO ADJUST:
     * 1. If scoring too high/low, adjust the HEIGHT value
     * 2. If piece doesn't release cleanly, adjust the ANGLE
     * 3. Make small changes (0.5-1 inch at a time)
     * 4. Test after each change!
     */
    public static final class ScoringConstants {
        // --------------------------------------------------------------------
        // HEIGHTS - How high the elevator goes for each level
        // --------------------------------------------------------------------
        public static final double LOADING_HEIGHT = Units.inchesToMeters(30.5);  // Level 0
        public static final double L1_HEIGHT = Units.inchesToMeters(31.0);       // Level 1
        public static final double L2_HEIGHT = Units.inchesToMeters(29.75);      // Level 2
        public static final double L3_HEIGHT = Units.inchesToMeters(45.25);      // Level 3
        public static final double L4_HEIGHT = Units.inchesToMeters(78.5);       // Level 4 (MAX)

        // --------------------------------------------------------------------
        // ARM ANGLES - How the arm tilts at each level
        // --------------------------------------------------------------------
        // Positive = arm up, Negative = arm down
        // 0 degrees = horizontal
        public static final Rotation2d LOADING_ANGLE = Rotation2d.fromDegrees(0.0);   // Horizontal
        public static final Rotation2d L1_ANGLE = Rotation2d.fromDegrees(-35.0);      // Tilted down
        public static final Rotation2d L2_ANGLE = Rotation2d.fromDegrees(-35.0);
        public static final Rotation2d L3_ANGLE = Rotation2d.fromDegrees(-35.0);
        public static final Rotation2d L4_ANGLE = Rotation2d.fromDegrees(-35.0);
    }

    // ========================================================================
    // SECTION: CONTROLLER PORTS
    // ========================================================================
    // Which USB port each controller is plugged into.
    // ========================================================================

    /**
     * Controller configuration.
     *
     * PORT ASSIGNMENT:
     * - Port 0: Driver controller (moves the robot)
     * - Port 1: Operator controller (controls mechanisms)
     * - Port 2: Button board (quick scoring positions)
     *
     * Check Driver Station to see which port each controller is on.
     */
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int BUTTON_BOARD_PORT = 2;

        /**
         * JOYSTICK_DEADBAND: Ignores small joystick movements.
         * 0.1 means values from -0.1 to 0.1 are treated as 0.
         * This prevents drift when joysticks don't center perfectly.
         */
        public static final double JOYSTICK_DEADBAND = 0.1;
    }
}
