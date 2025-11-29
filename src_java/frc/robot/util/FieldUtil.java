package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

/**
 * Utility class for field-related calculations and alliance flipping.
 */
public final class FieldUtil {

    private FieldUtil() {
        // Utility class - prevent instantiation
    }

    // ==================== Alliance Flipping ====================

    /**
     * Flip a translation across the field based on alliance color.
     * Red alliance mirrors the position to the opposite side.
     *
     * @param translation The translation to flip
     * @return The flipped translation
     */
    public static Translation2d flipAlliance(Translation2d translation) {
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        if (shouldFlip) {
            return new Translation2d(
                FieldConstants.FIELD_LENGTH - translation.getX(),
                FieldConstants.FIELD_WIDTH - translation.getY()
            );
        }
        return translation;
    }

    /**
     * Flip a rotation across the field based on alliance color.
     *
     * @param rotation The rotation to flip
     * @return The flipped rotation
     */
    public static Rotation2d flipAlliance(Rotation2d rotation) {
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        if (shouldFlip) {
            return new Rotation2d(-rotation.getCos(), -rotation.getSin());
        }
        return rotation;
    }

    /**
     * Flip a pose across the field based on alliance color.
     *
     * @param pose The pose to flip
     * @return The flipped pose
     */
    public static Pose2d flipAlliance(Pose2d pose) {
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        if (shouldFlip) {
            return new Pose2d(
                new Translation2d(
                    FieldConstants.FIELD_LENGTH - pose.getX(),
                    FieldConstants.FIELD_WIDTH - pose.getY()
                ),
                new Rotation2d(-pose.getRotation().getCos(), -pose.getRotation().getSin())
            );
        }
        return pose;
    }

    // ==================== Reef Positions ====================

    // Reef transformation data: [pipe offset direction, rotation degrees]
    // Pipe offset direction: 1 = left, -1 = right
    private static final double[][] REEF_TRANSFORMATIONS = {
        {1, 0},     // A
        {-1, 0},    // B
        {1, 60},    // C
        {-1, 60},   // D
        {1, 120},   // E
        {-1, 120},  // F
        {1, 180},   // G
        {-1, 180},  // H
        {1, 240},   // I
        {-1, 240},  // J
        {1, 300},   // K
        {-1, 300}   // L
    };

    private static final double REEF_INSCRIBED_DIAMETER = 1.0;  // meters - placeholder
    private static final double REEF_PIPE_TO_PIPE_DISTANCE = 0.3;  // meters - placeholder

    /**
     * Get the translation of a reef pipe position.
     *
     * @param position The position label (A-L)
     * @return The translation of the pipe center
     */
    public static Translation2d getReefPipeTranslation(String position) {
        int index = position.toUpperCase().charAt(0) - 'A';
        if (index < 0 || index > 11) {
            throw new IllegalArgumentException("Position must be A-L, got: " + position);
        }

        double[] transform = REEF_TRANSFORMATIONS[index];
        double pipeOffset = transform[0];
        double rotationDeg = transform[1];

        // Calculate reef center
        Translation2d reefCenter = new Translation2d(FieldConstants.REEF_CENTER_X, FieldConstants.REEF_CENTER_Y);

        // Start with pipe translation relative to reef center
        Translation2d translation = reefCenter.plus(new Translation2d(
            -REEF_INSCRIBED_DIAMETER / 2,
            REEF_PIPE_TO_PIPE_DISTANCE / 2 * pipeOffset
        ));

        // Rotate around reef center
        translation = translation.minus(reefCenter);
        translation = translation.rotateBy(Rotation2d.fromDegrees(rotationDeg));
        translation = translation.plus(reefCenter);

        // Flip for alliance
        return flipAlliance(translation);
    }

    /**
     * Get the rotation for a reef position.
     *
     * @param position The position label (A-L)
     * @return The rotation in degrees
     */
    public static double getReefRotation(String position) {
        int index = position.toUpperCase().charAt(0) - 'A';
        if (index < 0 || index > 11) {
            throw new IllegalArgumentException("Position must be A-L, got: " + position);
        }
        return REEF_TRANSFORMATIONS[index][1];
    }

    // ==================== Coral Stations ====================

    /**
     * Enum representing the coral loading stations.
     */
    public enum CoralStation {
        LEFT(new Pose2d(0.5, FieldConstants.FIELD_WIDTH - 1.0, Rotation2d.fromDegrees(135))),
        RIGHT(new Pose2d(0.5, 1.0, Rotation2d.fromDegrees(-135)));

        private final Pose2d pose;

        CoralStation(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }
}
