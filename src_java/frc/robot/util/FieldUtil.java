package frc.robot.util;

/*
 * ========================================================================
 * FIELD UTILITY - Field Geometry and Alliance Flipping
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Provides utility functions for field-related calculations:
 *   1. Alliance flipping - convert positions between Red and Blue
 *   2. Reef positions - calculate where to score on the REEF
 *   3. Coral station positions - where to collect game pieces
 *
 * WHY ALLIANCE FLIPPING?
 * ----------------------
 * The FRC field is symmetric. Code written for Blue alliance can work
 * for Red alliance by "flipping" coordinates to the other side.
 *
 *   BLUE ALLIANCE VIEW:              RED ALLIANCE VIEW:
 *   (Blue on left)                   (Red on left)
 *
 *   ┌─────────────────────┐          ┌─────────────────────┐
 *   │ B               R   │          │ B               R   │
 *   │ L               E   │          │ L               E   │
 *   │ U    REEF       D   │          │ U    REEF       D   │
 *   │ E               ●   │←Robot    │ ●               E   │←Robot
 *   └─────────────────────┘          └─────────────────────┘
 *          X=0 ──────→ X=max              X=max ←────── X=0
 *
 *   Same code works for both - just flip coordinates for Red!
 *
 * THE REEF STRUCTURE:
 * -------------------
 * The REEF is hexagonal with 12 scoring positions (A through L):
 *
 *              G   H
 *           F    ●    I      ← Each letter is a scoring pipe
 *          E    ───    J     ← Pipes are in pairs on each face
 *           D         K
 *              C   L
 *            B       A
 *
 * Each face of the hexagon has two pipes (left and right).
 * The transformation data defines offset direction and rotation.
 *
 * HOW TO MODIFY:
 * --------------
 * - Change reef dimensions: REEF_INSCRIBED_DIAMETER, REEF_PIPE_TO_PIPE_DISTANCE
 * - Add new positions: Add to REEF_TRANSFORMATIONS array
 * - Change coral stations: Modify CoralStation enum
 *
 * QUICK REFERENCE:
 * ----------------
 * → Flip for alliance: FieldUtil.flipAlliance(pose)
 * → Get reef position: FieldUtil.getReefPipeTranslation("A")
 * → Get reef rotation: FieldUtil.getReefRotation("A")
 *
 * ========================================================================
 */

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

/**
 * ========================================================================
 * FIELD UTILITY CLASS
 * ========================================================================
 *
 * Static utility methods for field geometry calculations.
 * This is a "utility class" - all methods are static, no instances needed.
 */
public final class FieldUtil {

    /**
     * Private constructor prevents instantiation.
     * Utility classes should never be instantiated.
     */
    private FieldUtil() {
        // Utility class - prevent instantiation
    }

    // ========================================================================
    // ALLIANCE FLIPPING
    // ========================================================================
    //
    // [WHY WE FLIP]
    // All our autonomous paths and positions are defined for Blue alliance.
    // When we're on Red alliance, we flip everything to the mirrored position.
    //
    // [THE MATH]
    // X: Flip across center of field → newX = FIELD_LENGTH - oldX
    // Y: Flip across center of field → newY = FIELD_WIDTH - oldY
    // Rotation: Flip direction → rotate 180° (negate cos and sin)
    //
    // ========================================================================

    /**
     * Flip a translation (X, Y position) for Red alliance.
     *
     * [WHEN TO USE]
     * Call this on any position that's defined for Blue alliance
     * before using it. If we're Blue, it returns unchanged.
     *
     * @param translation The position (Blue alliance origin)
     * @return The flipped position if Red, original if Blue
     */
    public static Translation2d flipAlliance(Translation2d translation) {
        // Check our alliance color (default to Blue if unknown)
        boolean shouldFlip = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        if (shouldFlip) {
            // Mirror position to opposite side of field
            return new Translation2d(
                FieldConstants.FIELD_LENGTH - translation.getX(),  // Flip X
                FieldConstants.FIELD_WIDTH - translation.getY()    // Flip Y
            );
        }
        return translation;  // Blue alliance - no change
    }

    /**
     * Flip a rotation for Red alliance.
     *
     * [THE MATH]
     * To rotate 180°, we negate both cos and sin components.
     * This effectively points the robot the opposite direction.
     *
     * @param rotation The rotation (Blue alliance reference)
     * @return The flipped rotation if Red, original if Blue
     */
    public static Rotation2d flipAlliance(Rotation2d rotation) {
        boolean shouldFlip = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        if (shouldFlip) {
            // Rotate 180° by negating cos and sin
            return new Rotation2d(-rotation.getCos(), -rotation.getSin());
        }
        return rotation;
    }

    /**
     * Flip a complete pose (position + rotation) for Red alliance.
     *
     * @param pose The pose (Blue alliance reference)
     * @return The flipped pose if Red, original if Blue
     */
    public static Pose2d flipAlliance(Pose2d pose) {
        boolean shouldFlip = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        if (shouldFlip) {
            return new Pose2d(
                // Flip position
                new Translation2d(
                    FieldConstants.FIELD_LENGTH - pose.getX(),
                    FieldConstants.FIELD_WIDTH - pose.getY()
                ),
                // Flip rotation
                new Rotation2d(-pose.getRotation().getCos(), -pose.getRotation().getSin())
            );
        }
        return pose;
    }

    // ========================================================================
    // REEF POSITIONS
    // ========================================================================
    //
    // The reef is hexagonal with 6 faces. Each face has 2 scoring pipes
    // (left and right), giving us 12 total positions: A through L.
    //
    // ========================================================================

    /**
     * Reef transformation data: [pipe offset direction, rotation degrees]
     *
     * [STRUCTURE]
     * Index 0 = Pipe offset: 1 = left pipe, -1 = right pipe
     * Index 1 = Face rotation in degrees (each face is 60° apart)
     *
     * [THE LAYOUT]
     * Position | Offset | Rotation | Face
     * ---------|--------|----------|------
     *    A     |   +1   |    0°    | Face 1
     *    B     |   -1   |    0°    | Face 1
     *    C     |   +1   |   60°    | Face 2
     *    D     |   -1   |   60°    | Face 2
     *   ...and so on around the hexagon
     */
    private static final double[][] REEF_TRANSFORMATIONS = {
        {1, 0},     // A - Face 1, left pipe
        {-1, 0},    // B - Face 1, right pipe
        {1, 60},    // C - Face 2, left pipe
        {-1, 60},   // D - Face 2, right pipe
        {1, 120},   // E - Face 3, left pipe
        {-1, 120},  // F - Face 3, right pipe
        {1, 180},   // G - Face 4, left pipe
        {-1, 180},  // H - Face 4, right pipe
        {1, 240},   // I - Face 5, left pipe
        {-1, 240},  // J - Face 5, right pipe
        {1, 300},   // K - Face 6, left pipe
        {-1, 300}   // L - Face 6, right pipe
    };

    /**
     * Distance from reef center to the face (inscribed circle diameter).
     * TODO: Update with actual 2025 REEFSCAPE measurements.
     */
    private static final double REEF_INSCRIBED_DIAMETER = 1.0;  // meters - placeholder

    /**
     * Distance between the two pipes on each face.
     * TODO: Update with actual 2025 REEFSCAPE measurements.
     */
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
