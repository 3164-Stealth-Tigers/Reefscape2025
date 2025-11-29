package frc.robot.subsystems;

/*
 * ========================================================================
 * VISION SUBSYSTEM - Robot Localization using AprilTags
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Uses cameras to see AprilTags on the field and figure out where the
 * robot is. This is like having GPS for the robot!
 *
 * WHAT ARE APRILTAGS?
 * -------------------
 * AprilTags are special 2D barcodes placed around the FRC field.
 * Each tag has a unique ID and known position. By seeing a tag and
 * knowing where it is on the field, we can calculate where WE are.
 *
 *   AprilTag example:
 *   ┌─────────────┐
 *   │ ▓░▓░▓░▓░▓░▓ │
 *   │ ░▓░▓░▓░▓░▓░ │
 *   │ ▓░   ID   ░▓ │
 *   │ ░▓░▓░▓░▓░▓░ │
 *   │ ▓░▓░▓░▓░▓░▓ │
 *   └─────────────┘
 *
 * HOW POSE ESTIMATION WORKS:
 * --------------------------
 *   1. Camera sees an AprilTag
 *   2. PhotonVision (running on a coprocessor) detects it
 *   3. Using the tag's known field position + how it looks in the image,
 *      we calculate where the camera must be
 *   4. Knowing camera position on robot, we calculate robot position
 *
 *      AprilTag                    Camera
 *      ┌───┐                      [   ]
 *      │ # │  ←─ distance ─→      [===]
 *      └───┘                      Robot
 *      (known position)           (calculated position)
 *
 * MULTI-TAG PNP:
 * --------------
 * When we see MULTIPLE AprilTags at once, we can get a more accurate
 * position estimate. "PNP" stands for Perspective-n-Point - a fancy
 * way of saying "use multiple points to solve the 3D position."
 *
 * PHOTONVISION:
 * -------------
 * PhotonVision is vision software that runs on a coprocessor (like
 * a Raspberry Pi or Orange Pi). It does all the image processing
 * and sends results to the roboRIO over NetworkTables.
 *
 * HOW TO MODIFY:
 * --------------
 * - Add more cameras: Call initializeCamera() with new name/transform
 * - Change camera position: Update VisionConstants.FRONT_CAMERA_TRANSFORM
 * - Change field layout: Already loads 2025 field automatically
 *
 * QUICK REFERENCE:
 * ----------------
 * → Get robot pose estimate: vision.getPoseEstimation(currentPose)
 * → Test cameras: vision.testCameras()
 *
 * ========================================================================
 */

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// PhotonVision imports
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// WPILib imports
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * ========================================================================
 * VISION SUBSYSTEM
 * ========================================================================
 *
 * Uses PhotonVision and AprilTags to estimate robot position on the field.
 * Supports multiple cameras for better coverage and accuracy.
 *
 * [WHY VISION?]
 * Odometry (wheel-based tracking) drifts over time. Vision provides
 * absolute position fixes to correct the drift.
 */
public class Vision extends SubsystemBase {

    // ========================================================================
    // POSE ESTIMATORS - One per camera
    // ========================================================================

    /**
     * List of pose estimators (one for each camera).
     *
     * [WHY A LIST?]
     * We can have multiple cameras (front, back, sides) for better field coverage.
     * Each camera has its own pose estimator that produces position estimates.
     */
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();

    // ========================================================================
    // FIELD LAYOUT - Where AprilTags are located
    // ========================================================================

    /**
     * The AprilTag field layout for the current game.
     *
     * [WHAT THIS CONTAINS]
     * - Position of every AprilTag on the field
     * - Each tag's ID and 3D pose
     *
     * WPILib provides official layouts for each year's game.
     */
    private final AprilTagFieldLayout fieldLayout;

    // ========================================================================
    // DATA STRUCTURES
    // ========================================================================

    /**
     * Result class for pose estimation.
     *
     * [JAVA RECORD]
     * A "record" is a compact way to create a simple data class.
     * This automatically creates a constructor, getters, equals, hashCode, etc.
     *
     * @param pose The estimated robot pose (X, Y, rotation)
     * @param timestampSeconds When this estimate was calculated (FPGA timestamp)
     */
    public record PoseEstimate(Pose2d pose, double timestampSeconds) {}

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new Vision subsystem.
     *
     * [WHAT HAPPENS HERE]
     * 1. Load the official 2025 field layout (AprilTag positions)
     * 2. Initialize each camera with its pose estimator
     */
    public Vision() {
        // Load the 2025 Reefscape field layout
        // This file contains the position of every AprilTag on the field
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // Initialize pose estimators for each camera
        // Each camera needs:
        //   - A name (must match PhotonVision configuration)
        //   - A transform (where the camera is on the robot)
        initializeCamera("front_camera", VisionConstants.FRONT_CAMERA_TRANSFORM);
        initializeCamera("back_camera", VisionConstants.BACK_CAMERA_TRANSFORM);
    }

    // ========================================================================
    // CAMERA INITIALIZATION
    // ========================================================================

    /**
     * Initialize a camera and its pose estimator.
     *
     * [THE TRANSFORM]
     * robotToCamera tells us where the camera is mounted on the robot:
     *   - X, Y, Z offset from robot center
     *   - Roll, pitch, yaw angles of the camera
     *
     *   Robot Center    Camera
     *       ○ ─────────→ [□]
     *           transform
     *
     * [POSE STRATEGY]
     * MULTI_TAG_PNP_ON_COPROCESSOR:
     *   - Uses multiple tags when visible (more accurate)
     *   - PNP calculations run on the coprocessor (Raspberry Pi)
     *   - Less CPU load on roboRIO
     *
     * @param name The camera name (must match PhotonVision config)
     * @param robotToCamera The 3D transform from robot center to camera
     */
    private void initializeCamera(String name, Transform3d robotToCamera) {
        // Create the PhotonCamera object (connects via NetworkTables)
        PhotonCamera camera = new PhotonCamera(name);

        // Create a pose estimator for this camera
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(
            fieldLayout,                              // Where tags are on the field
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Use multi-tag when possible
            robotToCamera                              // Where camera is on robot
        );

        // Add to our list of estimators
        poseEstimators.add(estimator);
    }

    // ========================================================================
    // TESTING
    // ========================================================================

    /**
     * Test cameras by displaying target information on SmartDashboard.
     *
     * [WHEN TO USE]
     * Called in test mode to verify cameras are working and seeing tags.
     */
    public void testCameras() {
        for (PhotonPoseEstimator estimator : poseEstimators) {
            // Note: Actual implementation would display target info
            // This varies based on PhotonVision version
        }
    }

    // ========================================================================
    // POSE ESTIMATION
    // ========================================================================

    /**
     * Get the best pose estimation from all cameras.
     *
     * [HOW IT WORKS]
     * 1. Ask each camera for a pose estimate
     * 2. Compare estimates to our current believed pose
     * 3. Return the estimate that's closest to where we think we are
     *
     * [WHY COMPARE TO CURRENT POSE?]
     * If a camera gives an estimate that's way off from where we think
     * we are, it's probably wrong (maybe saw a reflection or bad tag).
     * We trust estimates that agree with our odometry more.
     *
     * [THE TIMESTAMP]
     * Vision processing takes time. The timestamp tells us WHEN the image
     * was captured, so we can properly fuse it with odometry.
     *
     * @param robotPose The current robot pose (from odometry)
     * @return Optional containing the best pose estimate, or empty if no tags visible
     */
    public Optional<PoseEstimate> getPoseEstimation(Pose2d robotPose) {
        PoseEstimate bestResult = null;
        double lastDistance = Double.POSITIVE_INFINITY;

        // Check each camera's estimate
        for (PhotonPoseEstimator estimator : poseEstimators) {
            // Get the latest estimate (may be empty if no tags visible)
            Optional<EstimatedRobotPose> result = estimator.update();

            // Skip if this camera didn't see any tags
            if (result.isEmpty()) {
                continue;
            }

            // Get the estimate details
            EstimatedRobotPose estimated = result.get();
            Pose2d estimatedPose = estimated.estimatedPose.toPose2d();  // Convert 3D to 2D

            // Calculate how far this estimate is from our current pose
            double distance = estimatedPose.getTranslation().getDistance(robotPose.getTranslation());

            // Keep the estimate closest to our current pose
            if (distance < lastDistance) {
                lastDistance = distance;
                bestResult = new PoseEstimate(estimatedPose, estimated.timestampSeconds);
            }

            // Log to SmartDashboard for debugging
            double deltaTime = Timer.getFPGATimestamp() - estimated.timestampSeconds;
            SmartDashboard.putNumber("Vision/DeltaTime", deltaTime);  // How old is this estimate?
            SmartDashboard.putNumberArray("Vision/2dPoseEstimate",
                new double[] {estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getDegrees()});
        }

        // Return the best result (or empty if no estimates)
        return Optional.ofNullable(bestResult);
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================

    @Override
    public void periodic() {
        // Vision updates are handled by SwerveDrive when it calls getPoseEstimation()
        // We could add periodic logging here if needed
    }

}  // End of Vision class
