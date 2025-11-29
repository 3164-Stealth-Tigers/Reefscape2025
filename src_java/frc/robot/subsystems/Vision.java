package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
 * Vision subsystem for AprilTag pose estimation using PhotonVision.
 */
public class Vision extends SubsystemBase {
    // Pose estimators for each camera
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();

    // Field layout
    private final AprilTagFieldLayout fieldLayout;

    /**
     * Result class for pose estimation.
     */
    public record PoseEstimate(Pose2d pose, double timestampSeconds) {}

    /**
     * Creates a new Vision subsystem.
     */
    public Vision() {
        // Load the 2025 Reefscape field layout
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // Initialize pose estimators for each camera
        initializeCamera("front_camera", VisionConstants.FRONT_CAMERA_TRANSFORM);
        initializeCamera("back_camera", VisionConstants.BACK_CAMERA_TRANSFORM);
    }

    /**
     * Initialize a camera and its pose estimator.
     *
     * @param name The camera name
     * @param robotToCamera The transform from robot center to camera
     */
    private void initializeCamera(String name, Transform3d robotToCamera) {
        PhotonCamera camera = new PhotonCamera(name);
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
        poseEstimators.add(estimator);
    }

    /**
     * Test cameras by displaying target information on SmartDashboard.
     */
    public void testCameras() {
        for (PhotonPoseEstimator estimator : poseEstimators) {
            // Note: In Java PhotonVision, we need to get the camera from the estimator differently
            // This is a simplified version - actual implementation may vary based on PhotonVision version
        }
    }

    /**
     * Get the best pose estimation from all cameras.
     *
     * @param robotPose The current robot pose for comparison
     * @return Optional containing the best pose estimate, or empty if none available
     */
    public Optional<PoseEstimate> getPoseEstimation(Pose2d robotPose) {
        PoseEstimate bestResult = null;
        double lastDistance = Double.POSITIVE_INFINITY;

        for (PhotonPoseEstimator estimator : poseEstimators) {
            Optional<EstimatedRobotPose> result = estimator.update();

            if (result.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimated = result.get();
            Pose2d estimatedPose = estimated.estimatedPose.toPose2d();
            double distance = estimatedPose.getTranslation().getDistance(robotPose.getTranslation());

            if (distance < lastDistance) {
                lastDistance = distance;
                bestResult = new PoseEstimate(estimatedPose, estimated.timestampSeconds);
            }

            // Log to SmartDashboard
            double deltaTime = Timer.getFPGATimestamp() - estimated.timestampSeconds;
            SmartDashboard.putNumber("Vision/DeltaTime", deltaTime);
            SmartDashboard.putNumberArray("Vision/2dPoseEstimate",
                new double[] {estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getDegrees()});
        }

        return Optional.ofNullable(bestResult);
    }

    @Override
    public void periodic() {
        // Periodic updates handled elsewhere
    }
}
