import math
from typing import Optional

import photonlibpy as pv
import wpilib
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose2d, Pose3d

from constants import VisionConstants


class Vision:
    def __init__(self):
        field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        self.pose_estimators = []
        for camera_name, transform in VisionConstants.CAMERAS.items():
            camera = pv.PhotonCamera(camera_name)
            self.pose_estimators.append(
                pv.PhotonPoseEstimator(field_layout, pv.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                                       transform)
            )

    def test_cameras(self):
        for estimator in self.pose_estimators:
            camera = estimator._camera
            robot_to_camera = estimator.robotToCamera

            if not camera.isConnected():
                continue

            target = camera.getLatestResult().getBestTarget()
            if target:
                target_pose = Pose3d()
                camera_pose = target_pose.transformBy(target.getBestCameraToTarget().inverse())
                robot_pose = camera_pose.transformBy(robot_to_camera.inverse())

                wpilib.SmartDashboard.putNumberArray(f"Vision/{camera._name}/CameraPose",
                                                     [camera_pose.x, camera_pose.y, camera_pose.z])
                wpilib.SmartDashboard.putNumberArray(f"Vision/{camera._name}/CameraPoseInch",
                                                     [camera_pose.x * 39.37, camera_pose.y * 39.37, camera_pose.z * 39.37])
                wpilib.SmartDashboard.putNumberArray(f"Vision/{camera._name}/RobotPose",
                                                     [robot_pose.x, robot_pose.y, robot_pose.z])
                wpilib.SmartDashboard.putNumberArray(f"Vision/{camera._name}/RobotPoseInch",
                                                     [robot_pose.x * 39.37, robot_pose.y * 39.37, robot_pose.z * 39.37])

    def get_pose_estimation(self, robot_pose: Pose2d) -> Optional[tuple[Pose2d, float]]:
        robot_translation = robot_pose.translation()
        best_result = None
        last_distance = math.inf

        for estimator in self.pose_estimators:
            result = estimator.update()
            if not result:
                continue

            estimated_pose = result.estimatedPose.toPose2d()
            distance = estimated_pose.translation().distance(robot_translation)
            if distance < last_distance:
                last_distance = distance
                best_result = (estimated_pose, result.timestampSeconds)

            camera = estimator._camera
            wpilib.SmartDashboard.putNumber(f"Vision/{camera.getName()}/DeltaTime",
                                            wpilib.Timer.getFPGATimestamp() - result.timestampSeconds)
            wpilib.SmartDashboard.putNumberArray(f"Vision/{camera.getName()}/2dPoseEstimate",
                                                 [estimated_pose.x, estimated_pose.y, estimated_pose.rotation().degrees()])

        return best_result

