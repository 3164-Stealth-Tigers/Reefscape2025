import math
from typing import Optional

import photonlibpy as pv
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose2d

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

    def get_pose_estimation(self, robot_pose: Pose2d) -> Optional[Pose2d]:
        robot_translation = robot_pose.translation()
        closest_pose = None
        last_distance = math.inf

        for estimator in self.pose_estimators:
            result = estimator.update()
            if not result:
                continue

            estimated_pose = result.estimatedPose.toPose2d()
            distance = estimated_pose.translation().distance(robot_translation)
            if distance < last_distance:
                last_distance = distance
                closest_pose = estimated_pose

        return closest_pose
