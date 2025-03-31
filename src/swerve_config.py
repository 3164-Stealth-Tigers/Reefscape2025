import math

from pathplannerlib.path import PathConstraints
from wpimath.geometry import Translation2d, Rotation2d

from constants import DrivingConstants
from swervepy import u, TrajectoryFollowerParameters
from swervepy.impl import TypicalDriveComponentParameters, NeutralMode, TypicalAzimuthComponentParameters, Pigeon2Gyro, \
    CoaxialSwerveModule, NEOCoaxialDriveComponent, NEOCoaxialAzimuthComponent, AbsoluteCANCoder

TRACK_WIDTH = (17.75 * u.inch).m_as(u.m)
WHEEL_BASE = (29.75 * u.inch).m_as(u.m)
MAX_VELOCITY = 4.2 * (u.m / u.s)#0.5 * (u.m / u.s)
MAX_ANGULAR_VELOCITY = 9.547 * (u.rad / u.s)  #500 * (u.deg / u.s)#200 * (u.deg / u.s)

FL_ENCODER_OFFSET = 19.072266 + 180  # degrees
FR_ENCODER_OFFSET = 269.208984 - 180
RL_ENCODER_OFFSET = 244.863281 - 180
RR_ENCODER_OFFSET = 217.529297 - 180

DRIVE_PARAMS = TypicalDriveComponentParameters(
    wheel_circumference=4 * math.pi * u.inch,
    gear_ratio=6.75 / 1,  # SDS Mk4i L2
    max_speed=MAX_VELOCITY,
    open_loop_ramp_rate=0.25,
    closed_loop_ramp_rate=0,
    continuous_current_limit=60,
    peak_current_limit=80,
    peak_current_duration=0.01,
    neutral_mode=NeutralMode.COAST,
    kP=0.064395,
    kI=0,
    kD=0,
    kS=0.18656,
    kV=2.5833,
    kA=0.40138,
    invert_motor=False,
)
AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
    gear_ratio=150 / 7,  # SDS Mk4i
    max_angular_velocity=MAX_ANGULAR_VELOCITY,
    ramp_rate=0,
    continuous_current_limit=30,
    peak_current_limit=40,
    peak_current_duration=0.01,
    neutral_mode=NeutralMode.BRAKE,
    kP=0.01,
    kI=0,
    kD=0,
    invert_motor=True,
)
GYRO = Pigeon2Gyro(0, False)

SWERVE_MODULES = (
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(7, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(8, Rotation2d.fromDegrees(FL_ENCODER_OFFSET), AZIMUTH_PARAMS, AbsoluteCANCoder(1)),
        Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    ),
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(5, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(6, Rotation2d.fromDegrees(FR_ENCODER_OFFSET), AZIMUTH_PARAMS, AbsoluteCANCoder(2)),
        Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    ),
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(3, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(4, Rotation2d.fromDegrees(RL_ENCODER_OFFSET), AZIMUTH_PARAMS, AbsoluteCANCoder(3)),
        Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    ),
    CoaxialSwerveModule(
        NEOCoaxialDriveComponent(1, DRIVE_PARAMS),
        NEOCoaxialAzimuthComponent(2, Rotation2d.fromDegrees(RR_ENCODER_OFFSET), AZIMUTH_PARAMS, AbsoluteCANCoder(4)),
        Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    ),
)

AUTONOMOUS_PARAMS = TrajectoryFollowerParameters(theta_kP=4, xy_kP=1, drive_open_loop=DrivingConstants.OPEN_LOOP)  # P: 4 or 0.7
PATHFINDING_CONSTRAINTS = PathConstraints(0.5, 20, 200, 1000, 12)#PathConstraints(1.5, 20, 100, 1000, 12)
