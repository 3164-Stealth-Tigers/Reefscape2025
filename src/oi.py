"""The OI (Operator Interface) module contains `Action Sets` and `Control Schemes`.

An `Action Set` defines the inputs a controller should have. For example, a driver's controller needs to have an input
for driving forwards and backwards. These inputs can be either functions that return a value such as a float or bool,
or they can be Commands buttons. A button can be bound to run a command when pressed, held, etc.

A `Control Scheme` implements an `Action Set` and defines which physical buttons or joysticks on a controller perform
each action. For example, on an Xbox controller, the left joystick might be bound to forwards and backwards movement.
Any special logic (e.g. inverting the Y axis on a joystick) is also defined in a `Control Scheme`.
"""

from typing import Protocol
from abc import abstractmethod

from commands2.button import CommandXboxController, CommandJoystick, Trigger, CommandPS4Controller


# Action Sets


class DriverActionSet(Protocol):
    @abstractmethod
    def forward(self) -> float:
        """Movement along the X axis, from -1 to 1"""
        raise NotImplementedError

    @abstractmethod
    def strafe(self) -> float:
        """Movement along the Y axis, from -1 to 1"""
        raise NotImplementedError

    @abstractmethod
    def turn(self) -> float:
        """Rotation around the Z axis, from -1 (clockwise) to 1 (counter-clockwise)"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reset_gyro(self) -> Trigger:
        raise NotImplementedError

    @property
    @abstractmethod
    def toggle_field_relative(self) -> Trigger:
        """Toggle field-relative control on or off"""
        raise NotImplementedError

    @property
    @abstractmethod
    def ski_stop(self) -> Trigger:
        """Turn the wheels to an 'X' shape"""
        raise NotImplementedError

    @abstractmethod
    def is_movement_commanded(self):
        """Return True if drive base movement is desired"""
        raise NotImplementedError


class OperatorActionSet(Protocol):
    # Elevator button functions
    @property
    @abstractmethod
    def loading_level(self) -> Trigger:
        """Approach Loading Level"""
        raise NotImplementedError

    @property
    @abstractmethod
    def level_1(self) -> Trigger:
        """Approach level 1"""
        raise NotImplementedError

    @property
    @abstractmethod
    def level_2(self) -> Trigger:
        """Approach level 2"""
        raise NotImplementedError

    @property
    @abstractmethod
    def level_3(self) -> Trigger:
        """Approach level 3"""
        raise NotImplementedError

    @property
    @abstractmethod
    def level_4(self) -> Trigger:
        """Approach level 4"""
        raise NotImplementedError

    @property
    @abstractmethod
    def climber_up(self) -> Trigger:
        raise

    @property
    @abstractmethod
    def climber_down(self) -> Trigger:
        raise NotImplementedError

    @property
    @abstractmethod
    def intake(self) -> Trigger:
        raise NotImplementedError

    @property
    @abstractmethod
    def outtake(self) -> Trigger:
        raise NotImplementedError

    @abstractmethod
    def elevator(self) -> float:
        """Manual control for elevator. Positive number moves elevator up, away from ground."""
        raise NotImplementedError

    @abstractmethod
    def coral_arm(self) -> float:
        """Manual control for coral arm. Positive number moves coral arm counterclockwise."""
        raise NotImplementedError


class ScoringPositionsActionSet(Protocol):
    # All of these positions are based on the view from robot sim (different sides of the hexagon)

    @property
    @abstractmethod
    def reef_c(self) -> Trigger:
        """Goes to the left, bottom left reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_d(self) -> Trigger:
        """Goes to the right, bottom left reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_e(self) -> Trigger:
        """Goes to the left, bottom right reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_f(self) -> Trigger:
        """Goes to the right, bottom right reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_g(self) -> Trigger:
        """Goes to the left, right reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_h(self) -> Trigger:
        """Goes to the right, right reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_i(self) -> Trigger:
        """Goes to the left, top right reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_j(self) -> Trigger:
        """Goes to the right, top right reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_k(self) -> Trigger:
        """Goes to the left, top left reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_l(self) -> Trigger:
        """Goes to the right, top left reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_a(self) -> Trigger:
        """Goes to the left, left reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reef_b(self) -> Trigger:
        """Goes to the right, left reef face scoring position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def station_left(self) -> Trigger:
        raise NotImplementedError

    @property
    @abstractmethod
    def station_right(self) -> Trigger:
        raise NotImplementedError



# Control schemes


class XboxOperator(OperatorActionSet):
    def __init__(self, port: int):
        self.stick = CommandXboxController(port)

    # Elevator button functions

    @property
    def loading_level(self) -> Trigger:
        return self.stick.rightTrigger()

    @property
    def level_1(self) -> Trigger:
        return self.stick.a()

    @property
    def level_2(self) -> Trigger:
        return self.stick.x()

    @property
    def level_3(self) -> Trigger:
        return self.stick.b()

    @property
    def level_4(self) -> Trigger:
        return self.stick.y()

    @property
    def climber_up(self) -> Trigger:
        return self.stick.povUp()

    @property
    def climber_down(self) -> Trigger:
        return self.stick.povDown()

    def elevator(self) -> float:
        return deadband(-self.stick.getLeftY(), 0.08)

    def coral_arm(self) -> float:
        return deadband(-self.stick.getRightY(), 0.08)

    @property
    def intake(self) -> Trigger:
        return self.stick.leftTrigger()

    @property
    def outtake(self) -> Trigger:
        return self.stick.leftBumper()


class XboxDriver(DriverActionSet):
    """Drive the robot with an Xbox controller"""

    def __init__(self, port: int):
        """Construct an XboxDriver

        :param port: The port that the joystick is plugged into. Reported on the Driver Station
        """
        self.stick = CommandXboxController(port)

    def forward(self) -> float:
        """The robot's movement along the X axis, controlled by moving the left joystick up and down. From -1 to 1"""
        return deadband(-self.stick.getLeftY(), 0.08) ** 2 * sgn(-self.stick.getLeftY())

    def strafe(self) -> float:
        """The robot's movement along the Y axis, controlled by moving the left joystick left and right. From -1 to 1"""
        return deadband(-self.stick.getLeftX(), 0.08) ** 2 * sgn(-self.stick.getLeftX())

    def turn(self) -> float:
        """The robot's movement around the Z axis, controlled by moving the right joystick left and right.
        From -1 to 1, CCW+
        """
        return (deadband(-self.stick.getRightX(), 0.08) * 0.7) ** 2 * sgn(-self.stick.getRightX())

    @property
    def reset_gyro(self) -> Trigger:
        return self.stick.start()

    @property
    def toggle_field_relative(self) -> Trigger:
        return self.stick.back()

    @property
    def ski_stop(self) -> Trigger:
        return self.stick.y()

    def is_movement_commanded(self):
        return self.forward() + self.strafe() + self.turn() != 0


class PS4Driver(DriverActionSet):
    """Drive the robot with an PS4 controller"""

    def __init__(self, port: int):
        """Construct a PS4Driver

        :param port: The port that the joystick is plugged into. Reported on the Driver Station
        """
        self.stick = CommandPS4Controller(port)

    def forward(self) -> float:
        """The robot's movement along the X axis, controlled by moving the left joystick up and down. From -1 to 1"""
        return deadband(-self.stick.getLeftY(), 0.08)

    def strafe(self) -> float:
        """The robot's movement along the Y axis, controlled by moving the left joystick left and right. From -1 to 1"""
        return deadband(-self.stick.getLeftX(), 0.08)

    def turn(self) -> float:
        """The robot's movement around the Z axis, controlled by moving the right joystick left and right.
        From -1 to 1, CCW+
        """
        return deadband(-self.stick.getRightX(), 0.08) * 0.6

    @property
    def reset_gyro(self) -> Trigger:
        return self.stick.options()

    @property
    def toggle_field_relative(self) -> Trigger:
        return self.stick.share()

    @property
    def ski_stop(self) -> Trigger:
        return self.stick.triangle()

    def is_movement_commanded(self):
        return self.forward() + self.strafe() + self.turn() != 0


class T16000MDriver(DriverActionSet):
    """Drive the robot with an T.16000M flight stick controller"""

    def __init__(self, port: int):
        """Construct a T.16000M flight stick

        :param port: The port that the joystick is plugged into. Reported on the Driver Station
        """
        self.stick = CommandJoystick(port)

    def forward(self) -> float:
        return deadband(-self.stick.getRawAxis(1), 0.001)

    def strafe(self) -> float:
        return deadband(-self.stick.getRawAxis(0), 0.001)

    def turn(self) -> float:
        return deadband(-self.stick.getRawAxis(2), 0.01) * 0.6

    @property
    def reset_gyro(self) -> Trigger:
        # Left-side button with single dot
        return self.stick.button(8)

    @property
    def toggle_field_relative(self) -> Trigger:
        # Left-side button with two dots
        return self.stick.button(9)

    @property
    def ski_stop(self) -> Trigger:
        return self.stick.trigger()

    def is_movement_commanded(self):
        return self.forward() + self.strafe() + self.turn() != 0


class KeyboardScoringPositions(ScoringPositionsActionSet):
    """
    Use a program on the Driver Station laptop to emulate an Xbox controller from a keyboard. That way, a keyboard
    may be used as a button board.
    """

    @property
    def station_left(self) -> Trigger:
        return Trigger()

    @property
    def station_right(self) -> Trigger:
        return Trigger()

    def __init__(self, port: int):
        self.stick = CommandXboxController(port)

    @property
    def reef_e(self) -> Trigger:
        return self.stick.x()

    @property
    def reef_f(self) -> Trigger:
        return self.stick.a()

    @property
    def reef_g(self) -> Trigger:
        return self.stick.b()

    @property
    def reef_h(self) -> Trigger:
        return self.stick.y()

    @property
    def reef_i(self) -> Trigger:
        return self.stick.rightTrigger()

    @property
    def reef_j(self) -> Trigger:
        return self.stick.rightBumper()

    @property
    def reef_k(self) -> Trigger:
        return self.stick.leftTrigger()

    @property
    def reef_l(self) -> Trigger:
        return self.stick.leftBumper()

    @property
    def reef_a(self) -> Trigger:
        return self.stick.povUp()

    @property
    def reef_b(self) -> Trigger:
        return self.stick.povRight()

    @property
    def reef_c(self) -> Trigger:
        return self.stick.povLeft()

    @property
    def reef_d(self) -> Trigger:
        return self.stick.povDown()


class ArcadeScoringPositions(ScoringPositionsActionSet):

    def __init__(self, port: int):
        self.stick = CommandJoystick(port)

    @property
    def station_right(self) -> Trigger:
        return self.stick.axisGreaterThan(0, 0.5)  # Right

    @property
    def station_left(self) -> Trigger:
        return self.stick.axisLessThan(0, -0.5)  # Left

    @property
    def reef_a(self) -> Trigger:
        return self.stick.button(1)  # K1

    @property
    def reef_b(self) -> Trigger:
        return self.stick.button(2)  # K2

    @property
    def reef_c(self) -> Trigger:
        return self.stick.button(3)  # K3

    @property
    def reef_d(self) -> Trigger:
        return self.stick.button(4)  # K4

    @property
    def reef_e(self) -> Trigger:
        return self.stick.button(5)  # L1

    @property
    def reef_f(self) -> Trigger:
        return self.stick.button(6)  # R1

    @property
    def reef_g(self) -> Trigger:
        return self.stick.button(7)  # L2

    @property
    def reef_h(self) -> Trigger:
        return self.stick.button(8)  # R2

    @property
    def reef_i(self) -> Trigger:
        return self.stick.button(9)  # SE

    @property
    def reef_j(self) -> Trigger:
        return self.stick.button(10)  # ST

    @property
    def reef_k(self) -> Trigger:
        return self.stick.button(11)  # K12

    @property
    def reef_l(self) -> Trigger:
        return self.stick.button(12)  # K12


def deadband(value, band):
    return value if abs(value) > band else 0

def sgn(x):
    return 1 if x > 0 else -1
