"""The OI (Operator Interface) module contains `Action Sets` and `Control Schemes`.

An `Action Set` defines the inputs a controller should have. For example, a driver's controller needs to have an input
for driving forwards and backwards. These inputs can be either functions that return a value such as a float or bool,
or they can be Commands buttons. A button can be bound to run a command when pressed, held, etc.

A `Control Scheme` implements an `Action Set` and defines which physical buttons or joysticks on a controller perform
each action. For example, on an Xbox controller, the left joystick might be bound to forwards and backwards movement.
Any special logic (e.g. inverting the Y axis on a joystick) is also defined in a `Control Scheme`.
"""

from typing import Protocol
from abc import abstractmethod, ABC

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


# TODO: Map controller buttons & functions
class OperatorActionSet(Protocol):
    # Elevator button functions
    @property
    @abstractmethod
    def loading_level(self) -> Trigger:
        # Approach Loading Level
        # ???
        raise NotImplementedError

    @property
    @abstractmethod
    def level_1(self) -> Trigger:
        # Approach level 1
        # A?
        raise NotImplementedError

    @property
    @abstractmethod
    def level_2(self) -> Trigger:
        # Approach level 2
        # X?
        raise NotImplementedError

    @property
    @abstractmethod
    def level_3(self) -> Trigger:
        # Approach level 3
        # B?
        raise NotImplementedError

    @property
    @abstractmethod
    def level_4(self) -> Trigger:
        # Approach level 4
        # Y?
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
        # Approach level 1
        # A?
        return self.stick.a()

    @property
    def level_2(self) -> Trigger:
        # Approach level 2
        # X?
        return self.stick.x()

    @property
    def level_3(self) -> Trigger:
        # Approach level 3
        # B?
        return self.stick.b()

    @property
    def level_4(self) -> Trigger:
        # Approach level 4
        # Y?
        return self.stick.y()


class XboxDriver(DriverActionSet):
    """Drive the robot with an Xbox controller"""

    def __init__(self, port: int):
        """Construct an XboxDriver

        :param port: The port that the joystick is plugged into. Reported on the Driver Station
        """
        self.stick = CommandXboxController(port)

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
        return self.stick.start()

    @property
    def toggle_field_relative(self) -> Trigger:
        return self.stick.back()

    @property
    def ski_stop(self) -> Trigger:
        return self.stick.y()

    def is_movement_commanded(self):
        return self.forward() + self.strafe() + self.turn() != 0


class XBoxDriver(DriverActionSet):
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


class T16000M(DriverActionSet):
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
        return deadband(-self.stick.getRawAxis(2), 0.1) * 0.6

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


def deadband(value, band):
    return value if abs(value) > band else 0
