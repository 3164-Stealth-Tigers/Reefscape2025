import commands2


class Arm(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Arm")

    """All these floats/ints are running as percents until i get full values o7"""
    @staticmethod
    def extend_retract(self) -> int:
        """Push claw out. Refers to claw length not movement."""
        """Pull claw in."""
        self.extension = 0
        """change extension to equal the extension of arm. Return this value for data"""
        return self.extension

    @staticmethod
    def movement_x(self) -> tuple[float, float]:
        """Move claw left(-x)"""
        """Move claw right(x)"""
        movementMaxX = 1
        movementMinX = -1
        testFloatX = 0
        '''Change test float to get a movement value. the value is a percent of how far it can move.
         0 is center,  anything (-) is left, anything (+) is right'''
        self.movementX = max(movementMinX, min(testFloatX, movementMaxX))
        return testFloatX, self.movementX

    @staticmethod
    def movement_y(self) -> tuple[float, float]:
        """Move claw down(-y)"""
        """Move claw up(y)"""
        movementMaxY = 1
        movementMinY = -1
        testFloatY = 0
        self.movementY = max(movementMinY, min(testFloatY, movementMaxY))
        return testFloatY, self.movementY

    def stop(self):
        """Stop when activated by control switch"""
