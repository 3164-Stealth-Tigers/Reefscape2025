import commands2


class Climber(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Climber")

    def Climb_Up_1(self):
        """Run the level 1 elevator motors at a constant power, rolling the elevator and the arm up."""

    def Climb_Down_1(self):
        """Run the level 1 elevator motors at a constant power reverse, rolling the elevator and the arm down."""

    def Climb_Up_2(self):
        """Run the level 2 elevator motors at a constant power, rolling the elevator and the arm up."""

    def Climb_Down_2(self):
        """Run the level 2 elevator motors at a constant power reverse, rolling the elevator and the arm down."""

    def Climb_Up_3(self):
        """Run the level 3 elevator motors at a constant power, rolling the elevator and the arm up."""

    def Climb_Down_3(self):
        """Run the level 3 elevator motors at a constant power reverse, rolling the elevator and the arm up."""

    def stop(self):
        """Stop running the elevator motors."""

    def full_climb_1(self) -> bool:
        """Return whether the level 1 elevator is currently fully extended."""

    if full_climb_1() == yes:
        """make climb_up_1 not work if level 1 elevator is fully extended"""

    def full_climb_2(self) -> bool:
        """Return whether the level 2 elevator is currently fully extended."""

    if full_climb_2() == yes:
        """Make climb_up_2 not work if level 2 elevator is fully extended."""

    def full_climb_3(self) -> bool:
        """Return whether the level 3 elevator is currently fully extended."""

    if full_climb_3() == yes:
        """Make climb_up_3 not work if level 3 elevator is fully extended."""

    def level_of_climb_1(self) -> string:
        """Return how far up the level 1 motors have run."""

    def level_of_climb_2(self) -> string:
        """Return how far up the level 2 motors have run."""

    def level_of_climb_3(self) -> string:
        """Return how far up the level 3 motors have run."""
