import commands2


class Claw(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Claw")

    def intake(self):
        """Run the intake motors at a constant power, pulling CORAL into the claw."""

    def outtake(self):
        """Run the intake motors at a constant power, pushing CORAL out of the claw."""

    def stop(self):
        """Stop running the intake motors."""

    def has_possession(self) -> bool:
        """Return whether the claw is currently holding CORAL."""
