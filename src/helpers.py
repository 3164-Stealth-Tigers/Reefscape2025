class SimHelper:
    currents: list[tuple[float, float]] = []

    @classmethod
    def add_simulated_current_load(cls, now: float, amps: float) -> None:
        """
        Add a current load to the simulation.

        :param now: The FPGA timestamp.
        :param amps: The current (in amps).
        """

        cls.currents.append((now, amps))

    @classmethod
    def clear_simulated_currents(cls) -> None:
        """Empty the list of simulated current loads."""

        cls.currents.clear()
