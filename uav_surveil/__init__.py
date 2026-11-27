"""Persistent and resilient UAV-swarm surveillance simulator."""

from .params import Parameters
from .simulation import Simulation

__all__ = ["Parameters", "Simulation", "make_simulation"]


def make_simulation(params: Parameters) -> Simulation:
    """Run Choset decomposition with round-robin route assignment."""
    return Simulation(params)
