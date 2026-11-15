"""Persistent and resilient UAV-swarm surveillance simulator."""

from .params import Parameters
from .patrol import LoopSimulation
from .simulation import Simulation

__all__ = ["LoopSimulation", "Parameters", "Simulation", "make_simulation"]


def make_simulation(params: Parameters) -> Simulation | LoopSimulation:
    """Simulation for the patrol mode of ``params``."""
    if params.patrol.mode == "shared_loop":
        return LoopSimulation(params)
    return Simulation(params)
