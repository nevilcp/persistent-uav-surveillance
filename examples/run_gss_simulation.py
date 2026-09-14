"""
Example: Running the GSS (Guidance, Surveillance, and Safety) Simulation

This script demonstrates how to initialize and run the complete UAV surveillance
simulation using the baseline configuration.
"""

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))

from uav_surveil.config import load_scenario  # noqa: E402
from uav_surveil.gss.simulation import GSSSimulation  # noqa: E402


def main():
    """Run a basic GSS simulation example."""

    print("=" * 60)
    print("🚁 UAV Surveillance GSS Simulation Demo")
    print("=" * 60)

    # Load baseline configuration
    print("📋 Loading baseline configuration...")
    config = load_scenario("baseline")
    print("   ✅ Loaded baseline scenario")

    # Create and initialize simulation
    print("\n🔧 Initializing GSS simulation...")
    simulation = GSSSimulation(config=config)

    # Run simulation for 5 minutes (300 seconds)
    print("\n🎮 Starting simulation...")
    success = simulation.run(duration=300.0)

    if success:
        print("\n📊 Final Results:")
        print(f"   • Total runtime: {simulation.metrics.total_runtime:.1f}s")
        print(f"   • Simulation time: {simulation.metrics.current_time:.0f}s")
        print(f"   • Coverage: {simulation.metrics.coverage_percentage:.1f}%")
        print(f"   • Cells overdue: {simulation.metrics.cells_overdue}")
        print(f"   • Active UAVs: {simulation.metrics.active_uavs}")
        print(f"   • Spare UAVs: {simulation.metrics.spare_uavs}")
        print(
            f"   • STL violations: C1={simulation.metrics.stl_c1_violations}, "
            f"C2={simulation.metrics.stl_c2_violations}"
        )
        print("\n🎉 Simulation completed successfully!")
    else:
        print("\n❌ Simulation failed or encountered errors.")
        print(f"   Final state: {simulation.state}")

    print("=" * 60)


if __name__ == "__main__":
    main()
