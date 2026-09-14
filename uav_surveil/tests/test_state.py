"""Tests for GSS State model."""

import pytest

from ..core.cell import Cell
from ..core.route import Route
from ..core.uav import UAV
from ..gss.state import State


class TestState:
    """Test GSS State model functionality."""

    def test_empty_state_creation(self):
        """Test creating empty state with defaults."""
        state = State()

        assert state.clock == 0
        assert state.uavs == []
        assert state.cells == []
        assert state.spare_count == 0
        assert state.need_route_update is False
        assert state.need_spare is False
        assert state.total_coverage_gaps == 0
        assert state.max_revisit_gap == 0.0

    def test_state_with_single_uav(self):
        """Test state instantiation with one UAV."""
        # Create a test route
        test_route = Route(id="route_1", cell_sequence=["cell_1", "cell_2"])

        # Create a test UAV
        uav = UAV(id="uav_001", x=100.0, y=200.0, soc=0.8, route_list=[test_route])

        state = State(uavs=[uav], spare_count=2)

        assert len(state.uavs) == 1
        assert state.uavs[0].id == "uav_001"
        assert state.spare_count == 2
        assert state.get_total_fleet_size() == 3  # 1 active + 2 spare

    def test_unique_uav_ids_validation(self):
        """Test that duplicate UAV IDs are rejected."""
        route1 = Route(id="r1", cell_sequence=["c1"])
        route2 = Route(id="r2", cell_sequence=["c2"])

        uav1 = UAV(id="uav_001", x=0, y=0, soc=0.8, route_list=[route1])
        uav2 = UAV(
            id="uav_001", x=10, y=10, soc=0.7, route_list=[route2]
        )  # Duplicate ID

        with pytest.raises(ValueError, match="UAV IDs must be unique"):
            State(uavs=[uav1, uav2])

    def test_unique_cell_ids_validation(self):
        """Test that duplicate cell IDs are rejected."""
        cell1 = Cell(id="cell_001", x=0, y=0)
        cell2 = Cell(id="cell_001", x=40, y=0)  # Duplicate ID

        with pytest.raises(ValueError, match="Cell IDs must be unique"):
            State(cells=[cell1, cell2])

    def test_clock_update_validation(self):
        """Test clock update with time validation."""
        state = State(clock=100)

        # Forward time update should work
        state.update_clock(150)
        assert state.clock == 150

        # Backward time should raise error
        with pytest.raises(ValueError, match="Time cannot go backwards"):
            state.update_clock(50)

    def test_get_active_uavs(self):
        """Test filtering active UAVs."""
        route = Route(id="r1", cell_sequence=["c1"])

        uav1 = UAV(id="uav_001", x=0, y=0, soc=0.8, route_list=[route], is_active=True)
        uav2 = UAV(
            id="uav_002", x=10, y=10, soc=0.7, route_list=[route], is_active=False
        )
        uav3 = UAV(
            id="uav_003", x=20, y=20, soc=0.9, route_list=[route], is_active=True
        )

        state = State(uavs=[uav1, uav2, uav3])
        active_uavs = state.get_active_uavs()

        assert len(active_uavs) == 2
        assert active_uavs[0].id == "uav_001"
        assert active_uavs[1].id == "uav_003"

    def test_calculate_max_revisit_gap(self):
        """Test max revisit gap calculation."""
        # Create cells with different observation times
        cell1 = Cell(id="c1", x=0, y=0, last_seen_ts=50.0)
        cell2 = Cell(id="c2", x=40, y=0, last_seen_ts=20.0)
        cell3 = Cell(id="c3", x=80, y=0)  # Never observed

        state = State(cells=[cell1, cell2, cell3], clock=100)

        # Cell1 age: 100 - 50 = 50s
        # Cell2 age: 100 - 20 = 80s
        # Cell3 age: infinity (ignored)
        # Max should be 80s
        max_gap = state.calculate_max_revisit_gap()
        assert max_gap == 80.0
        assert state.max_revisit_gap == 80.0

    def test_calculate_max_revisit_gap_no_cells(self):
        """Test max revisit gap with no cells."""
        state = State()
        gap = state.calculate_max_revisit_gap()
        assert gap == 0.0

    def test_parameter_validation(self):
        """Test Pydantic field validation."""
        # Negative clock should fail
        with pytest.raises(ValueError):
            State(clock=-1)

        # Negative spare count should fail
        with pytest.raises(ValueError):
            State(spare_count=-1)

        # Negative coverage gaps should fail
        with pytest.raises(ValueError):
            State(total_coverage_gaps=-1)

        # Negative max revisit gap should fail
        with pytest.raises(ValueError):
            State(max_revisit_gap=-1.0)
