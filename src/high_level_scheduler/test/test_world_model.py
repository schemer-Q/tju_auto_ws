import pytest

from high_level_scheduler.world_model import (
    WorldModel,
    SlotStatus,
    Orientation,
    WallStatus,
)


@pytest.fixture
def wm():
    model = WorldModel()
    # Default test configuration: 1 site, 1 wall, 2 layers, 4 slots per layer
    model.create_storage_site(site_id=0, num_walls=1, layers_per_wall=2, slots_per_layer=4)
    return model


def _fill_layer(model: WorldModel, site_id: int, layer_index: int) -> None:
    """Fill all slots in a specific layer in order."""
    while True:
        slot = model.get_next_valid_slot(site_id)
        if slot is None:
            return
        if slot.layer_index != layer_index:
            return
        model.mark_slot_planned(slot)
        model.mark_slot_occupied(slot)


def test_initial_state_first_slot(wm):
    """Test 1: Initial State."""
    slot = wm.get_next_valid_slot(site_id=0)
    assert slot is not None
    assert slot.layer_index == 0
    assert slot.index_in_layer == 0
    assert slot.status == SlotStatus.EMPTY


def test_sequential_slot_filling_left_to_right(wm):
    """Test 2: Sequential Slot Filling."""
    indices = []
    for _ in range(4):
        slot = wm.get_next_valid_slot(site_id=0)
        assert slot is not None
        indices.append(slot.index_in_layer)
        wm.mark_slot_planned(slot)
        wm.mark_slot_occupied(slot)

    assert indices == [0, 1, 2, 3]


def test_layer_completion_transition(wm):
    """Test 3: Layer Completion Transition."""
    _fill_layer(wm, site_id=0, layer_index=0)
    slot = wm.get_next_valid_slot(site_id=0)
    assert slot is not None
    assert slot.layer_index == 1


def test_orientation_alternation_rule(wm):
    """Test 4: Orientation Alternation Rule."""
    site = wm.storage_sites[0]
    wall = site.walls[0]

    layer0 = wall.layers[0]
    layer1 = wall.layers[1]

    assert layer0.orientation_pattern == Orientation.HORIZONTAL
    assert layer1.orientation_pattern == Orientation.VERTICAL

    # Verify slot-level alternation within each layer
    assert layer0.slots[0].orientation == Orientation.HORIZONTAL
    assert layer0.slots[1].orientation == Orientation.VERTICAL
    assert layer1.slots[0].orientation == Orientation.VERTICAL
    assert layer1.slots[1].orientation == Orientation.HORIZONTAL


def test_wall_completion(wm):
    """Test 5: Wall Completion."""
    # Fill all slots in both layers
    _fill_layer(wm, site_id=0, layer_index=0)
    _fill_layer(wm, site_id=0, layer_index=1)

    wall = wm.storage_sites[0].walls[0]
    assert wall.is_complete() is True
    assert wall.status == WallStatus.COMPLETED
    assert wm.get_next_valid_slot(site_id=0) is None


def test_site_completion(wm):
    """Test 6: Site Completion."""
    _fill_layer(wm, site_id=0, layer_index=0)
    _fill_layer(wm, site_id=0, layer_index=1)

    assert wm.is_site_full(site_id=0) is True
    assert wm.get_next_valid_slot(site_id=0) is None


def test_slot_state_transitions(wm):
    """Test 7: Slot State Transitions."""
    slot = wm.get_next_valid_slot(site_id=0)
    assert slot.status == SlotStatus.EMPTY

    wm.mark_slot_planned(slot)
    assert slot.status == SlotStatus.PLANNED

    wm.mark_slot_occupied(slot)
    assert slot.status == SlotStatus.OCCUPIED

    with pytest.raises(ValueError):
        wm.mark_slot_planned(slot)

    with pytest.raises(ValueError):
        wm.mark_slot_occupied(slot)


def test_determinism_check():
    """Test 8: Determinism Check."""
    wm1 = WorldModel()
    wm1.create_storage_site(site_id=0, num_walls=1, layers_per_wall=2, slots_per_layer=4)

    wm2 = WorldModel()
    wm2.create_storage_site(site_id=0, num_walls=1, layers_per_wall=2, slots_per_layer=4)

    seq1 = []
    seq2 = []

    while True:
        slot1 = wm1.get_next_valid_slot(site_id=0)
        slot2 = wm2.get_next_valid_slot(site_id=0)
        if slot1 is None or slot2 is None:
            break

        seq1.append((slot1.layer_index, slot1.index_in_layer))
        seq2.append((slot2.layer_index, slot2.index_in_layer))

        wm1.mark_slot_planned(slot1)
        wm1.mark_slot_occupied(slot1)

        wm2.mark_slot_planned(slot2)
        wm2.mark_slot_occupied(slot2)

    assert seq1 == seq2


def test_invalid_slot_access(wm):
    """Test 9: Invalid Slot Access (EMPTY -> OCCUPIED forbidden)."""
    slot = wm.get_next_valid_slot(site_id=0)
    assert slot.status == SlotStatus.EMPTY

    with pytest.raises(ValueError):
        wm.mark_slot_occupied(slot)


def test_overfilling_protection(wm):
    """Test 10: Overfilling Protection."""
    # Fill all slots in both layers
    _fill_layer(wm, site_id=0, layer_index=0)
    _fill_layer(wm, site_id=0, layer_index=1)

    assert wm.get_next_valid_slot(site_id=0) is None

    # Attempt to overfill a previously completed slot
    site = wm.storage_sites[0]
    slot = site.walls[0].layers[0].slots[0]
    with pytest.raises(ValueError):
        wm.mark_slot_planned(slot)
