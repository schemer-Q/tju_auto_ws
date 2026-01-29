from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Dict, Optional

class SlotStatus(Enum):
    """Status of a single slot."""
    EMPTY = auto()
    PLANNED = auto()
    OCCUPIED = auto()

class Orientation(Enum):
    """Orientation of the cotton bale."""
    HORIZONTAL = auto()
    VERTICAL = auto()


def _toggle_orientation(orientation: Orientation) -> Orientation:
    """Returns the opposite orientation."""
    return Orientation.VERTICAL if orientation == Orientation.HORIZONTAL else Orientation.HORIZONTAL

class WallStatus(Enum):
    """Status of a wall building process."""
    BUILDING = auto()
    COMPLETED = auto()

@dataclass
class Slot:
    """
    Represents a single cotton bale position.
    """
    slot_id: str
    layer_index: int
    index_in_layer: int
    orientation: Orientation
    status: SlotStatus = SlotStatus.EMPTY

@dataclass
class Layer:
    """
    Represents a horizontal stacking layer.
    """
    layer_index: int
    orientation_pattern: Orientation
    slots: List[Slot] = field(default_factory=list)

    def is_complete(self) -> bool:
        """
        Returns True if the layer is physically complete (all slots OCCUPIED).
        Used for the Compression Rule: strict dependency on physical completion.
        """
        return all(s.status == SlotStatus.OCCUPIED for s in self.slots)

    def is_fully_planned(self) -> bool:
        """
        Returns True if all slots are either PLANNED or OCCUPIED.
        """
        return all(s.status != SlotStatus.EMPTY for s in self.slots)

    def get_next_empty_slot(self) -> Optional[Slot]:
        """
        Returns the next available (EMPTY) slot in the sequence (Left to Right).
        """
        # Slots are assumed to be sorted by index_in_layer
        for slot in self.slots:
            if slot.status == SlotStatus.EMPTY:
                return slot
        return None

@dataclass
class Wall:
    """
    Represents one cotton bale wall.
    """
    wall_id: int
    # Layers should be ordered from bottom (0) to top
    layers: List[Layer] = field(default_factory=list)
    status: WallStatus = WallStatus.BUILDING

    def is_complete(self) -> bool:
        """
        Returns True if the wall is fully completed (all layers OCCUPIED).
        """
        if not self.layers:
            return False
            
        # Check if all layers are complete
        if all(l.is_complete() for l in self.layers):
            self.status = WallStatus.COMPLETED
            return True
        return False

    def get_active_layer(self) -> Optional[Layer]:
        """
        Returns the current layer being worked on.
        Follows Compression Rule: A new layer starts only after previous is fully filled.
        """
        for layer in self.layers:
            # If this layer is not fully occupied, it is the active one (either filling or waiting)
            if not layer.is_complete():
                return layer
        return None

    def get_next_slot(self) -> Optional[Slot]:
        """
        Determines the next valid slot for this wall.
        """
        if self.status == WallStatus.COMPLETED:
            return None

        # Find the active layer (bottom-up, compression rule enforced)
        active_layer = self.get_active_layer()
        if active_layer is None:
            # All layers complete
            self.status = WallStatus.COMPLETED
            return None
            
        # Try to find an empty slot in the active layer
        next_slot = active_layer.get_next_empty_slot()
        
        if next_slot:
            return next_slot
        else:
            # Active layer has no empty slots (fully PLANNED) but is NOT COMPLETE (not fully OCCUPIED).
            # We must wait for this layer to finish physically before starting the next.
            return None

@dataclass
class StorageSite:
    """
    Represents one standard storage site containing multiple walls.
    """
    site_id: int
    walls: List[Wall] = field(default_factory=list)
    active_wall_index: int = 0

    def get_active_wall(self) -> Optional[Wall]:
        """
        Returns the wall currently being built.
        """
        # Ensure index is valid
        if self.active_wall_index >= len(self.walls):
            return None
            
        wall = self.walls[self.active_wall_index]
        if wall.is_complete():
            # Trigger transition to next wall automatically
            self.active_wall_index += 1
            if self.active_wall_index >= len(self.walls):
                return None
            return self.walls[self.active_wall_index]
            
        return wall

    def get_next_slot(self) -> Optional[Slot]:
        """
        Returns the next valid slot for the entire site.
        """
        wall = self.get_active_wall()
        if wall is None:
            return None
            
        return wall.get_next_slot()

class WorldModel:
    """
    Top-level state container for the Cotton Bale Stacking System.
    """
    def __init__(self):
        self.storage_sites: Dict[int, StorageSite] = {}

    def create_storage_site(self, site_id: int, num_walls: int, layers_per_wall: int, slots_per_layer: int) -> None:
        """
        Helper to initialize a standard storage site with deterministically generated slots.
        
        Rules applied:
        - Layer 0: Horizontal Pattern
        - Layer 1: Vertical Pattern
        - (Alternating)
        """
        walls = []
        for w_idx in range(num_walls):
            layers = []
            for l_idx in range(layers_per_wall):
                # Orientation Rule: Alternates every layer.
                # Layer 0 starts with HORIZONTAL, layer 1 starts with VERTICAL, and so on.
                if l_idx % 2 == 0:
                    orientation = Orientation.HORIZONTAL
                else:
                    orientation = Orientation.VERTICAL
                
                slots = []
                for s_idx in range(slots_per_layer):
                    # Within a layer, slots alternate orientation starting from orientation_pattern.
                    slot_orientation = orientation if s_idx % 2 == 0 else _toggle_orientation(orientation)
                    slot_id = f"S{site_id}_W{w_idx}_L{l_idx}_{s_idx}"
                    slot = Slot(
                        slot_id=slot_id,
                        layer_index=l_idx,
                        index_in_layer=s_idx,
                        orientation=slot_orientation,
                        status=SlotStatus.EMPTY
                    )
                    slots.append(slot)
                
                layers.append(Layer(
                    layer_index=l_idx,
                    orientation_pattern=orientation,
                    slots=slots
                ))
            
            walls.append(Wall(
                wall_id=w_idx,
                layers=layers,
                status=WallStatus.BUILDING
            ))
            
        self.storage_sites[site_id] = StorageSite(
            site_id=site_id,
            walls=walls,
            active_wall_index=0
        )

    def get_next_valid_slot(self, site_id: int) -> Optional[Slot]:
        """
        Returns a reference to the next legal Slot based on stacking rules, or None if full/busy.
        """
        if site_id not in self.storage_sites:
            return None
            
        return self.storage_sites[site_id].get_next_slot()

    def mark_slot_planned(self, slot: Slot) -> None:
        """
        Transitions a slot from EMPTY to PLANNED.
        """
        if slot.status != SlotStatus.EMPTY:
            raise ValueError("Slot must be EMPTY to mark as PLANNED.")
        slot.status = SlotStatus.PLANNED

    def mark_slot_occupied(self, slot: Slot) -> None:
        """
        Transitions a slot to OCCUPIED.
        """
        if slot.status != SlotStatus.PLANNED:
            raise ValueError("Slot must be PLANNED to mark as OCCUPIED.")
        slot.status = SlotStatus.OCCUPIED

    def is_site_full(self, site_id: int) -> bool:
        """
        Checks if the entire site is completed.
        """
        if site_id not in self.storage_sites:
            return False
        
        site = self.storage_sites[site_id]
        # Check if we passed the last wall
        if site.active_wall_index >= len(site.walls):
            return True
            
        # Or check if the last wall is complete
        # (Though active_wall_index logic should handle it)
        return all(w.status == WallStatus.COMPLETED for w in site.walls)
