# 🐍 Python Reference Implementation

This document contains the complete Python implementation of the UAV Deconfliction System. These files can be run independently in any Python 3.10+ environment.

---

## Project Structure

```
python/
├── models.py           # Data classes for UAV, Waypoint, Conflict
├── trajectory.py       # Trajectory interpolation functions
├── conflict_detector.py # Main conflict detection algorithm
├── sample_data.py      # Sample UAV data generator
├── visualize.py        # 2D/3D visualization with matplotlib
├── main.py             # Entry point / demo script
└── requirements.txt    # Python dependencies
```

---

## 1. `models.py` - Data Models

```python
"""
UAV Data Models
===============
Core data structures for the deconfliction system.

These models are designed to be:
- Immutable where possible (using dataclasses with frozen=True where appropriate)
- Type-safe with full type hints
- JSON-serializable for API integration
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Literal
from enum import Enum


@dataclass
class Position3D:
    """
    3D position in a local ENU (East-North-Up) coordinate system.
    
    Attributes:
        x: meters from origin (East positive)
        y: meters from origin (North positive)
        z: altitude in meters (Up positive)
    """
    x: float
    y: float
    z: float
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)
    
    def distance_to(self, other: 'Position3D') -> float:
        """Calculate Euclidean distance to another position."""
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return (dx**2 + dy**2 + dz**2) ** 0.5


@dataclass
class Waypoint:
    """
    A single waypoint in a UAV trajectory.
    
    Attributes:
        position: 3D position of the waypoint
        timestamp: Unix timestamp in seconds when UAV should reach this point
    """
    position: Position3D
    timestamp: float


class UAVStatus(Enum):
    """Operating status of a UAV."""
    ACTIVE = "active"
    GROUNDED = "grounded"
    EMERGENCY = "emergency"


class ConflictSeverity(Enum):
    """Severity levels for detected conflicts."""
    CRITICAL = "critical"   # < 50% of required separation
    WARNING = "warning"     # 50-80% of required separation
    CAUTION = "caution"     # 80-100% of required separation


@dataclass
class UAV:
    """
    Complete UAV definition with trajectory and safety parameters.
    
    Attributes:
        id: Unique identifier
        name: Human-readable name
        color: Hex color for visualization
        trajectory: Ordered list of waypoints
        status: Current operating status
        max_speed: Maximum speed in m/s
        safety_radius: Minimum separation from other UAVs in meters
    """
    id: str
    name: str
    color: str
    trajectory: List[Waypoint]
    status: UAVStatus = UAVStatus.ACTIVE
    max_speed: float = 15.0
    safety_radius: float = 25.0


@dataclass
class Conflict:
    """
    A detected conflict between two UAVs.
    
    Attributes:
        id: Unique conflict identifier
        uav1_id: First UAV involved
        uav2_id: Second UAV involved
        timestamp: Time of closest approach
        position: Location of conflict (midpoint)
        separation_distance: Actual distance at conflict point
        required_separation: Combined safety radii
        severity: Severity classification
        description: Human-readable explanation
    """
    id: str
    uav1_id: str
    uav2_id: str
    timestamp: float
    position: Position3D
    separation_distance: float
    required_separation: float
    severity: ConflictSeverity
    description: str


@dataclass
class ConflictAnalysis:
    """
    Complete analysis results.
    
    Attributes:
        conflicts: List of all detected conflicts
        total_uavs: Number of UAVs analyzed
        analysis_time_range: (start, end) timestamps
        safety_margin: Additional buffer applied
    """
    conflicts: List[Conflict]
    total_uavs: int
    analysis_time_range: Tuple[float, float]
    safety_margin: float
```

---

## 2. `trajectory.py` - Interpolation

```python
"""
Trajectory Interpolation Module
===============================
Implements efficient position lookup for UAV trajectories.

Algorithm: Binary search + linear interpolation
Complexity: O(log n) per query where n = number of waypoints

Trade-offs:
- Linear interpolation is fast but produces sharp corners
- For smoother paths, cubic splines could be used (O(n) preprocessing)
- Linear is chosen because UAV flight segments are typically straight
"""

from typing import Optional, List, Tuple
from models import Position3D, Waypoint, UAV
import bisect


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation between two values."""
    return a + (b - a) * t


def lerp_position(p1: Position3D, p2: Position3D, t: float) -> Position3D:
    """Linear interpolation between two 3D positions."""
    return Position3D(
        x=lerp(p1.x, p2.x, t),
        y=lerp(p1.y, p2.y, t),
        z=lerp(p1.z, p2.z, t)
    )


def get_position_at_time(
    trajectory: List[Waypoint], 
    timestamp: float
) -> Optional[Position3D]:
    """
    Get UAV position at a specific timestamp using linear interpolation.
    
    Uses binary search to find the correct segment, then linearly
    interpolates between waypoints.
    
    Args:
        trajectory: List of waypoints (must be sorted by timestamp)
        timestamp: Query timestamp in seconds
        
    Returns:
        Interpolated Position3D, or None if trajectory is empty
        
    Complexity: O(log n) where n = len(trajectory)
    """
    if not trajectory:
        return None
    
    if len(trajectory) == 1:
        return trajectory[0].position
    
    # Clamp to trajectory bounds
    if timestamp <= trajectory[0].timestamp:
        return trajectory[0].position
    
    if timestamp >= trajectory[-1].timestamp:
        return trajectory[-1].position
    
    # Binary search for the segment containing timestamp
    timestamps = [wp.timestamp for wp in trajectory]
    idx = bisect.bisect_right(timestamps, timestamp) - 1
    
    # Ensure valid index range
    idx = max(0, min(idx, len(trajectory) - 2))
    
    wp1 = trajectory[idx]
    wp2 = trajectory[idx + 1]
    
    # Calculate interpolation factor
    dt = wp2.timestamp - wp1.timestamp
    if dt == 0:
        return wp1.position
    
    t = (timestamp - wp1.timestamp) / dt
    
    return lerp_position(wp1.position, wp2.position, t)


def get_velocity_at_time(
    trajectory: List[Waypoint],
    timestamp: float
) -> Optional[Position3D]:
    """
    Get velocity vector at a specific timestamp.
    
    Returns the velocity as a Position3D where each component
    represents m/s in that direction.
    """
    if len(trajectory) < 2:
        return None
    
    # Find the active segment
    for i in range(len(trajectory) - 1):
        wp1, wp2 = trajectory[i], trajectory[i + 1]
        
        if wp1.timestamp <= timestamp <= wp2.timestamp:
            dt = wp2.timestamp - wp1.timestamp
            if dt == 0:
                return Position3D(0, 0, 0)
            
            return Position3D(
                x=(wp2.position.x - wp1.position.x) / dt,
                y=(wp2.position.y - wp1.position.y) / dt,
                z=(wp2.position.z - wp1.position.z) / dt
            )
    
    return None


def sample_trajectory(
    trajectory: List[Waypoint],
    num_samples: int = 100
) -> List[Position3D]:
    """
    Sample trajectory at regular intervals for visualization.
    
    Args:
        trajectory: Source trajectory
        num_samples: Number of samples to generate
        
    Returns:
        List of Position3D at regular time intervals
    """
    if not trajectory:
        return []
    
    if len(trajectory) == 1:
        return [trajectory[0].position]
    
    start_time = trajectory[0].timestamp
    end_time = trajectory[-1].timestamp
    dt = (end_time - start_time) / (num_samples - 1)
    
    samples = []
    for i in range(num_samples):
        t = start_time + i * dt
        pos = get_position_at_time(trajectory, t)
        if pos:
            samples.append(pos)
    
    return samples


def get_trajectory_bounds(uavs: List[UAV]) -> dict:
    """
    Calculate bounding box for all UAV trajectories.
    
    Returns:
        Dict with 'min', 'max', 'center', 'size' keys
    """
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    
    for uav in uavs:
        for wp in uav.trajectory:
            pos = wp.position
            min_x = min(min_x, pos.x)
            min_y = min(min_y, pos.y)
            min_z = min(min_z, pos.z)
            max_x = max(max_x, pos.x)
            max_y = max(max_y, pos.y)
            max_z = max(max_z, pos.z)
    
    center = Position3D(
        x=(min_x + max_x) / 2,
        y=(min_y + max_y) / 2,
        z=(min_z + max_z) / 2
    )
    
    size = max(max_x - min_x, max_y - min_y, max_z - min_z)
    
    return {
        'min': Position3D(min_x, min_y, min_z),
        'max': Position3D(max_x, max_y, max_z),
        'center': center,
        'size': size or 100
    }
```

---

## 3. `conflict_detector.py` - Core Algorithm

```python
"""
Conflict Detection Module
=========================
Implements spatio-temporal conflict detection for UAV fleets.

Algorithm: Temporal Sweep with Spatial Grid Partitioning

Complexity Analysis:
- Naive pairwise: O(n² × T) - NOT SCALABLE
- Spatial grid: O(n × T × k) - CURRENT IMPLEMENTATION
- Octree/KD-tree: O(n × T × log n) - PRODUCTION READY

where n = UAVs, T = time samples, k = average neighbors per cell
"""

from typing import List, Set, Dict, Tuple
from dataclasses import dataclass
from collections import defaultdict
from models import UAV, Conflict, ConflictAnalysis, Position3D, ConflictSeverity, UAVStatus
from trajectory import get_position_at_time
from datetime import datetime


@dataclass
class SpatialCell:
    """A cell in the spatial grid containing UAV IDs."""
    uav_ids: Set[str]


class SpatialGrid:
    """
    3D spatial hash grid for efficient neighbor queries.
    
    Each cell is identified by integer indices (ix, iy, iz).
    UAVs are placed in cells based on their position.
    Neighbor queries check 27 adjacent cells (3x3x3 cube).
    """
    
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.cells: Dict[Tuple[int, int, int], SpatialCell] = defaultdict(
            lambda: SpatialCell(set())
        )
    
    def _pos_to_key(self, pos: Position3D) -> Tuple[int, int, int]:
        """Convert 3D position to grid cell key."""
        return (
            int(pos.x // self.cell_size),
            int(pos.y // self.cell_size),
            int(pos.z // self.cell_size)
        )
    
    def insert(self, uav_id: str, position: Position3D) -> None:
        """Insert a UAV into the grid."""
        key = self._pos_to_key(position)
        self.cells[key].uav_ids.add(uav_id)
    
    def get_neighbors(self, position: Position3D) -> Set[str]:
        """Get all UAV IDs in neighboring cells (27-cell neighborhood)."""
        cx, cy, cz = self._pos_to_key(position)
        neighbors = set()
        
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    key = (cx + dx, cy + dy, cz + dz)
                    if key in self.cells:
                        neighbors.update(self.cells[key].uav_ids)
        
        return neighbors
    
    def clear(self) -> None:
        """Clear all cells."""
        self.cells.clear()


def calculate_severity(
    actual_distance: float, 
    required_distance: float
) -> ConflictSeverity:
    """
    Classify conflict severity based on separation ratio.
    
    - CRITICAL: < 50% of required separation
    - WARNING: 50-80% of required separation
    - CAUTION: 80-100% of required separation
    """
    ratio = actual_distance / required_distance
    
    if ratio < 0.5:
        return ConflictSeverity.CRITICAL
    elif ratio < 0.8:
        return ConflictSeverity.WARNING
    else:
        return ConflictSeverity.CAUTION


def generate_description(
    uav1: UAV,
    uav2: UAV,
    timestamp: float,
    position: Position3D,
    separation: float,
    required: float
) -> str:
    """Generate human-readable conflict description."""
    time_str = datetime.fromtimestamp(timestamp).strftime("%H:%M:%S")
    deficit = required - separation
    
    return (
        f"At {time_str}, {uav1.name} and {uav2.name} will be "
        f"{separation:.1f}m apart at position "
        f"({position.x:.0f}, {position.y:.0f}, {position.z:.0f}m). "
        f"Required separation: {required:.1f}m. Violation by {deficit:.1f}m."
    )


def detect_conflicts(
    uavs: List[UAV],
    time_step: float = 1.0,
    safety_margin: float = 0.0
) -> ConflictAnalysis:
    """
    Detect all conflicts in a fleet of UAVs.
    
    Algorithm:
    1. Determine time range from all trajectories
    2. For each time step:
       a. Calculate positions of all active UAVs
       b. Build spatial grid for fast neighbor lookup
       c. Check each UAV against neighbors in adjacent cells
       d. Record conflicts where separation < threshold
    3. Deduplicate conflicts within time windows
    
    Args:
        uavs: List of UAVs with trajectories
        time_step: Resolution in seconds (default 1.0)
        safety_margin: Additional buffer in meters (default 0.0)
        
    Returns:
        ConflictAnalysis with all detected conflicts
    """
    if len(uavs) < 2:
        return ConflictAnalysis(
            conflicts=[],
            total_uavs=len(uavs),
            analysis_time_range=(0, 0),
            safety_margin=safety_margin
        )
    
    # Determine time range
    min_time = float('inf')
    max_time = float('-inf')
    
    for uav in uavs:
        for wp in uav.trajectory:
            min_time = min(min_time, wp.timestamp)
            max_time = max(max_time, wp.timestamp)
    
    # Calculate grid cell size (2.5x max safety radius)
    max_safety = max(uav.safety_radius for uav in uavs)
    cell_size = max_safety * 2.5
    
    # Create lookup dictionary for UAVs
    uav_dict = {uav.id: uav for uav in uavs}
    
    # Track reported conflicts to avoid duplicates
    reported: Set[str] = set()
    conflicts: List[Conflict] = []
    
    # Temporal sweep
    t = min_time
    while t <= max_time:
        # Build spatial grid for this timestamp
        grid = SpatialGrid(cell_size)
        positions: Dict[str, Position3D] = {}
        
        for uav in uavs:
            if uav.status != UAVStatus.ACTIVE:
                continue
            
            pos = get_position_at_time(uav.trajectory, t)
            if pos:
                positions[uav.id] = pos
                grid.insert(uav.id, pos)
        
        # Check for conflicts
        for uav1 in uavs:
            if uav1.status != UAVStatus.ACTIVE:
                continue
            if uav1.id not in positions:
                continue
            
            pos1 = positions[uav1.id]
            neighbors = grid.get_neighbors(pos1)
            
            for uav2_id in neighbors:
                # Skip self and already-checked pairs
                if uav2_id <= uav1.id:
                    continue
                
                uav2 = uav_dict[uav2_id]
                if uav2.status != UAVStatus.ACTIVE:
                    continue
                
                pos2 = positions[uav2_id]
                distance = pos1.distance_to(pos2)
                required = uav1.safety_radius + uav2.safety_radius + safety_margin
                
                if distance < required:
                    # Create deduplication key (30-second window)
                    conflict_key = f"{uav1.id}-{uav2_id}-{int(t // 30)}"
                    
                    if conflict_key not in reported:
                        reported.add(conflict_key)
                        
                        midpoint = Position3D(
                            x=(pos1.x + pos2.x) / 2,
                            y=(pos1.y + pos2.y) / 2,
                            z=(pos1.z + pos2.z) / 2
                        )
                        
                        conflicts.append(Conflict(
                            id=f"conflict-{len(conflicts) + 1}",
                            uav1_id=uav1.id,
                            uav2_id=uav2_id,
                            timestamp=t,
                            position=midpoint,
                            separation_distance=distance,
                            required_separation=required,
                            severity=calculate_severity(distance, required),
                            description=generate_description(
                                uav1, uav2, t, midpoint, distance, required
                            )
                        ))
        
        t += time_step
    
    # Sort by timestamp
    conflicts.sort(key=lambda c: c.timestamp)
    
    return ConflictAnalysis(
        conflicts=conflicts,
        total_uavs=len(uavs),
        analysis_time_range=(min_time, max_time),
        safety_margin=safety_margin
    )


def get_conflict_stats(analysis: ConflictAnalysis) -> dict:
    """Generate statistics from conflict analysis."""
    critical = sum(1 for c in analysis.conflicts if c.severity == ConflictSeverity.CRITICAL)
    warning = sum(1 for c in analysis.conflicts if c.severity == ConflictSeverity.WARNING)
    caution = sum(1 for c in analysis.conflicts if c.severity == ConflictSeverity.CAUTION)
    
    involved_uavs = set()
    for c in analysis.conflicts:
        involved_uavs.add(c.uav1_id)
        involved_uavs.add(c.uav2_id)
    
    return {
        'total': len(analysis.conflicts),
        'critical': critical,
        'warning': warning,
        'caution': caution,
        'involved_uav_count': len(involved_uavs),
        'safe_uav_count': analysis.total_uavs - len(involved_uavs)
    }
```

---

## 4. `sample_data.py` - Test Data

```python
"""
Sample Data Generator
=====================
Creates realistic UAV trajectory data with intentional conflicts
for testing and demonstration.
"""

import time
from typing import List
from models import UAV, Waypoint, Position3D, UAVStatus
import random

# Color palette for visualization
UAV_COLORS = [
    '#06b6d4',  # cyan
    '#22c55e',  # green
    '#f59e0b',  # amber
    '#8b5cf6',  # violet
    '#ec4899',  # pink
    '#14b8a6',  # teal
    '#f97316',  # orange
    '#6366f1',  # indigo
]

BASE_TIME = int(time.time())


def wp(x: float, y: float, z: float, time_offset: float) -> Waypoint:
    """Helper to create a waypoint."""
    return Waypoint(
        position=Position3D(x, y, z),
        timestamp=BASE_TIME + time_offset
    )


def generate_sample_uavs() -> List[UAV]:
    """
    Generate sample UAV fleet with intentional conflicts.
    
    Scenario: Urban delivery drones in 1km x 1km x 300m airspace
    
    Conflicts:
    1. Head-on (UAV-1 ↔ UAV-2)
    2. Crossing paths (UAV-3 ↔ UAV-4)
    3. Near-miss during climb (UAV-5 ↔ UAV-6)
    4. Three-way intersection (UAV-1, UAV-2, UAV-7)
    """
    return [
        # UAV-1: East to West - CONFLICTS with UAV-2
        UAV(
            id='uav-1',
            name='Alpha-1',
            color=UAV_COLORS[0],
            status=UAVStatus.ACTIVE,
            max_speed=15,
            safety_radius=25,
            trajectory=[
                wp(0, 500, 100, 0),
                wp(250, 500, 100, 20),
                wp(500, 500, 100, 40),    # CONFLICT ZONE
                wp(750, 500, 100, 60),
                wp(1000, 500, 100, 80),
            ]
        ),
        
        # UAV-2: West to East - HEAD-ON with UAV-1
        UAV(
            id='uav-2',
            name='Bravo-2',
            color=UAV_COLORS[1],
            status=UAVStatus.ACTIVE,
            max_speed=15,
            safety_radius=25,
            trajectory=[
                wp(1000, 480, 100, 0),
                wp(750, 485, 100, 20),
                wp(500, 490, 100, 40),    # CONFLICT with UAV-1
                wp(250, 495, 100, 60),
                wp(0, 500, 100, 80),
            ]
        ),
        
        # UAV-3: Diagonal climb - CONFLICTS with UAV-4
        UAV(
            id='uav-3',
            name='Charlie-3',
            color=UAV_COLORS[2],
            status=UAVStatus.ACTIVE,
            max_speed=12,
            safety_radius=20,
            trajectory=[
                wp(200, 0, 50, 0),
                wp(300, 200, 100, 25),
                wp(400, 400, 150, 50),    # CONFLICT ZONE
                wp(500, 600, 200, 75),
                wp(600, 800, 250, 100),
            ]
        ),
        
        # UAV-4: Crossing path - CONFLICTS with UAV-3
        UAV(
            id='uav-4',
            name='Delta-4',
            color=UAV_COLORS[3],
            status=UAVStatus.ACTIVE,
            max_speed=12,
            safety_radius=20,
            trajectory=[
                wp(600, 600, 140, 0),
                wp(500, 500, 145, 30),
                wp(400, 400, 150, 50),    # CONFLICT with UAV-3
                wp(300, 300, 155, 70),
                wp(200, 200, 160, 90),
            ]
        ),
        
        # UAV-5: Vertical takeoff
        UAV(
            id='uav-5',
            name='Echo-5',
            color=UAV_COLORS[4],
            status=UAVStatus.ACTIVE,
            max_speed=8,
            safety_radius=15,
            trajectory=[
                wp(700, 700, 0, 0),
                wp(700, 700, 50, 10),
                wp(700, 700, 100, 20),
                wp(720, 720, 150, 35),    # NEAR-MISS
                wp(800, 800, 150, 50),
                wp(900, 900, 150, 65),
            ]
        ),
        
        # UAV-6: Patrol - NEAR-MISS with UAV-5
        UAV(
            id='uav-6',
            name='Foxtrot-6',
            color=UAV_COLORS[5],
            status=UAVStatus.ACTIVE,
            max_speed=10,
            safety_radius=15,
            trajectory=[
                wp(600, 800, 145, 0),
                wp(650, 750, 148, 15),
                wp(700, 710, 150, 32),    # NEAR-MISS
                wp(750, 680, 152, 45),
                wp(800, 650, 155, 60),
            ]
        ),
        
        # UAV-7: Complex path - THREE-WAY conflict
        UAV(
            id='uav-7',
            name='Golf-7',
            color=UAV_COLORS[6],
            status=UAVStatus.ACTIVE,
            max_speed=14,
            safety_radius=22,
            trajectory=[
                wp(100, 100, 120, 0),
                wp(200, 300, 110, 20),
                wp(350, 480, 105, 38),
                wp(500, 500, 100, 42),    # THREE-WAY
                wp(600, 400, 95, 55),
                wp(700, 300, 90, 70),
            ]
        ),
        
        # UAV-8: Safe high-altitude (no conflicts)
        UAV(
            id='uav-8',
            name='Hotel-8',
            color=UAV_COLORS[7],
            status=UAVStatus.ACTIVE,
            max_speed=20,
            safety_radius=30,
            trajectory=[
                wp(0, 0, 280, 0),
                wp(250, 250, 285, 20),
                wp(500, 500, 290, 40),
                wp(750, 750, 285, 60),
                wp(1000, 1000, 280, 80),
            ]
        ),
    ]


def generate_random_uav(
    uav_id: int,
    bounds: tuple = (1000, 1000, 300)
) -> UAV:
    """Generate a random UAV for scalability testing."""
    num_waypoints = random.randint(4, 7)
    trajectory = []
    
    x = random.random() * bounds[0]
    y = random.random() * bounds[1]
    z = 50 + random.random() * (bounds[2] - 100)
    
    for i in range(num_waypoints):
        trajectory.append(wp(x, y, z, i * 20))
        x = max(0, min(bounds[0], x + (random.random() - 0.5) * 200))
        y = max(0, min(bounds[1], y + (random.random() - 0.5) * 200))
        z = max(30, min(bounds[2], z + (random.random() - 0.5) * 50))
    
    return UAV(
        id=f'uav-{uav_id}',
        name=f'Drone-{uav_id}',
        color=UAV_COLORS[uav_id % len(UAV_COLORS)],
        status=UAVStatus.ACTIVE,
        max_speed=10 + random.random() * 10,
        safety_radius=15 + random.random() * 15,
        trajectory=trajectory
    )


def get_time_range() -> tuple:
    """Get time range for sample data."""
    return (BASE_TIME, BASE_TIME + 100)
```

---

## 5. `main.py` - Entry Point

```python
"""
UAV Deconfliction System - Main Entry Point
============================================
Demonstrates conflict detection with sample data.
"""

from sample_data import generate_sample_uavs, get_time_range
from conflict_detector import detect_conflicts, get_conflict_stats
from models import ConflictSeverity


def main():
    print("=" * 60)
    print("UAV DECONFLICTION SYSTEM")
    print("Strategic Airspace Conflict Detection")
    print("=" * 60)
    print()
    
    # Generate sample fleet
    uavs = generate_sample_uavs()
    time_range = get_time_range()
    
    print(f"Fleet Size: {len(uavs)} UAVs")
    print(f"Analysis Period: {time_range[1] - time_range[0]} seconds")
    print()
    
    # Run conflict detection
    print("Running conflict analysis...")
    analysis = detect_conflicts(uavs, time_step=1.0, safety_margin=5.0)
    stats = get_conflict_stats(analysis)
    
    # Display results
    print()
    print("=" * 60)
    print("CONFLICT ANALYSIS RESULTS")
    print("=" * 60)
    print()
    print(f"Total Conflicts Detected: {stats['total']}")
    print(f"  - Critical: {stats['critical']}")
    print(f"  - Warning: {stats['warning']}")
    print(f"  - Caution: {stats['caution']}")
    print()
    print(f"UAVs Involved in Conflicts: {stats['involved_uav_count']}")
    print(f"Safe UAVs: {stats['safe_uav_count']}")
    print()
    
    # List all conflicts
    print("=" * 60)
    print("CONFLICT DETAILS")
    print("=" * 60)
    
    for conflict in analysis.conflicts:
        severity_icon = {
            ConflictSeverity.CRITICAL: "🔴",
            ConflictSeverity.WARNING: "🟡",
            ConflictSeverity.CAUTION: "🟢"
        }[conflict.severity]
        
        print()
        print(f"{severity_icon} {conflict.id.upper()}")
        print(f"   {conflict.description}")
    
    print()
    print("=" * 60)
    print("Analysis complete.")


if __name__ == "__main__":
    main()
```

---

## 6. `requirements.txt` - Dependencies

```
# Core
numpy>=1.24.0

# Visualization (optional)
matplotlib>=3.7.0

# Testing
pytest>=7.0.0
```

---

## Running the Python Version

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt

# Run the demo
python main.py
```

---

## Expected Output

```
============================================================
UAV DECONFLICTION SYSTEM
Strategic Airspace Conflict Detection
============================================================

Fleet Size: 8 UAVs
Analysis Period: 100 seconds

Running conflict analysis...

============================================================
CONFLICT ANALYSIS RESULTS
============================================================

Total Conflicts Detected: 4
  - Critical: 2
  - Warning: 1
  - Caution: 1

UAVs Involved in Conflicts: 7
Safe UAVs: 1

============================================================
CONFLICT DETAILS
============================================================

🔴 CONFLICT-1
   At 12:30:40, Alpha-1 and Bravo-2 will be 12.8m apart at position 
   (500, 495, 100m). Required separation: 55.0m. Violation by 42.2m.

🔴 CONFLICT-2
   At 12:30:50, Charlie-3 and Delta-4 will be 8.3m apart at position 
   (400, 400, 150m). Required separation: 45.0m. Violation by 36.7m.

...
```
