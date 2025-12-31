#  Strategic UAV Deconfliction System

A production-ready **Spatio-Temporal Conflict Detection System** for multiple UAVs operating in shared airspace. Built as a hybrid project featuring an interactive React/TypeScript web dashboard and Python reference implementation.

![System Status](https://img.shields.io/badge/status-production--ready-brightgreen)
![TypeScript](https://img.shields.io/badge/TypeScript-5.0-blue)
![React](https://img.shields.io/badge/React-18.3-61dafb)
![Python](https://img.shields.io/badge/Python-3.10+-yellow)
![Three.js](https://img.shields.io/badge/Three.js-0.170-black)

---

##  Table of Contents

1. [Project Overview](#project-overview)
2. [Key Features](#key-features)
3. [System Architecture](#system-architecture)
4. [Algorithm Design](#algorithm-design)
5. [Module Breakdown](#module-breakdown)
6. [Getting Started](#getting-started)
7. [Usage Guide](#usage-guide)
8. [Scalability Discussion](#scalability-discussion)
9. [Testing Strategy](#testing-strategy)


---

##  Project Overview

### Problem Statement
In shared urban airspace, multiple autonomous UAVs (drones) operate simultaneously for delivery, surveillance, and other missions. Without proper coordination, these drones risk mid-air collisions, causing safety hazards and property damage.

### Solution
This system provides **real-time spatio-temporal conflict detection** by:
- Analyzing pre-planned UAV trajectories
- Detecting potential conflicts BEFORE they occur
- Providing explainable alerts with timing, location, and severity
- Enabling operators to take preventive action

### Why This Matters
- **Safety**: Prevents mid-air collisions in autonomous drone fleets
- **Scalability**: Designed to handle 10,000+ drones efficiently
- **Explainability**: Every conflict is explained in human-readable terms
- **Production-Ready**: Modular, testable, and maintainable code

---

##  Key Features

| Feature | Description |
|---------|-------------|
| **3D Visualization** | Interactive Three.js viewer showing trajectories, positions, and conflicts |
| **Conflict Detection** | O(n × T × k) algorithm using spatial grid partitioning |
| **Severity Classification** | Critical, Warning, Caution levels based on separation ratio |
| **Timeline Playback** | Simulate drone movements with variable speed control |
| **Explainable Alerts** | Human-readable descriptions of each conflict |
| **Python Reference** | Complete Python implementation for interview discussions |

---

##  System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    UAV DECONFLICTION SYSTEM                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │  Data Layer  │───▶│ Core Engine  │───▶│ Presentation │     │
│  │              │    │              │    │    Layer     │     │
│  │  • UAV Data  │    │  • Conflict  │    │  • 3D View   │     │
│  │  • Waypoints │    │    Detector  │    │  • Timeline  │     │
│  │  • Positions │    │  • Trajectory│    │  • Alerts    │     │
│  │              │    │    Interp.   │    │              │     │
│  └──────────────┘    └──────────────┘    └──────────────┘     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Component Diagram

```
src/
├── types/
│   └── uav.ts                 # TypeScript interfaces (mirrors Python models.py)
├── lib/
│   ├── trajectoryInterpolation.ts  # Position interpolation (mirrors trajectory.py)
│   └── conflictDetector.ts    # Conflict detection (mirrors conflict_detector.py)
├── data/
│   └── sampleUAVData.ts       # Sample data generator (mirrors sample_data.py)
├── components/
│   ├── AirspaceViewer.tsx     # 3D visualization with Three.js
│   ├── ConflictPanel.tsx      # Conflict list and details
│   ├── UAVList.tsx            # Fleet management view
│   ├── TimelineControls.tsx   # Playback controls
│   ├── StatsPanel.tsx         # Analytics dashboard
│   └── PythonCodePanel.tsx    # Python code reference
└── pages/
    └── Index.tsx              # Main dashboard layout
```

---

##  Algorithm Design

### Core Algorithm: Temporal Sweep with Spatial Grid Partitioning

#### Step 1: Temporal Sweep
```
For each time step t from T_start to T_end:
    1. Calculate position of each UAV at time t
    2. Build spatial grid for fast neighbor queries
    3. Check UAV pairs in neighboring cells
    4. Record conflicts if separation < threshold
```

#### Step 2: Spatial Grid Hashing
```
┌─────┬─────┬─────┬─────┐
│     │     │ UAV │     │
│     │     │  1  │     │
├─────┼─────┼─────┼─────┤
│     │ UAV │     │     │
│     │  2  │     │     │  ← Only check UAVs in adjacent cells
├─────┼─────┼─────┼─────┤
│     │     │     │ UAV │
│     │     │     │  3  │
├─────┼─────┼─────┼─────┤
│ UAV │     │     │     │
│  4  │     │     │     │
└─────┴─────┴─────┴─────┘
```

#### Step 3: Trajectory Interpolation
```typescript
// Binary search for O(log n) position lookup
function getPositionAtTime(trajectory: Waypoint[], timestamp: number): Position3D {
    // Find segment using binary search
    // Linearly interpolate between waypoints
    return lerpPosition(wp1.position, wp2.position, t);
}
```

### Complexity Analysis

| Approach | Time Complexity | Space | Scalability |
|----------|-----------------|-------|-------------|
| Naive Pairwise | O(n² × T) | O(1) | ❌ Not scalable |
| **Spatial Grid** (Current) | O(n × T × k) | O(n) | ✅ Good |
| Octree/KD-tree | O(n × T × log n) | O(n) | ✅✅ Best |

Where: `n` = UAVs, `T` = time samples, `k` = average neighbors per cell

### Conflict Severity Classification

```typescript
function calculateSeverity(actualDistance: number, requiredDistance: number): Severity {
    const ratio = actualDistance / requiredDistance;
    if (ratio < 0.5) return 'critical';   // < 50% of required separation
    if (ratio < 0.8) return 'warning';    // 50-80% of required separation
    return 'caution';                      // 80-100% of required separation
}
```

---

## 📦 Module Breakdown

### 1. `types/uav.ts` - Data Models

| Interface | Purpose |
|-----------|---------|
| `Position3D` | 3D coordinate (x, y, z in meters) |
| `Waypoint` | Position + timestamp |
| `UAV` | Complete UAV with trajectory, safety radius |
| `Conflict` | Detected conflict with severity and description |
| `ConflictAnalysis` | Analysis results with statistics |

### 2. `lib/trajectoryInterpolation.ts` - Position Calculator

| Function | Complexity | Purpose |
|----------|------------|---------|
| `getPositionAtTime()` | O(log n) | Binary search + linear interpolation |
| `distance3D()` | O(1) | Euclidean distance calculation |
| `sampleTrajectory()` | O(n) | Generate points for visualization |
| `getTrajectoryBounds()` | O(n × m) | Calculate airspace bounds |

### 3. `lib/conflictDetector.ts` - Core Detection Engine

| Function | Complexity | Purpose |
|----------|------------|---------|
| `detectConflicts()` | O(n × T × k) | Main detection algorithm |
| `positionToGridKey()` | O(1) | Spatial hash function |
| `getNeighborKeys()` | O(1) | 27 neighbor cells lookup |
| `calculateSeverity()` | O(1) | Severity classification |
| `generateConflictDescription()` | O(1) | Human-readable explanation |

### 4. `data/sampleUAVData.ts` - Test Scenarios

**Conflict Scenarios Included:**
1. **Head-on collision** (UAV-1 ↔ UAV-2): East-West vs West-East at same altitude
2. **Crossing paths** (UAV-3 ↔ UAV-4): Diagonal trajectories intersecting
3. **Near-miss during climb** (UAV-5 ↔ UAV-6): Vertical takeoff meets patrol
4. **Three-way intersection** (UAV-1, UAV-2, UAV-7): Multiple UAVs converging

### 5. `components/AirspaceViewer.tsx` - 3D Visualization

**Technologies Used:**
- React Three Fiber (React renderer for Three.js)
- @react-three/drei (helpers for common 3D patterns)
- Three.js (WebGL-based 3D graphics)

**Visualization Elements:**
- UAV octahedron meshes with glow effects
- Trajectory lines with opacity based on selection
- Safety radius spheres (when selected)
- Conflict markers with rotating torus geometry
- Ground grid with axis labels

---

##  Getting Started

### Prerequisites
- Node.js 18+ with npm
- Modern browser with WebGL support

### Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/uav-deconfliction.git

# Navigate to project
cd uav-deconfliction

# Install dependencies
npm install

# Start development server
npm run dev
```

### Build for Production

```bash
npm run build
```

---

## 📖 Usage Guide

### Dashboard Controls

| Control | Function |
|---------|----------|
| **Play/Pause** | Start/stop simulation playback |
| **Speed (0.5x-4x)** | Adjust playback speed |
| **Timeline Slider** | Jump to specific time |
| **Reset** | Return to start time |
| **UAV List** | Click to select/highlight UAV |
| **Conflict List** | Click to jump to conflict time |

### 3D Viewer Interactions

| Action | Result |
|--------|--------|
| Left-click drag | Rotate camera |
| Right-click drag | Pan camera |
| Scroll | Zoom in/out |
| Click UAV | Select and show details |

---

## 📈 Scalability Discussion

### Current Implementation (Good for 100-500 UAVs)
- Spatial grid with fixed cell size
- O(n × T × k) complexity
- Single-threaded JavaScript execution

### Production Scaling (10,000+ UAVs)

#### 1. Data Structure Improvements
```python
# Replace grid with octree for O(log n) queries
class Octree:
    def insert(self, uav_position):
        # Recursive spatial subdivision
        pass
    
    def query_radius(self, center, radius):
        # Only visit relevant octants
        pass
```

#### 2. Parallel Processing
```python
from multiprocessing import Pool

def parallel_detect(uavs, time_ranges):
    with Pool(processes=8) as pool:
        results = pool.starmap(detect_in_range, time_ranges)
    return merge_conflicts(results)
```

#### 3. GPU Acceleration
```python
import cupy as cp

def gpu_distance_matrix(positions):
    # CUDA kernel for parallel distance computation
    # 1000x speedup for 10,000+ UAVs
    pass
```

#### 4. Real-Time Updates
- Use WebSockets for streaming trajectory updates
- Implement incremental conflict detection
- Cache spatial structures between frames

---

## 🧪 Testing Strategy

### Unit Tests

```typescript
describe('trajectoryInterpolation', () => {
    test('interpolates midpoint correctly', () => {
        const trajectory = [
            { position: { x: 0, y: 0, z: 0 }, timestamp: 0 },
            { position: { x: 100, y: 100, z: 100 }, timestamp: 10 }
        ];
        const pos = getPositionAtTime(trajectory, 5);
        expect(pos.x).toBeCloseTo(50);
        expect(pos.y).toBeCloseTo(50);
        expect(pos.z).toBeCloseTo(50);
    });
});
```

### Integration Tests

```typescript
describe('conflictDetector', () => {
    test('detects head-on collision', () => {
        const uavs = generateSampleUAVs();
        const analysis = detectConflicts(uavs, 1, 0);
        
        // Verify UAV-1 and UAV-2 conflict is detected
        const conflict = analysis.conflicts.find(
            c => c.uav1Id === 'uav-1' && c.uav2Id === 'uav-2'
        );
        expect(conflict).toBeDefined();
        expect(conflict.severity).toBe('critical');
    });
});
```

### Performance Benchmarks

```typescript
describe('performance', () => {
    test('scales linearly with UAV count', () => {
        const times: number[] = [];
        
        for (const count of [10, 50, 100, 200]) {
            const uavs = generateRandomUAVs(count);
            const start = performance.now();
            detectConflicts(uavs, 1, 0);
            times.push(performance.now() - start);
        }
        
        // Verify sub-linear or linear growth
        expect(times[3] / times[0]).toBeLessThan(25); // Should be ~20x, not 400x
    });
});
```

---


## 📄 License

MIT License - See LICENSE file for details.

---

## 👤 Author

Built as a portfolio project demonstrating expertise in:
- Robotics software engineering
- Spatial algorithms and data structures
- Real-time 3D visualization
- TypeScript/React and Python development
- Production-ready system design

---


