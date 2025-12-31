# 🏛️ System Architecture Documentation

## Overview

This document provides a detailed technical architecture overview of the UAV Deconfliction System, suitable for technical interviews and code reviews.

---

## Architecture Layers

```
┌────────────────────────────────────────────────────────────────────────┐
│                           PRESENTATION LAYER                           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐     │
│  │ 3D Viewer   │ │ UAV List    │ │ Conflict    │ │ Timeline    │     │
│  │ (Three.js)  │ │             │ │ Panel       │ │ Controls    │     │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘     │
├────────────────────────────────────────────────────────────────────────┤
│                           APPLICATION LAYER                            │
│  ┌─────────────────────────┐ ┌─────────────────────────────────┐     │
│  │ Simulation State        │ │ User Interaction Handlers       │     │
│  │ (React Hooks)           │ │ (Selection, Playback, etc.)     │     │
│  └─────────────────────────┘ └─────────────────────────────────┘     │
├────────────────────────────────────────────────────────────────────────┤
│                              CORE LAYER                                │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐                     │
│  │ Conflict    │ │ Trajectory  │ │ Spatial     │                     │
│  │ Detector    │ │ Interpolator│ │ Grid        │                     │
│  └─────────────┘ └─────────────┘ └─────────────┘                     │
├────────────────────────────────────────────────────────────────────────┤
│                              DATA LAYER                                │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐                     │
│  │ UAV Models  │ │ Conflict    │ │ Sample Data │                     │
│  │             │ │ Models      │ │ Generator   │                     │
│  └─────────────┘ └─────────────┘ └─────────────┘                     │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow

```
User Input → Simulation State → Position Calculation → Conflict Detection → UI Update
     ↑                                                                          │
     └──────────────────────────────────────────────────────────────────────────┘
```

### Detailed Flow

1. **Initial Load**
   - `generateSampleUAVs()` creates UAV fleet
   - `detectConflicts()` runs analysis on all trajectories
   - Results cached in React state via `useMemo`

2. **Playback Loop**
   - `setInterval` advances `currentTime`
   - Each component reads `currentTime` to update display
   - `getPositionAtTime()` calculates UAV positions

3. **User Interaction**
   - Click UAV → Update `selectedUAV` state
   - Click conflict → Jump `currentTime` to conflict timestamp
   - Drag timeline → Update `currentTime` directly

---

## Key Data Structures

### UAV Trajectory

```typescript
interface UAV {
  id: string;           // Unique identifier
  name: string;         // Human-readable name
  trajectory: Waypoint[]; // Ordered list of positions + times
  safetyRadius: number;   // Minimum separation from other UAVs
}

interface Waypoint {
  position: Position3D;   // x, y, z in meters
  timestamp: number;      // Unix timestamp in seconds
}
```

### Spatial Grid

```typescript
interface SpatialGrid {
  cells: Map<string, SpatialCell>;  // Hash map: "ix,iy,iz" → UAV IDs
  cellSize: number;                  // Grid resolution in meters
}

// Grid key generation
function positionToGridKey(pos: Position3D, cellSize: number): string {
  const ix = Math.floor(pos.x / cellSize);
  const iy = Math.floor(pos.y / cellSize);
  const iz = Math.floor(pos.z / cellSize);
  return `${ix},${iy},${iz}`;
}
```

---

## Algorithm Details

### Trajectory Interpolation

```
Given: trajectory = [(pos1, t1), (pos2, t2), ..., (posN, tN)]
Goal: Find position at arbitrary time T

Algorithm:
1. Binary search to find i where t[i] <= T < t[i+1]
2. Calculate interpolation factor: alpha = (T - t[i]) / (t[i+1] - t[i])
3. Linear interpolation: result = pos[i] + alpha * (pos[i+1] - pos[i])

Complexity: O(log N) where N = number of waypoints
```

### Conflict Detection

```
Given: UAVs with trajectories, safety radii
Goal: Find all (uav_i, uav_j, time) where distance < required_separation

Algorithm:
1. FOR each time step t in [T_start, T_end]:
   a. Build spatial grid for current positions
   b. FOR each UAV:
      - Find candidate neighbors (27 adjacent cells)
      - FOR each candidate:
        - Calculate 3D distance
        - IF distance < combined_safety_radii:
          - Record conflict

Complexity: O(T × N × k) where:
  - T = number of time steps
  - N = number of UAVs
  - k = average neighbors per cell (typically 2-5)
```

---

## Design Decisions

### 1. Why Spatial Grid over Naive Pairwise?

| Factor | Naive O(n²) | Spatial Grid O(n×k) |
|--------|-------------|---------------------|
| 100 UAVs | 10,000 checks | ~500 checks |
| 1000 UAVs | 1,000,000 checks | ~5,000 checks |
| Scalability | ❌ Poor | ✅ Good |

### 2. Why Linear Interpolation over Cubic Splines?

| Factor | Linear | Cubic Spline |
|--------|--------|--------------|
| Speed | O(1) per segment | O(n) preprocessing |
| Accuracy | Sufficient for ~1s steps | Better for visualization |
| Simplicity | ✅ Simple | More complex |
| Real UAV paths | Matches flight segments | Over-smooths |

### 3. Why React Three Fiber over Plain Three.js?

| Factor | Plain Three.js | React Three Fiber |
|--------|----------------|-------------------|
| Integration | Manual DOM manipulation | Native React |
| State sync | Complex event binding | Automatic |
| Component model | Imperative | Declarative |
| Code maintainability | Lower | Higher |

---

## Performance Considerations

### Current Optimizations

1. **useMemo** for expensive computations (conflict detection, trajectory sampling)
2. **Binary search** for O(log n) position lookup
3. **Spatial hashing** for O(k) neighbor queries
4. **Conflict deduplication** to avoid redundant alerts

### Future Optimizations

1. **Web Workers** for off-main-thread conflict detection
2. **GPU instancing** for rendering 1000+ UAVs
3. **LOD (Level of Detail)** for distant UAVs
4. **Incremental updates** instead of full recomputation

---

## Testing Approach

### Unit Testing

```typescript
// Test interpolation accuracy
test('interpolates correctly at midpoint', () => {
  const trajectory = [
    { position: { x: 0, y: 0, z: 0 }, timestamp: 0 },
    { position: { x: 100, y: 0, z: 0 }, timestamp: 10 }
  ];
  expect(getPositionAtTime(trajectory, 5).x).toBeCloseTo(50);
});
```

### Integration Testing

```typescript
// Test conflict detection end-to-end
test('detects known conflicts in sample data', () => {
  const uavs = generateSampleUAVs();
  const analysis = detectConflicts(uavs, 1, 0);
  expect(analysis.conflicts.length).toBeGreaterThan(0);
});
```

### Visual Testing

- Manual inspection of 3D visualization
- Verify conflict markers appear at correct positions
- Verify timeline scrubbing updates positions smoothly

---

## Security Considerations

1. **Input Validation**: All trajectory data validated before processing
2. **No External APIs**: Runs entirely client-side
3. **No User Data Storage**: Stateless operation

---

## Deployment

### Development
```bash
npm run dev
```

### Production Build
```bash
npm run build
npm run preview
```

### Docker (Optional)
```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY package*.json ./
RUN npm install
COPY . .
RUN npm run build
CMD ["npm", "run", "preview"]
```
