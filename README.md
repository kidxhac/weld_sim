# Welding Simulation System

A comprehensive simulation system for multi-robot welding with gantry coordination, implementing both **Weld-on-Move (WOM)** and **Stop-and-Weld (SAW)** strategies.

## Overview

This system simulates a 4-robot welding setup mounted on a linear gantry. The robots can weld simultaneously while avoiding collisions in shared workspace zones. The system automatically selects the optimal welding strategy based on workpiece geometry.

### Key Features

- **Dual Strategy Planning**: WOM for long parallel welds, SAW for compact pieces
- **Collision Avoidance**: Automatic handling of shared workspace zones (s1, s2)
- **Real-time Visualization**: Animated display of robot movements and welding progress
- **Performance Optimization**: Minimizes total welding time through parallel execution
- **Flexible Configuration**: Easy customization of workspace and robot parameters

## System Architecture

```
weld_sim/
├── config/
│   └── scene.py              # Scene configuration (gantry, robots, zones)
├── planner/
│   ├── data_model.py         # Core data structures
│   ├── collision_rules.py    # Collision detection and mutex management
│   ├── wom_strategy.py       # Weld-on-Move algorithm
│   ├── saw_strategy.py       # Stop-and-Weld algorithm  
│   └── weld_planner.py       # Master planner coordinator
├── simulator/
│   └── simulator.py          # Time-stepped physics simulation
├── ui/
│   └── renderer.py           # Matplotlib-based visualization
└── main.py                   # Entry point with test scenarios
```

## Welding Strategies

### Weld-on-Move (WOM)

**Use case**: Long welds parallel to X-axis (Lx/Ly > 7)

**Algorithm**:
1. Group welds into "windows" based on similar X-length and position
2. Calculate optimal fixed Y positions for each robot
3. Robots remain stationary in Y while gantry moves continuously
4. All robots weld simultaneously for maximum efficiency

**Advantages**:
- Continuous operation, no start/stop cycles
- Maximum throughput for long parallel welds
- Simplified coordination

### Stop-and-Weld (SAW)

**Use case**: Compact pieces with varying Y positions

**Algorithm**:
1. Determine optimal gantry stop positions along X-axis
2. At each stop, assign welds to robots based on Y reachability
3. Robots move independently in Y to reach weld positions
4. Priority-based collision resolution in shared zones

**Advantages**:
- Flexible positioning for complex geometries
- Better coverage of scattered weld patterns
- Efficient for shorter welds

### Hybrid Strategy

Automatically classifies welds and applies the best strategy for each, combining WOM and SAW in a single execution plan.

## Collision Handling

The system manages two collision zones:
- **s1**: Shared zone between R1 and R3 (x_plus side)
- **s2**: Shared zone between R2 and R4 (x_minus side)

**Collision Resolution**:
1. Mutex locks prevent simultaneous access to zones
2. Priority-based system: R1 > R3, R2 > R4
3. Automatic safe distance maintenance (150mm minimum)
4. Wait queues for blocked robots

## Installation

```bash
# Clone or extract the project
cd weld_sim

# Install dependencies
pip install numpy matplotlib --break-system-packages

# Run the simulation
python main.py
```

## Usage

### Running Scenarios

The main program offers 7 pre-configured scenarios:

```bash
python main.py
```

**Scenarios**:
1. **WOM Test**: Long parallel welds (demonstrates WOM efficiency)
2. **SAW Test**: Short scattered welds (demonstrates SAW flexibility)
3. **Hybrid Test**: Mixed weld lengths (automatic strategy selection)
4. **Strategy Comparison**: Compares WOM, SAW, and Hybrid performance
5. **Small Workspace**: Quick test with reduced dimensions
6. **Large Workspace**: Complex scenario with many welds
7. **Custom Scenario**: User-defined weld patterns

### Controls

During simulation:
- **SPACE**: Pause/Resume
- **Q**: Quit

### Configuration

Edit `config/scene.py` to customize:

```python
SCENE = {
    "gantry": {
        "x_length": 6000.0,      # Total travel distance
        "x_speed": 300.0,        # Movement speed (mm/s)
        "y_span": 3000.0         # Workspace width
    },
    "robots": [
        {"id": "R1", "side": "x_plus", "y_range": (0, 1000), 
         "tcp_speed": 120},
        # ... more robots
    ],
    "interference": {
        "s1": (800, 1200),       # Collision zone Y-range
        "s2": (800, 1200)
    }
}
```

## Algorithm Details

### WOM Window Grouping

Welds are grouped into windows using:
- **Length similarity**: |(w1.length - w2.length) / w1.length| < 0.3
- **X-overlap**: Welds must have >70% X-axis overlap
- **Parallel execution**: All welds in a window execute simultaneously

### SAW Stop Optimization

Gantry stops are determined by:
- **Coverage analysis**: Ensure all welds reachable from at least one stop
- **Spacing optimization**: Maximum 500mm between stops
- **Reachability**: Robots can access welds within 400mm of stop position

### Time Estimation

**WOM Mode**:
```
time = Σ(window_time) + repositioning_time
window_time = max(weld_length / gantry_speed) across parallel welds
```

**SAW Mode**:
```
time = gantry_travel_time + Σ(stop_work_time)
stop_work_time = max(robot_work_time) across parallel robots
robot_work_time = move_time + weld_time
```

## Performance Metrics

The simulator tracks:
- Total welding time
- Robot utilization (weld time / total time)
- Idle time per robot
- Collision wait events
- Weld completion progress

Example output:
```
SIMULATION STATISTICS
==============================================================
Total time: 45.2s
Progress: 100.0%
Collision waits: 3

Robot Performance:
--------------------------------------------------------------
  R1: 5 welds, 78.3% utilization
       Weld: 35.4s | Move: 2.1s | Idle: 7.7s
  R2: 4 welds, 72.1% utilization
       Weld: 32.6s | Move: 3.2s | Idle: 9.4s
  ...
```

## Extending the System

### Adding New Strategies

1. Create new strategy class in `planner/`
2. Implement `is_suitable()` and `create_plan()` methods
3. Register in `WeldPlanner.plan()`

### Custom Weld Patterns

```python
from planner.data_model import Weld

welds = [
    Weld(id=1, x_start=0, x_end=2000, y=500, side="x_plus"),
    Weld(id=2, x_start=1000, x_end=2500, y=1200, side="x_minus"),
    # ... more welds
]
```

### Modifying Visualization

Edit `ui/renderer.py` to customize:
- Colors and styling
- Display elements
- Statistics panels
- Animation speed

## Testing

Run strategy comparison to evaluate performance:

```bash
python main.py
# Select option 4: Strategy Comparison
```

This will execute the same weld pattern with all three strategies and display timing results.

## Troubleshooting

**Issue**: Animation is slow or jerky
- Reduce `dt` (time step) in Simulator
- Increase `interval` in `renderer.show()`
- Use smaller scene (SCENE_SMALL)

**Issue**: Robots colliding
- Check collision zone definitions in scene config
- Verify safe_distance setting (default 150mm)
- Enable collision debug output

**Issue**: Poor strategy selection
- Check weld length thresholds (1500mm default)
- Verify aspect ratio calculations
- Review window grouping parameters

## Theory of Operation

### WOM Efficiency

WOM mode achieves maximum efficiency for long parallel welds because:
1. **No repositioning delays**: Robots stay fixed in Y
2. **Continuous motion**: Gantry moves without stopping
3. **Perfect parallelization**: All robots work simultaneously
4. **Predictable timing**: Deterministic based on longest weld

### SAW Flexibility

SAW mode handles complex patterns by:
1. **Independent robot motion**: Each robot optimizes its own path
2. **Load balancing**: Work distributed based on reachability
3. **Priority scheduling**: Collision zones managed with priorities
4. **Adaptive stops**: Gantry positions optimize coverage

### Collision Management Philosophy

The system uses **optimistic concurrency** with rollback:
- Robots attempt to enter zones without explicit permission
- Mutex locks prevent actual collisions
- Priority system breaks deadlocks
- Wait queues ensure fairness

## Future Enhancements

Potential improvements:
- [ ] Multi-pass welding (root, fill, cap)
- [ ] Quality monitoring and defect simulation
- [ ] Energy consumption modeling
- [ ] More complex collision geometries
- [ ] Machine learning for strategy selection
- [ ] Real robot controller integration

## Contributing

This is a simulation framework for research and education. Contributions welcome:
- New welding strategies
- Performance optimizations
- Visualization improvements
- Additional test scenarios

## License

MIT License - Free for research and commercial use

## References

- Robotic welding: Industrial applications and automation
- Multi-agent coordination in shared workspaces
- Motion planning for gantry-mounted robots
- Manufacturing optimization strategies

---

**Version**: 1.0.0  
**Author**: Welding Simulation Team  
**Last Updated**: January 2026
