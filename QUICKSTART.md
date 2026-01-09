# Quick Start Guide

Get up and running with the welding simulation in 5 minutes!

## Installation

```bash
# Navigate to the project directory
cd weld_sim

# Install dependencies
pip install numpy matplotlib --break-system-packages

# Verify installation
python test_system.py
```

You should see:
```
All tests passed! System is ready to use.
```

## Running Your First Simulation

```bash
python main.py
```

You'll see a menu:
```
Select scenario:
  1. WOM Test (Long parallel welds)
  2. SAW Test (Short scattered welds)
  3. Hybrid Test (Mixed weld lengths)
  ...
```

**Recommended**: Start with option **3** (Hybrid Test) for a balanced demonstration.

Press `3` and hit Enter.

## Understanding the Visualization

The simulation window shows:

### Main View (Top)
- **Blue vertical line**: Gantry position (moves left/right along X)
- **Colored circles**: Robots (one per robot)
  - Gray: IDLE (waiting for work)
  - Blue: MOVING_Y (repositioning)
  - Red: WELDING (actively welding)
  - Orange: WAIT_MUTEX (waiting for collision zone)
- **Light green lines**: Weld seams (to be welded)
- **Dark green lines**: Completed portions of welds
- **Yellow zones**: Collision zones (s1, s2)

### Statistics Panel (Bottom)
- Current time and progress
- Robot status and current tasks
- Collision wait count

### Controls
- **SPACE**: Pause/Resume
- **Q**: Quit

## Interpreting Results

After simulation completes, you'll see:

```
SIMULATION STATISTICS
==============================================================
Total time: 45.2s
Progress: 100.0%
Collision waits: 3

Robot Performance:
  R1: 5 welds, 78.3% utilization
       Weld: 35.4s | Move: 2.1s | Idle: 7.7s
```

**Key Metrics:**
- **Total time**: How long the job took
- **Collision waits**: Times robots waited for collision zones
- **Utilization**: Percentage of time spent welding (higher is better)
- **Weld/Move/Idle**: Breakdown of robot activity

## Trying Different Strategies

### Pure WOM Mode
```bash
python main.py
# Select option 1
```
Best for: Long parallel welds
Expected: High utilization, low collision waits

### Pure SAW Mode
```bash
python main.py
# Select option 2
```
Best for: Short scattered welds
Expected: More movement, potential collision waits

### Compare All Strategies
```bash
python main.py
# Select option 4
```
Runs the same scenario with WOM, SAW, and Hybrid, shows timing comparison.

## Customizing the Simulation

### Modify Scene Parameters

Edit `config/scene.py`:

```python
SCENE = {
    "gantry": {
        "x_length": 6000.0,      # Change workspace size
        "x_speed": 300.0,        # Adjust gantry speed
    },
    "robots": [
        {"id": "R1", "tcp_speed": 120},  # Adjust welding speed
        # ...
    ],
}
```

### Create Custom Welds

Edit `main.py` to add your own weld patterns:

```python
welds = [
    Weld(id=1, x_start=500, x_end=3000, y=600, side="x_plus"),
    Weld(id=2, x_start=1000, x_end=2500, y=1200, side="x_minus"),
    # Add more...
]
```

### Adjust Simulation Speed

In `renderer.show()` call, change interval:

```python
renderer.show(interval=20)  # Faster (20ms per frame)
renderer.show(interval=100)  # Slower (100ms per frame)
```

## Common Issues

### Animation is jerky
- Increase interval: `renderer.show(interval=50)`
- Reduce time step in Simulator: `dt=0.1`

### Window doesn't appear
- Check matplotlib backend: `import matplotlib; print(matplotlib.get_backend())`
- Try: `export MPLBACKEND=TkAgg`

### Import errors
```bash
# Make sure you're in the weld_sim directory
cd weld_sim
python main.py
```

## Next Steps

1. **Read ALGORITHM_EXPLANATION.md** for deep dive into strategies
2. **Read README.md** for complete documentation
3. **Experiment with custom scenarios** (option 7 in menu)
4. **Try different workspace sizes** (options 5-6 in menu)

## Example Workflow

```bash
# 1. Run quick test
python test_system.py

# 2. Try basic scenario
python main.py
# Select: 3 (Hybrid)

# 3. Compare strategies
python main.py
# Select: 4 (Comparison)

# 4. Experiment with custom
python main.py
# Select: 7 (Custom)
# Enter: 3 long welds, 5 short welds
```

## Understanding the Code Structure

```
weld_sim/
├── config/         # Scene configurations
├── planner/        # Planning algorithms (WOM, SAW)
├── simulator/      # Physics simulation
├── ui/             # Visualization
└── main.py         # Entry point
```

**To modify planning**: Edit `planner/wom_strategy.py` or `planner/saw_strategy.py`
**To modify visualization**: Edit `ui/renderer.py`
**To add scenarios**: Edit `main.py`

## Pro Tips

1. **Start simple**: Use SCENE_SMALL for faster testing
2. **Watch collision zones**: Yellow areas show where robots coordinate
3. **Compare strategies**: Option 4 shows which works best for your scenario
4. **Use custom mode**: Option 7 lets you control weld count
5. **Save results**: Statistics are printed to console (redirect to file if needed)

## Getting Help

If something doesn't work:

1. Run `python test_system.py` to verify installation
2. Check console output for error messages
3. Review README.md for detailed documentation
4. Examine example scenarios in main.py

Happy welding! 🔥
