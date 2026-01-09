# Welding Simulation Algorithms - Detailed Explanation

## Overview

This document provides detailed explanations of the two main welding strategies implemented in the simulation system: **Weld-on-Move (WOM)** and **Stop-and-Weld (SAW)**.

---

## 1. Weld-on-Move (WOM) Algorithm

### Purpose
Optimize welding of long, parallel seams along the X-axis by keeping robots at fixed Y positions while the gantry moves continuously.

### When to Use
- Long welds where Lx/Ly > 7 (typically Lx > 1500mm)
- Parallel seams at similar X-positions
- When continuous motion is more efficient than repositioning

### Algorithm Steps

#### Step 1: Weld Classification
```python
def is_wom_suitable(weld):
    length_x = abs(weld.x_end - weld.x_start)
    return length_x > 1500  # Threshold for long welds
```

#### Step 2: Window Grouping
Group welds that can be welded simultaneously into "windows":

```
For each weld:
    If similar length to window base (within 30%)
    AND similar X-span (>70% overlap):
        Add to current window
    Else:
        Create new window
```

**Similarity Criteria:**
- Length ratio: `|(L1 - L2) / L1| < 0.3`
- X-overlap: `overlap_length / shorter_weld_length > 0.7`

**Example:**
```
Window 1: [W1(500-3000), W2(800-3200)]  # Similar lengths, overlapping X
Window 2: [W3(4000-5500)]                 # Different X position
```

#### Step 3: Robot Assignment
Assign welds to robots based on Y-position and workspace:

```
For each weld in window:
    Find robot with:
        - Matching side (x_plus or x_minus)
        - Y-range contains weld.y
        - Closest workspace_center to weld.y
    Assign weld to that robot
```

#### Step 4: Y-Position Optimization
Calculate optimal fixed Y position for each robot:

```
optimal_y = average(all_assigned_welds.y)
Clamp to robot's y_range [y_min, y_max]
```

#### Step 5: Collision Resolution
Adjust Y positions if robots would collide:

```
For each collision zone:
    If both robots in zone AND distance < safe_distance:
        midpoint = (y1 + y2) / 2
        y1 = midpoint - safe_distance/2 - buffer
        y2 = midpoint + safe_distance/2 + buffer
```

#### Step 6: Execution Sequence

```
For each window:
    Phase 1: Position robots at target Y coordinates
        While any robot not at target:
            Move robot towards target_y
    
    Phase 2: Acquire collision locks and start welding
        For each robot:
            If in collision zone:
                Try to acquire mutex lock
                If successful: start welding
                Else: wait
    
    Phase 3: Move gantry and weld
        While any weld incomplete:
            gantry.x += gantry.speed * dt
            For each welding robot:
                weld.done += robot.tcp_speed * dt
    
    Phase 4: Complete window
        Release all locks
        Move to next window
```

### Time Estimation

```
total_time = Σ(window_time) + repositioning_time

window_time = max(weld.length for weld in window) / gantry.speed
repositioning_time = 5 seconds per window transition
```

### Advantages
- **No stop/start cycles**: Continuous gantry motion
- **Maximum parallelization**: All robots work simultaneously
- **Predictable timing**: Based on longest weld in window
- **Efficient for long welds**: Minimizes overhead

### Limitations
- **Fixed Y positions**: Can't handle varying Y requirements
- **Requires similar lengths**: Inefficient if lengths vary greatly
- **Limited flexibility**: Not suitable for complex patterns

---

## 2. Stop-and-Weld (SAW) Algorithm

### Purpose
Handle compact workpieces with varying weld positions by stopping the gantry and letting robots move independently.

### When to Use
- Short welds (Lx ≤ 1500mm)
- Welds scattered across Y-axis
- Complex weld patterns requiring repositioning
- When Lx/Ly < 7

### Algorithm Steps

#### Step 1: Weld Classification
```python
def is_saw_suitable(weld):
    length_x = abs(weld.x_end - weld.x_start)
    return length_x <= 1500  # Threshold for short welds
```

#### Step 2: Determine Gantry Stops
Calculate optimal X positions where gantry should stop:

```
x_positions = extract all x_start and x_end from welds
x_min = min(x_positions)
x_max = max(x_positions)
x_range = x_max - x_min

If x_range <= max_stop_spacing (500mm):
    stops = [(x_min + x_max) / 2]  # Single stop
Else:
    num_stops = ceiling(x_range / max_stop_spacing)
    stops = [x_min + (i+0.5) * x_range / num_stops 
             for i in range(num_stops)]
```

**Example:**
```
Welds at X: [500, 1000, 2500, 3500, 4000]
x_range = 3500
num_stops = ceiling(3500 / 500) = 7
stops = [750, 1250, 1750, 2250, 2750, 3250, 3750]
```

#### Step 3: Assign Welds to Stops
Map each weld to reachable stop(s):

```
reach_distance = 400mm  # Robot reach from gantry position

For each weld:
    weld_x_center = (weld.x_start + weld.x_end) / 2
    For each stop:
        If |weld_x_center - stop| <= reach_distance:
            assignments[stop].append(weld)
```

#### Step 4: Schedule Tasks at Each Stop
Optimize robot task assignment using greedy algorithm:

```
For welds at this stop:
    Sort by Y position
    
    For each weld:
        Find best robot:
            - Must be able to reach weld.y (in y_range)
            - Minimize: distance + current_workload * 10
        
        Assign weld to best robot
        Update robot's workload
```

**Scoring Function:**
```
score = |weld.y - robot.workspace_center| + robot.current_workload * 10
Select robot with minimum score
```

#### Step 5: Collision-Aware Sequencing
Order tasks to minimize collision zone conflicts:

```
For each robot's tasks:
    Sort by distance from current position (minimize travel)
    
    For tasks in collision zones:
        Assign priority based on collision_manager.priority_robot
        High-priority tasks execute first
```

#### Step 6: Execution Sequence

```
For each stop_x in stops:
    Phase 1: Move gantry to stop
        While |gantry.x - stop_x| > threshold:
            gantry.x += gantry.speed * dt
    
    Phase 2: Execute robot tasks
        For each robot with tasks at this stop:
            While robot has queued tasks:
                next_weld = robot.weld_queue[0]
                
                Sub-phase A: Move to weld position
                    While |robot.y - next_weld.y| > threshold:
                        robot.y += direction * robot.speed * dt
                
                Sub-phase B: Acquire lock and weld
                    If in collision zone:
                        If try_acquire_lock(robot, position):
                            weld the seam
                        Else:
                            wait in queue
                    Else:
                        weld the seam (no lock needed)
                
                Sub-phase C: Complete weld
                    weld.done += robot.tcp_speed * dt
                    If weld.done >= weld.length:
                        release_lock()
                        remove from queue
                        robot.state = IDLE
    
    Phase 3: Move to next stop
        Repeat for next stop_x
```

### Time Estimation

```
gantry_travel_time = Σ(distance between stops) / gantry.speed

For each stop:
    robot_times = {}
    For each task at stop:
        move_time = |task.y - robot.current_y| / robot.y_speed
        weld_time = task.weld.length / robot.tcp_speed
        robot_times[robot] += move_time + weld_time
    
    stop_work_time = max(robot_times.values())

total_time = gantry_travel_time + Σ(stop_work_time)
```

### Advantages
- **Flexible positioning**: Handles any weld pattern
- **Independent robots**: Each optimizes its own path
- **Efficient for scattered welds**: Good Y-coverage
- **Load balancing**: Work distributed based on reachability

### Limitations
- **Stop/start overhead**: Time lost in gantry positioning
- **Sequential constraints**: Collision zones can cause waits
- **Less parallelization**: Robots may idle waiting for gantry

---

## 3. Collision Management System

### Collision Zones
Two zones where robots share workspace:
- **s1**: R1 and R3 (x_plus side)
- **s2**: R2 and R4 (x_minus side)

### Mutex Lock System

```python
class Mutex:
    owner = None  # Which robot holds the lock
    
    def try_acquire(robot_id, y_position):
        if y_position not in collision_zone:
            return True  # No lock needed
        
        if owner is None:
            owner = robot_id
            return True
        
        return owner == robot_id
    
    def release(robot_id):
        if owner == robot_id:
            owner = None
```

### Priority System
When both robots need same zone:
1. **R1 has priority over R3**
2. **R2 has priority over R4**

Lower-priority robot waits until zone is free.

### Safe Distance
Minimum 150mm separation maintained between robots in same zone.

### Collision Resolution Algorithm

```
For each time step:
    active_collisions = []
    
    For each zone:
        robots_in_zone = find robots with y in zone.y_range
        
        If len(robots_in_zone) >= 2:
            For each pair:
                If distance < safe_distance:
                    active_collisions.append(pair)
    
    For each collision:
        priority_robot, other_robot = determine_priority(pair)
        
        If priority_robot welding:
            other_robot.state = WAIT_MUTEX
        Else if other_robot has priority task:
            swap priority
```

---

## 4. Hybrid Strategy

Combines WOM and SAW by classifying welds:

```
wom_welds = [w for w in welds if is_wom_suitable(w)]
saw_welds = [w for w in welds if is_saw_suitable(w)]

# Execute WOM welds first (typically more efficient)
wom_plan = wom_strategy.create_plan(wom_welds)
saw_plan = saw_strategy.create_plan(saw_welds)

total_time = wom_plan.time + saw_plan.time
return combined_plan
```

### Decision Tree

```
For each weld:
    If length_x > 1500mm:
        Use WOM
    Else:
        Use SAW

Group WOM welds into windows
Group SAW welds by X-position
Execute in optimal sequence
```

---

## 5. Performance Optimization Techniques

### 1. Window Grouping Optimization
- Group similar welds to maximize parallel execution
- Avoid frequent gantry repositioning
- Balance workload across robots

### 2. Robot Assignment Optimization
```
# Greedy assignment with lookahead
score = distance + future_workload_penalty
```

### 3. Collision Avoidance
- Adjust Y positions proactively
- Use priority system to prevent deadlocks
- Minimize wait time through smart scheduling

### 4. Path Planning
```
# For SAW mode, order tasks by distance
tasks.sort(key=lambda t: abs(t.y - robot.current_y))
```

---

## 6. Implementation Notes

### Data Structures

**Weld:**
- Position: (x_start, x_end, y)
- State: done (progress in mm)
- Metadata: id, side, length

**Robot:**
- Position: current_y
- State: IDLE | MOVING_Y | WELDING | WAIT_MUTEX
- Constraints: y_range, side, tcp_speed

**Gantry:**
- Position: x
- State: is_moving
- Target: target_x

### Simulation Time Step

```
dt = 0.05 seconds  # 50ms time step

For each dt:
    Update gantry position
    Update robot positions
    Update weld progress
    Check collision zones
    Update state machine
```

### State Machine

```
IDLE → MOVING_Y → WELDING → IDLE
              ↓
         WAIT_MUTEX → WELDING
```

---

## 7. Comparison Example

### Scenario: 4 welds of varying lengths

**Welds:**
- W1: 2500mm at Y=600 (long)
- W2: 2400mm at Y=1400 (long)
- W3: 800mm at Y=400 (short)
- W4: 700mm at Y=1600 (short)

**WOM Strategy:**
- Window 1: W1, W2 (similar length, parallel)
- Time: max(2500, 2400) / 300 = 8.3s
- Window 2: W3, W4
- Time: max(800, 700) / 300 = 2.7s
- Total: 8.3 + 2.7 + 10s (repositioning) = **21.0s**

**SAW Strategy:**
- Stop 1 @ X=1500: W1 partial, W3
- Stop 2 @ X=2500: W1 complete, W2 partial
- Stop 3 @ X=3500: W2 complete, W4
- Total: 15s (gantry) + 18s (welding) = **33.0s**

**Winner: WOM** (36% faster for this scenario)

---

## 8. Future Enhancements

1. **Dynamic replanning**: Adjust strategy mid-execution
2. **Machine learning**: Learn optimal parameters from history
3. **Multi-objective optimization**: Balance time, quality, energy
4. **Predictive collision avoidance**: Forecast conflicts before they occur
5. **Adaptive windowing**: Dynamic window size based on real-time conditions

---

## Conclusion

The WOM and SAW algorithms provide complementary approaches to multi-robot welding:

- **WOM** excels at long, parallel welds with continuous motion
- **SAW** handles complex patterns with flexible positioning
- **Hybrid** combines both for optimal overall performance

The collision management system ensures safe operation in shared workspaces while maximizing parallelization and minimizing wait times.
