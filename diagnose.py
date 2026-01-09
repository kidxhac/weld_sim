#!/usr/bin/env python3
"""
Diagnostic script to identify simulation issues
"""

import sys
sys.path.insert(0, '/mnt/user-data/outputs/weld_sim')

from config.scene import SCENE
from planner.data_model import Weld, Robot, Gantry, SimulationState, WeldMode
from planner.weld_planner import WeldPlanner
from planner.collision_rules import create_standard_collision_zones, CollisionManager
from simulator.simulator import Simulator
import copy

def diagnose():
    print("\n" + "="*80)
    print("DIAGNOSTIC TEST - IDENTIFYING ISSUES")
    print("="*80 + "\n")
    
    # Create robots
    print("1. Creating robots...")
    robots = []
    for r_config in SCENE["robots"]:
        robot = Robot(
            id=r_config["id"],
            side=r_config["side"],
            y_range=r_config["y_range"],
            tcp_speed=r_config["tcp_speed"]
        )
        robots.append(robot)
        print(f"   {robot.id}: side={robot.side}, y_range={robot.y_range}, y={robot.current_y:.0f}")
    
    # Create gantry
    print("\n2. Creating gantry...")
    gantry = Gantry(
        x=0.0,
        speed=SCENE["gantry"]["x_speed"],
        x_length=SCENE["gantry"]["x_length"]
    )
    print(f"   Gantry at X={gantry.x:.0f}, speed={gantry.speed:.0f}mm/s")
    
    # Create simple welds
    print("\n3. Creating test welds...")
    welds = [
        Weld(id=1, x_start=500, x_end=1300, y=600, side="x_plus"),   # Short for SAW
        Weld(id=2, x_start=1500, x_end=2200, y=1400, side="x_plus"), # Short for SAW
        Weld(id=3, x_start=800, x_end=1400, y=400, side="x_minus"),  # Short for SAW
        Weld(id=4, x_start=2000, x_end=2800, y=1600, side="x_minus"),# Short for SAW
    ]
    for w in welds:
        print(f"   W{w.id}: X=[{w.x_start:.0f}, {w.x_end:.0f}], Y={w.y:.0f}, "
              f"side={w.side}, length={w.length:.0f}mm")
    
    # Test SAW planner
    print("\n4. Testing SAW planner...")
    planner = WeldPlanner(welds, robots, SCENE)
    saw_plan = planner.plan_saw()
    
    print(f"\n   SAW Plan created:")
    print(f"   - {len(saw_plan.tasks)} tasks")
    print(f"   - {len(saw_plan.saw_stops)} stops at: {[f'{s:.0f}' for s in saw_plan.saw_stops]}")
    print(f"   - Estimated time: {saw_plan.estimated_total_time:.1f}s")
    
    print(f"\n   Task details:")
    for i, task in enumerate(saw_plan.tasks):
        print(f"   Task {i+1}: {task.robot_id} -> W{task.weld.id} at stop X={task.start_x:.1f}, Y={task.y_position:.1f}")
    
    # Create simulation
    print("\n5. Creating simulation...")
    state_robots = copy.deepcopy(robots)
    state = SimulationState(
        time=0.0,
        gantry=gantry,
        robots=state_robots,
        welds=welds,
        plan=saw_plan
    )
    
    print(f"   State created with {len(state.robots)} robots")
    for r in state.robots:
        print(f"   {r.id}: Y={r.current_y:.0f}, state={r.state.value}")
    
    collision_zones = create_standard_collision_zones(SCENE)
    collision_manager = CollisionManager(collision_zones)
    
    simulator = Simulator(saw_plan, state, collision_manager, dt=0.1)
    
    # Run a few steps
    print("\n6. Running 100 simulation steps...")
    for i in range(100):
        if not simulator.step():
            print(f"   Simulation stopped at step {i+1}")
            break
        if i % 10 == 0 or simulator.state.gantry.x > 720:
            print(f"   Step {i}: t={simulator.state.time:.1f}s, gantry X={simulator.state.gantry.x:.1f}, at_stop={simulator.at_stop}")
            for r in simulator.state.robots:
                if r.state.value != "IDLE" or r.welds_completed > 0:
                    print(f"      {r.id}: {r.state.value}, Y={r.current_y:.0f}, queue={len(r.weld_queue)}, completed={r.welds_completed}")
    
    print(f"\n7. Final status:")
    print(f"   Time: {simulator.state.time:.1f}s")
    print(f"   Gantry X: {simulator.state.gantry.x:.1f}mm")
    print(f"   Progress: {simulator.get_progress():.1f}%")
    for r in simulator.state.robots:
        print(f"   {r.id}: {r.state.value}, Y={r.current_y:.0f}, welds={r.welds_completed}")
    
    print("\n" + "="*80)
    print("DIAGNOSIS COMPLETE")
    print("="*80 + "\n")

if __name__ == "__main__":
    try:
        diagnose()
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
