#!/usr/bin/env python3
"""
Quick WOM visualization test
"""

import sys
sys.path.insert(0, '/mnt/user-data/outputs/weld_sim')

from config.scene import SCENE
from planner.data_model import Weld, Robot, Gantry, SimulationState, WeldMode
from planner.weld_planner import WeldPlanner
from planner.collision_rules import create_standard_collision_zones, CollisionManager
from simulator.simulator import Simulator
from ui.renderer import Renderer
import copy


def main():
    print("\n" + "="*80)
    print(" " * 25 + "WOM MODE VISUAL TEST")
    print("="*80 + "\n")
    
    # Create robots
    robots = [Robot(id=r['id'], side=r['side'], y_range=r['y_range'], 
                    tcp_speed=r['tcp_speed']) for r in SCENE['robots']]
    
    gantry = Gantry(x=0.0, speed=SCENE['gantry']['x_speed'], 
                   x_length=SCENE['gantry']['x_length'])
    
    # Create long parallel welds suitable for WOM
    welds = [
        Weld(id=1, x_start=500, x_end=3000, y=600, side="x_plus"),    # 2500mm
        Weld(id=2, x_start=800, x_end=3200, y=1400, side="x_plus"),   # 2400mm
        Weld(id=3, x_start=500, x_end=2800, y=400, side="x_minus"),   # 2300mm
        Weld(id=4, x_start=600, x_end=3000, y=1600, side="x_minus"),  # 2400mm
    ]
    
    print("Test scenario: 4 long parallel welds")
    for w in welds:
        print(f"  W{w.id}: X=[{w.x_start:.0f}, {w.x_end:.0f}], Y={w.y:.0f}, length={w.length:.0f}mm, side={w.side}")
    
    # Create plan
    print("\nCreating WOM plan...")
    planner = WeldPlanner(welds, robots, SCENE)
    plan = planner.plan(mode=WeldMode.WOM)
    
    print(f"\nPlan created:")
    print(f"  Mode: {plan.mode.value}")
    print(f"  Tasks: {len(plan.tasks)}")
    print(f"  Windows: {len(plan.wom_windows)}")
    print(f"  Estimated time: {plan.estimated_total_time:.1f}s")
    
    # Create simulation
    state = SimulationState(
        time=0.0,
        gantry=gantry,
        robots=copy.deepcopy(robots),
        welds=welds,
        plan=plan
    )
    
    collision_zones = create_standard_collision_zones(SCENE)
    collision_manager = CollisionManager(collision_zones)
    
    simulator = Simulator(plan, state, collision_manager, dt=0.05)
    
    # Create renderer and show
    print("\n" + "="*80)
    print("Starting visualization...")
    print("Watch for:")
    print("  - Gantry (blue line) moving left to right along X-axis")
    print("  - 4 robots (circles) at fixed Y positions")
    print("  - Robots change to RED when welding")
    print("  - Green weld lines grow as welding progresses")
    print("\nControls: SPACE=pause/resume, Q=quit")
    print("="*80 + "\n")
    
    renderer = Renderer(simulator, SCENE)
    renderer.show(interval=20)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
