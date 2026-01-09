#!/usr/bin/env python3
"""
Test scenario demonstrating new advanced features:
1. Collision zone work splitting
2. Bidirectional WOM with uneven robot workload
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


def test_collision_zone_splitting():
    """
    Test Scenario 1: Collision Zone Work Splitting
    
    Setup:
    - R1 has heavy workload outside collision zone
    - R3 has light workload outside collision zone  
    - Long weld in collision zone s1
    
    Expected: System splits the collision zone weld to balance workload
    """
    print("\n" + "="*80)
    print("TEST 1: COLLISION ZONE WORK SPLITTING")
    print("="*80 + "\n")
    
    # Create robots
    robots = [Robot(id=r['id'], side=r['side'], y_range=r['y_range'], 
                    tcp_speed=r['tcp_speed']) for r in SCENE['robots']]
    
    gantry = Gantry(x=0.0, speed=SCENE['gantry']['x_speed'], 
                   x_length=SCENE['gantry']['x_length'])
    
    # Create welds with imbalanced workload
    welds = [
        # R1 area (Y=600): Heavy workload
        Weld(id=1, x_start=500, x_end=2500, y=600, side="x_plus"),   # 2000mm
        Weld(id=2, x_start=500, x_end=2000, y=700, side="x_plus"),   # 1500mm
        
        # Collision zone s1 (Y=800-1200): Long weld to be split
        Weld(id=3, x_start=500, x_end=2500, y=1000, side="x_plus"),  # 2000mm IN s1!
        
        # R3 area (Y=1400): Light workload
        Weld(id=4, x_start=500, x_end=1500, y=1400, side="x_plus"),  # 1000mm
    ]
    
    print("Initial workload distribution:")
    print("  R1 area (Y<800): 3500mm (W1 + W2)")
    print("  Collision zone s1 (Y=800-1200): 2000mm (W3) - TO BE SPLIT")
    print("  R3 area (Y>1200): 1000mm (W4)")
    print("  Imbalance: R1 would get 5500mm vs R3 would get 1000mm (5.5x difference!)")
    print("\nExpected: W3 should be split to balance loads\n")
    
    # Plan with collision zone splitting enabled
    planner = WeldPlanner(welds, robots, SCENE)
    plan = planner.plan(mode=WeldMode.WOM)
    
    print("\n" + planner.get_task_summary(plan))
    
    # Show split results
    print("\nAnalyzing split results:")
    r1_tasks = [t for t in plan.tasks if t.robot_id == "R1"]
    r3_tasks = [t for t in plan.tasks if t.robot_id == "R3"]
    
    r1_total = sum(t.weld.length for t in r1_tasks)
    r3_total = sum(t.weld.length for t in r3_tasks)
    
    print(f"  R1 final workload: {r1_total:.0f}mm ({len(r1_tasks)} tasks)")
    print(f"  R3 final workload: {r3_total:.0f}mm ({len(r3_tasks)} tasks)")
    print(f"  Balance ratio: {max(r1_total, r3_total) / min(r1_total, r3_total):.2f}x")
    
    if abs(r1_total - r3_total) < 500:
        print("  ✓ WELL BALANCED!")
    else:
        print("  ✗ Still imbalanced")
    
    return plan, robots, gantry, welds


def test_bidirectional_wom():
    """
    Test Scenario 2: Bidirectional WOM with Uneven Robot Participation
    
    Setup:
    - Group 1: R1, R2, R3 all have welds (3 robots)
    - Group 2: Only R4 has weld (1 robot)
    - Group 3: R1 and R2 have welds (2 robots)
    
    Expected: Gantry moves forward/backward, allowing 1-3 robots to work
    """
    print("\n" + "="*80)
    print("TEST 2: BIDIRECTIONAL WOM - UNEVEN ROBOT PARTICIPATION")
    print("="*80 + "\n")
    
    robots = [Robot(id=r['id'], side=r['side'], y_range=r['y_range'],
                    tcp_speed=r['tcp_speed']) for r in SCENE['robots']]
    
    gantry = Gantry(x=0.0, speed=SCENE['gantry']['x_speed'],
                   x_length=SCENE['gantry']['x_length'])
    
    # Create welds with uneven distribution
    welds = [
        # Group 1 (X: 500-2500): 3 robots work
        Weld(id=1, x_start=500, x_end=2500, y=600, side="x_plus"),    # R1: 2000mm
        Weld(id=2, x_start=500, x_end=2300, y=1400, side="x_plus"),   # R3: 1800mm
        Weld(id=3, x_start=500, x_end=2000, y=400, side="x_minus"),   # R2: 1500mm
        
        # Group 2 (X: 3000-3500): Only R4 works!
        Weld(id=4, x_start=3000, x_end=3500, y=1600, side="x_minus"), # R4: 500mm ALONE
        
        # Group 3 (X: 4000-4800): 2 robots work
        Weld(id=5, x_start=4000, x_end=4800, y=700, side="x_plus"),   # R1: 800mm
        Weld(id=6, x_start=4000, x_end=4600, y=500, side="x_minus"),  # R2: 600mm
    ]
    
    print("Weld distribution:")
    print("  Group 1 (X: 500-2500mm): R1, R2, R3 work (3 robots)")
    print("  Group 2 (X: 3000-3500mm): R4 works ALONE (1 robot)")
    print("  Group 3 (X: 4000-4800mm): R1, R2 work (2 robots)")
    print("\nTraditional WOM would require all 4 robots in each window.")
    print("Bidirectional WOM allows flexible participation!\n")
    
    # Plan with bidirectional WOM
    planner = WeldPlanner(welds, robots, SCENE)
    plan = planner.plan(mode=WeldMode.WOM)
    
    print("\n" + planner.get_task_summary(plan))
    
    # Analyze robot participation
    print("\nRobot participation analysis:")
    for window_idx, window in enumerate(plan.wom_windows):
        active_robots = set(t.robot_id for t in window)
        print(f"  Window {window_idx + 1}: {len(active_robots)} robots - {sorted(active_robots)}")
    
    return plan, robots, gantry, welds


def run_test_with_visualization(test_func, test_name):
    """Run test and show visualization"""
    plan, robots, gantry, welds = test_func()
    
    # Create simulation
    state_robots = copy.deepcopy(robots)
    state = SimulationState(
        time=0.0,
        gantry=gantry,
        robots=state_robots,
        welds=welds,
        plan=plan
    )
    
    collision_zones = create_standard_collision_zones(SCENE)
    collision_manager = CollisionManager(collision_zones)
    
    simulator = Simulator(plan, state, collision_manager, dt=0.05)
    
    # Create and show renderer
    renderer = Renderer(simulator, SCENE)
    renderer.show(interval=20)


if __name__ == "__main__":
    print("\n" + "="*80)
    print(" " * 15 + "ADVANCED FEATURES TEST SUITE")
    print("="*80)
    print("\nSelect test:")
    print("  1. Collision Zone Work Splitting")
    print("  2. Bidirectional WOM (Uneven Participation)")
    print("  3. Run Both (Sequential)")
    print("")
    
    choice = input("Enter choice (1-3) [default=1]: ").strip() or "1"
    
    if choice == "1":
        run_test_with_visualization(test_collision_zone_splitting, 
                                   "Collision Zone Work Splitting")
    elif choice == "2":
        run_test_with_visualization(test_bidirectional_wom,
                                   "Bidirectional WOM")
    elif choice == "3":
        # Run test 1
        print("\n\nRunning Test 1...")
        plan1, robots1, gantry1, welds1 = test_collision_zone_splitting()
        input("\nPress Enter to continue to Test 2...")
        
        # Run test 2
        print("\n\nRunning Test 2...")
        run_test_with_visualization(test_bidirectional_wom,
                                   "Bidirectional WOM")
    else:
        print("Invalid choice")
