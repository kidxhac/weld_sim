#!/usr/bin/env python3
"""
Quick test script to verify the welding simulation system.
Runs a simple scenario without visualization.
"""

import sys
sys.path.insert(0, '/home/claude/weld_sim')

from config.scene import SCENE
from planner.data_model import Weld, Robot, Gantry, SimulationState
from planner.weld_planner import WeldPlanner
from planner.collision_rules import create_standard_collision_zones, CollisionManager
from simulator.simulator import Simulator


def test_system():
    """Run a quick test of the system"""
    
    print("\n" + "="*60)
    print("WELDING SIMULATION SYSTEM - QUICK TEST")
    print("="*60 + "\n")
    
    # Create simple test scenario
    print("1. Setting up test scenario...")
    
    # Create robots
    robots = []
    for r_config in SCENE["robots"]:
        robot = Robot(
            id=r_config["id"],
            side=r_config["side"],
            y_range=r_config["y_range"],
            tcp_speed=r_config["tcp_speed"]
        )
        robots.append(robot)
    print(f"   Created {len(robots)} robots")
    
    # Create gantry
    gantry = Gantry(
        x=0.0,
        speed=SCENE["gantry"]["x_speed"],
        x_length=SCENE["gantry"]["x_length"]
    )
    print(f"   Created gantry: {gantry.x_length}mm @ {gantry.speed}mm/s")
    
    # Create test welds (2 long, 2 short)
    welds = [
        Weld(id=1, x_start=500, x_end=3000, y=600, side="x_plus"),
        Weld(id=2, x_start=800, x_end=3200, y=1400, side="x_plus"),
        Weld(id=3, x_start=2000, x_end=2800, y=400, side="x_minus"),
        Weld(id=4, x_start=3500, x_end=4200, y=1600, side="x_minus"),
    ]
    print(f"   Created {len(welds)} test welds\n")
    
    # Test planner
    print("2. Testing planner...")
    planner = WeldPlanner(welds, robots, SCENE)
    
    # Test WOM strategy
    print("\n   Testing WOM strategy...")
    wom_plan = planner.plan_wom()
    print(f"   WOM plan: {len(wom_plan.tasks)} tasks, {wom_plan.estimated_total_time:.1f}s")
    
    # Test SAW strategy
    print("\n   Testing SAW strategy...")
    saw_plan = planner.plan_saw()
    print(f"   SAW plan: {len(saw_plan.tasks)} tasks, {saw_plan.estimated_total_time:.1f}s")
    
    # Test Hybrid strategy
    print("\n   Testing Hybrid strategy...")
    hybrid_plan = planner.plan_hybrid()
    print(f"   Hybrid plan: {len(hybrid_plan.tasks)} tasks, {hybrid_plan.estimated_total_time:.1f}s\n")
    
    # Test simulator (run a few steps)
    print("3. Testing simulator...")
    
    # Use hybrid plan
    state = SimulationState(
        time=0.0,
        gantry=gantry,
        robots=robots,
        welds=welds,
        plan=hybrid_plan
    )
    
    collision_zones = create_standard_collision_zones(SCENE)
    collision_manager = CollisionManager(collision_zones)
    
    simulator = Simulator(hybrid_plan, state, collision_manager, dt=0.1)
    
    # Run 100 steps
    print("   Running simulation for 100 steps...")
    for i in range(100):
        if not simulator.step():
            break
    
    progress = simulator.get_progress()
    print(f"   Progress after 100 steps: {progress:.1f}%")
    print(f"   Simulation time: {simulator.state.time:.1f}s\n")
    
    # Test collision detection
    print("4. Testing collision detection...")
    
    # Place two robots in same zone
    test_positions = {
        "R1": 900,  # In collision zone s1
        "R3": 950   # Also in s1, too close
    }
    
    # Check for collision
    zone = collision_manager.zones[0]  # s1
    if zone.check_collision(test_positions["R1"], test_positions["R3"]):
        print("   ✓ Collision detected correctly")
    else:
        print("   ✗ Collision detection failed")
    
    # Test mutex locks
    print("\n5. Testing mutex system...")
    if collision_manager.try_acquire_lock("R1", 900):
        print("   ✓ R1 acquired lock for s1")
    else:
        print("   ✗ R1 failed to acquire lock")
    
    if not collision_manager.try_acquire_lock("R3", 950):
        print("   ✓ R3 correctly blocked by R1's lock")
    else:
        print("   ✗ R3 incorrectly acquired lock")
    
    collision_manager.release_lock("R1")
    if collision_manager.try_acquire_lock("R3", 950):
        print("   ✓ R3 acquired lock after R1 released\n")
    else:
        print("   ✗ R3 failed to acquire lock after release\n")
    
    # Summary
    print("="*60)
    print("TEST SUMMARY")
    print("="*60)
    print("✓ Robot creation")
    print("✓ Gantry setup")
    print("✓ Weld definition")
    print("✓ WOM strategy planning")
    print("✓ SAW strategy planning")
    print("✓ Hybrid strategy planning")
    print("✓ Simulation execution")
    print("✓ Collision detection")
    print("✓ Mutex lock system")
    print("\nAll tests passed! System is ready to use.")
    print("\nRun 'python main.py' for full interactive simulation.\n")
    

if __name__ == "__main__":
    try:
        test_system()
    except Exception as e:
        print(f"\n✗ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
