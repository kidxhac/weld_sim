# main.py
"""
Main entry point for welding simulation.

Provides multiple test scenarios:
1. WOM-only (long parallel welds)
2. SAW-only (short scattered welds)
3. Hybrid (mixed weld lengths)
4. Strategy comparison
"""

import random
from config.scene import SCENE, SCENE_SMALL, SCENE_LARGE
from planner.data_model import Weld, Robot, Gantry, WeldMode, SimulationState
from planner.weld_planner import WeldPlanner
from planner.collision_rules import create_standard_collision_zones, CollisionManager
from simulator.simulator import Simulator
from ui.renderer import Renderer


def create_robots_from_config(scene_config: dict) -> list:
    """Create Robot objects from scene configuration"""
    robots = []
    for r_config in scene_config["robots"]:
        robot = Robot(
            id=r_config["id"],
            side=r_config["side"],
            y_range=r_config["y_range"],
            tcp_speed=r_config["tcp_speed"]
        )
        robots.append(robot)
    return robots


def create_gantry_from_config(scene_config: dict) -> Gantry:
    """Create Gantry object from scene configuration"""
    gantry_config = scene_config["gantry"]
    return Gantry(
        x=0.0,
        speed=gantry_config["x_speed"],
        x_length=gantry_config["x_length"]
    )


def generate_wom_test_welds(scene_config: dict) -> list:
    """
    Generate test welds suitable for WOM mode.
    Long welds parallel to X-axis.
    """
    welds = []
    weld_config = scene_config["weld_config"]["x_long"]
    
    weld_id = 1
    for i, y_pos in enumerate(weld_config["y_positions"]):
        # Determine side (alternate or based on Y position)
        side = "x_plus" if y_pos < scene_config["gantry"]["y_span"] / 2 else "x_minus"
        
        # Random length within range
        length = random.uniform(weld_config["length"][0], weld_config["length"][1])
        
        # Position along X
        x_start = random.uniform(200, 1000)
        x_end = x_start + length
        
        weld = Weld(
            id=weld_id,
            x_start=x_start,
            x_end=x_end,
            y=y_pos,
            side=side
        )
        welds.append(weld)
        weld_id += 1
    
    return welds


def generate_saw_test_welds(scene_config: dict) -> list:
    """
    Generate test welds suitable for SAW mode.
    Short welds scattered in Y.
    """
    welds = []
    weld_config = scene_config["weld_config"]["y_long"]
    
    weld_id = 1
    for i, y_pos in enumerate(weld_config["y_positions"]):
        side = "x_plus" if i % 2 == 0 else "x_minus"
        
        # Shorter lengths
        length = random.uniform(weld_config["length"][0], weld_config["length"][1])
        
        # Random X position
        x_start = random.uniform(500, 4000)
        x_end = x_start + length
        
        weld = Weld(
            id=weld_id,
            x_start=x_start,
            x_end=x_end,
            y=y_pos,
            side=side
        )
        welds.append(weld)
        weld_id += 1
    
    return welds


def generate_hybrid_test_welds(scene_config: dict) -> list:
    """
    Generate mixed welds: some long (WOM), some short (SAW).
    """
    welds = []
    weld_id = 1
    
    # Add some long welds
    for i in range(3):
        y = 400 + i * 400
        side = "x_plus" if i % 2 == 0 else "x_minus"
        length = random.uniform(2000, 3500)
        x_start = random.uniform(300, 1000)
        
        welds.append(Weld(
            id=weld_id,
            x_start=x_start,
            x_end=x_start + length,
            y=y,
            side=side
        ))
        weld_id += 1
    
    # Add some short welds
    for i in range(5):
        y = 200 + i * 300
        side = "x_minus" if i % 2 == 0 else "x_plus"
        length = random.uniform(600, 1000)
        x_start = random.uniform(1000, 4000)
        
        welds.append(Weld(
            id=weld_id,
            x_start=x_start,
            x_end=x_start + length,
            y=y,
            side=side
        ))
        weld_id += 1
    
    return welds


def run_scenario(scenario_name: str, welds: list, robots: list, 
                 gantry: Gantry, scene_config: dict, 
                 mode: WeldMode = None, compare: bool = False):
    """
    Run a complete simulation scenario.
    
    Args:
        scenario_name: Name for display
        welds: List of welds to complete
        robots: List of robots
        gantry: Gantry system
        scene_config: Scene configuration
        mode: Force specific mode or None for auto
        compare: If True, compare all strategies
    """
    print("\n" + "="*80)
    print(f"SCENARIO: {scenario_name}")
    print("="*80)
    print(f"Welds: {len(welds)}")
    print(f"Robots: {len(robots)}")
    print(f"Gantry: {gantry.x_length:.0f}mm @ {gantry.speed:.0f}mm/s")
    print("")
    
    # Create planner
    planner = WeldPlanner(welds, robots, scene_config)
    
    if compare:
        # Compare all strategies
        results = planner.compare_strategies()
        
        # Use best strategy
        best_strategy = results.get('best', 'hybrid')
        plan = results[best_strategy]['plan']
    else:
        # Create plan with specified mode
        plan = planner.plan(mode)
    
    print(planner.get_task_summary(plan))
    
    # Create simulation state
    state = SimulationState(
        time=0.0,
        gantry=gantry,
        robots=robots,
        welds=welds,
        plan=plan
    )
    
    # Create collision manager
    collision_zones = create_standard_collision_zones(scene_config)
    collision_manager = CollisionManager(collision_zones)
    
    # Create simulator
    simulator = Simulator(plan, state, collision_manager, dt=0.05)
    
    # Create renderer
    renderer = Renderer(simulator, scene_config)
    
    # Run visualization
    renderer.show(interval=20)


def main():
    """Main entry point with scenario selection"""
    
    print("\n" + "="*80)
    print(" " * 20 + "WELDING SIMULATION SYSTEM")
    print("="*80)
    print("\nSelect scenario:")
    print("  1. WOM Test (Long parallel welds)")
    print("  2. SAW Test (Short scattered welds)")
    print("  3. Hybrid Test (Mixed weld lengths)")
    print("  4. Strategy Comparison (Hybrid with comparison)")
    print("  5. Small Workspace (Quick test)")
    print("  6. Large Workspace (Complex scenario)")
    print("  7. Custom Scenario")
    print("")
    
    choice = input("Enter choice (1-7) [default=3]: ").strip() or "3"
    
    if choice == "1":
        # WOM test
        scene = SCENE
        welds = generate_wom_test_welds(scene)
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("WOM Test - Long Parallel Welds", welds, robots, gantry, 
                    scene, mode=WeldMode.WOM)
    
    elif choice == "2":
        # SAW test
        scene = SCENE
        welds = generate_saw_test_welds(scene)
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("SAW Test - Short Scattered Welds", welds, robots, gantry, 
                    scene, mode=WeldMode.SAW)
    
    elif choice == "3":
        # Hybrid test
        scene = SCENE
        welds = generate_hybrid_test_welds(scene)
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("Hybrid Test - Mixed Welds", welds, robots, gantry, 
                    scene, mode=None)
    
    elif choice == "4":
        # Strategy comparison
        scene = SCENE
        welds = generate_hybrid_test_welds(scene)
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("Strategy Comparison", welds, robots, gantry, 
                    scene, compare=True)
    
    elif choice == "5":
        # Small workspace
        scene = SCENE_SMALL
        welds = generate_hybrid_test_welds(scene)
        # Scale down weld positions
        for w in welds:
            w.y = w.y * 0.8
            w.x_start = w.x_start * 0.5
            w.x_end = w.x_end * 0.5
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("Small Workspace Test", welds, robots, gantry, scene)
    
    elif choice == "6":
        # Large workspace
        scene = SCENE_LARGE
        welds = generate_hybrid_test_welds(scene)
        # Add more welds
        welds.extend(generate_wom_test_welds(scene)[:3])
        welds.extend(generate_saw_test_welds(scene)[:4])
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("Large Workspace Test", welds, robots, gantry, scene)
    
    elif choice == "7":
        # Custom scenario
        print("\nCustom Scenario Builder")
        scene = SCENE
        
        num_long = int(input("Number of long welds (WOM): ") or "2")
        num_short = int(input("Number of short welds (SAW): ") or "3")
        
        welds = []
        weld_id = 1
        
        # Generate long welds
        for i in range(num_long):
            y = 400 + i * 500
            side = "x_plus" if i % 2 == 0 else "x_minus"
            length = random.uniform(2000, 3500)
            x_start = random.uniform(200, 800)
            
            welds.append(Weld(
                id=weld_id,
                x_start=x_start,
                x_end=x_start + length,
                y=y,
                side=side
            ))
            weld_id += 1
        
        # Generate short welds
        for i in range(num_short):
            y = 300 + i * 400
            side = "x_minus" if i % 2 == 0 else "x_plus"
            length = random.uniform(600, 1200)
            x_start = random.uniform(1000, 4000)
            
            welds.append(Weld(
                id=weld_id,
                x_start=x_start,
                x_end=x_start + length,
                y=y,
                side=side
            ))
            weld_id += 1
        
        robots = create_robots_from_config(scene)
        gantry = create_gantry_from_config(scene)
        run_scenario("Custom Scenario", welds, robots, gantry, scene)
    
    else:
        print("Invalid choice. Running default scenario (Hybrid Test).")
        main()


if __name__ == "__main__":
    # Set random seed for reproducibility
    random.seed(42)
    
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
