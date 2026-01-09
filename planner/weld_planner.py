# planner/weld_planner.py
"""
Master Weld Planner

Coordinates WOM and SAW strategies to create optimal welding plans.
Minimizes total welding time while handling collision constraints.
"""

from typing import List, Optional
from planner.data_model import Weld, Robot, WeldPlan, WeldMode
from planner.collision_rules import CollisionManager, create_standard_collision_zones
from planner.wom_strategy import WOMStrategy
from planner.saw_strategy import SAWStrategy, HybridStrategy


class WeldPlanner:
    """
    Main welding planner that coordinates different strategies.
    """
    
    def __init__(self, welds: List[Weld], robots: List[Robot], 
                 scene_config: dict, 
                 enable_bidirectional_wom: bool = True,
                 enable_collision_splitting: bool = True):
        """
        Args:
            welds: All welds to be completed
            robots: All available robots
            scene_config: Scene configuration with gantry and collision zones
            enable_bidirectional_wom: Enable bidirectional gantry movement in WOM
            enable_collision_splitting: Enable collision zone work splitting
        """
        self.welds = welds
        self.robots = robots
        self.scene_config = scene_config
        
        # Initialize collision manager
        collision_zones = create_standard_collision_zones(scene_config)
        self.collision_manager = CollisionManager(collision_zones)
        
        # Initialize strategies with new features
        gantry_speed = scene_config["gantry"].get("x_speed", 300.0)
        self.wom_strategy = WOMStrategy(
            self.collision_manager, 
            gantry_speed,
            use_bidirectional=enable_bidirectional_wom,
            use_collision_splitting=enable_collision_splitting
        )
        self.saw_strategy = SAWStrategy(self.collision_manager)
        self.hybrid_strategy = HybridStrategy(self.wom_strategy, self.saw_strategy)
        
        feature_status = []
        if enable_bidirectional_wom:
            feature_status.append("Bidirectional WOM")
        if enable_collision_splitting:
            feature_status.append("Collision Zone Splitting")
        
        features_str = ", ".join(feature_status) if feature_status else "Standard"
        print(f"WeldPlanner initialized: {len(welds)} welds, {len(robots)} robots")
        print(f"Collision zones: {[z.name for z in collision_zones]}")
        print(f"Advanced features: {features_str}")
    
    def plan(self, mode: Optional[WeldMode] = None) -> WeldPlan:
        """
        Create welding plan using specified mode or auto-detect best mode.
        
        Args:
            mode: Force specific mode (WOM/SAW) or None for auto
        
        Returns:
            Complete WeldPlan
        """
        if mode == WeldMode.WOM:
            return self.plan_wom()
        elif mode == WeldMode.SAW:
            return self.plan_saw()
        else:
            return self.plan_hybrid()
    
    def plan_wom(self) -> WeldPlan:
        """Create plan using only Weld-on-Move strategy"""
        print("\n=== Planning with WOM Strategy ===")
        
        # Filter suitable welds
        wom_welds = [w for w in self.welds if self.wom_strategy.is_wom_suitable(w)]
        
        if not wom_welds:
            print("Warning: No welds suitable for WOM mode")
            return WeldPlan(tasks=[], mode=WeldMode.WOM)
        
        print(f"Processing {len(wom_welds)} WOM-suitable welds")
        
        plan = self.wom_strategy.create_wom_plan(wom_welds, self.robots)
        
        print(f"Created {len(plan.tasks)} tasks in {len(plan.wom_windows)} windows")
        print(f"Estimated time: {plan.estimated_total_time:.1f}s")
        
        return plan
    
    def plan_saw(self) -> WeldPlan:
        """Create plan using only Stop-and-Weld strategy"""
        print("\n=== Planning with SAW Strategy ===")
        
        # Filter suitable welds
        saw_welds = [w for w in self.welds if self.saw_strategy.is_saw_suitable(w)]
        
        if not saw_welds:
            print("Warning: No welds suitable for SAW mode")
            return WeldPlan(tasks=[], mode=WeldMode.SAW)
        
        print(f"Processing {len(saw_welds)} SAW-suitable welds")
        
        gantry_length = self.scene_config["gantry"]["x_length"]
        plan = self.saw_strategy.create_saw_plan(saw_welds, self.robots, gantry_length)
        
        print(f"Created {len(plan.tasks)} tasks at {len(plan.saw_stops)} stops")
        print(f"Estimated time: {plan.estimated_total_time:.1f}s")
        
        return plan
    
    def plan_hybrid(self) -> WeldPlan:
        """Create plan using hybrid strategy (WOM + SAW)"""
        print("\n=== Planning with Hybrid Strategy ===")
        
        gantry_length = self.scene_config["gantry"]["x_length"]
        plan = self.hybrid_strategy.create_hybrid_plan(
            self.welds, self.robots, gantry_length
        )
        
        # Count WOM vs SAW tasks
        wom_tasks = [t for t in plan.tasks if t.mode == WeldMode.WOM]
        saw_tasks = [t for t in plan.tasks if t.mode == WeldMode.SAW]
        
        print(f"Created {len(plan.tasks)} tasks:")
        print(f"  - WOM: {len(wom_tasks)} tasks")
        print(f"  - SAW: {len(saw_tasks)} tasks")
        print(f"Estimated time: {plan.estimated_total_time:.1f}s")
        
        return plan
    
    def compare_strategies(self) -> dict:
        """
        Compare all strategies and return performance metrics.
        
        Returns:
            Dictionary with comparison results
        """
        print("\n" + "="*60)
        print("STRATEGY COMPARISON")
        print("="*60)
        
        results = {}
        
        # WOM Strategy
        try:
            wom_plan = self.plan_wom()
            results['wom'] = {
                'plan': wom_plan,
                'time': wom_plan.estimated_total_time,
                'tasks': len(wom_plan.tasks),
                'windows': len(wom_plan.wom_windows)
            }
        except Exception as e:
            print(f"WOM strategy failed: {e}")
            results['wom'] = None
        
        # SAW Strategy
        try:
            saw_plan = self.plan_saw()
            results['saw'] = {
                'plan': saw_plan,
                'time': saw_plan.estimated_total_time,
                'tasks': len(saw_plan.tasks),
                'stops': len(saw_plan.saw_stops)
            }
        except Exception as e:
            print(f"SAW strategy failed: {e}")
            results['saw'] = None
        
        # Hybrid Strategy
        try:
            hybrid_plan = self.plan_hybrid()
            results['hybrid'] = {
                'plan': hybrid_plan,
                'time': hybrid_plan.estimated_total_time,
                'tasks': len(hybrid_plan.tasks)
            }
        except Exception as e:
            print(f"Hybrid strategy failed: {e}")
            results['hybrid'] = None
        
        # Print comparison
        print("\n" + "-"*60)
        print("Results Summary:")
        print("-"*60)
        
        for strategy_name, result in results.items():
            if result:
                print(f"{strategy_name.upper():10s}: {result['time']:6.1f}s  ({result['tasks']} tasks)")
        
        # Find best
        valid_results = {k: v for k, v in results.items() if v is not None}
        if valid_results:
            best_strategy = min(valid_results.items(), key=lambda x: x[1]['time'])
            print(f"\nBest strategy: {best_strategy[0].upper()} ({best_strategy[1]['time']:.1f}s)")
            results['best'] = best_strategy[0]
        
        print("="*60 + "\n")
        
        return results
    
    def get_task_summary(self, plan: WeldPlan) -> str:
        """Generate human-readable summary of a plan"""
        summary = []
        summary.append(f"\n{'='*60}")
        summary.append(f"WELD PLAN SUMMARY - {plan.mode.value.upper()}")
        summary.append(f"{'='*60}")
        summary.append(f"Total tasks: {len(plan.tasks)}")
        summary.append(f"Estimated time: {plan.estimated_total_time:.1f} seconds")
        
        if plan.mode == WeldMode.WOM:
            summary.append(f"WOM windows: {len(plan.wom_windows)}")
            for i, window in enumerate(plan.wom_windows):
                summary.append(f"\n  Window {i+1}: {len(window)} tasks")
                robots_used = set(t.robot_id for t in window)
                summary.append(f"    Robots: {', '.join(sorted(robots_used))}")
        
        elif plan.mode == WeldMode.SAW:
            summary.append(f"Gantry stops: {len(plan.saw_stops)}")
            for i, stop in enumerate(plan.saw_stops):
                stop_tasks = [t for t in plan.tasks if t.start_x == stop]
                summary.append(f"\n  Stop {i+1} @ X={stop:.0f}mm: {len(stop_tasks)} tasks")
        
        # Task breakdown by robot
        summary.append(f"\n{'Task Distribution:':-^60}")
        robot_tasks = {}
        for task in plan.tasks:
            if task.robot_id not in robot_tasks:
                robot_tasks[task.robot_id] = []
            robot_tasks[task.robot_id].append(task)
        
        for robot_id in sorted(robot_tasks.keys()):
            tasks = robot_tasks[robot_id]
            total_time = sum(t.estimated_time for t in tasks)
            summary.append(f"  {robot_id}: {len(tasks)} tasks, {total_time:.1f}s")
        
        summary.append(f"{'='*60}\n")
        
        return '\n'.join(summary)
