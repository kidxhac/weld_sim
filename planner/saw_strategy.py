# planner/saw_strategy.py
"""
Stop-and-Weld (SAW) Strategy

Key concept: For compact pieces (Lx/Ly < 7), stop gantry at discrete positions
and let robots move independently in Y to complete welds.

Algorithm:
1. Determine optimal gantry stop positions along X-axis
2. At each stop, assign welds to robots based on Y reachability
3. Schedule robot movements to minimize wait time
4. Handle collision zones with priority-based execution
"""

from typing import List, Dict, Tuple, Optional
from planner.data_model import Weld, Robot, WeldTask, WeldMode, WeldPlan
from planner.collision_rules import CollisionManager
import math


class SAWStrategy:
    """
    Stop-and-Weld strategy planner.
    Optimizes for compact pieces with varying Y positions.
    """
    
    def __init__(self, collision_manager: CollisionManager,
                 robot_y_speed: float = 100.0,
                 max_stop_spacing: float = 500.0):
        """
        Args:
            collision_manager: Handles collision detection
            robot_y_speed: Speed of robot Y-axis movement (mm/s)
            max_stop_spacing: Maximum spacing between gantry stops (mm)
        """
        self.collision_manager = collision_manager
        self.robot_y_speed = robot_y_speed
        self.max_stop_spacing = max_stop_spacing
    
    def is_saw_suitable(self, weld: Weld) -> bool:
        """
        Determine if a weld is suitable for SAW mode.
        Shorter welds or those requiring Y-movement are ideal.
        """
        length_x = abs(weld.x_end - weld.x_start)
        return length_x <= 1500  # Less than 1.5 meters
    
    def determine_gantry_stops(self, welds: List[Weld], 
                              gantry_x_length: float) -> List[float]:
        """
        Calculate optimal gantry stop positions to cover all welds.
        
        Strategy: Place stops to minimize total distance while ensuring
        all welds can be reached from at least one stop.
        """
        if not welds:
            return []
        
        # Get all X positions from welds
        x_positions = []
        for weld in welds:
            x_positions.extend([weld.x_start, weld.x_end])
        
        # Remove duplicates and sort
        x_positions = sorted(set(x_positions))
        
        if not x_positions:
            return [0.0]
        
        # Method 1: Uniform spacing with coverage
        x_min = min(x_positions)
        x_max = max(x_positions)
        x_range = x_max - x_min
        
        if x_range <= self.max_stop_spacing:
            # Single stop in middle
            return [(x_min + x_max) / 2]
        
        # Multiple stops needed
        num_stops = math.ceil(x_range / self.max_stop_spacing)
        stops = []
        
        for i in range(num_stops):
            stop_x = x_min + (i + 0.5) * x_range / num_stops
            stops.append(stop_x)
        
        return stops
    
    def assign_welds_to_stops(self, welds: List[Weld], 
                             stops: List[float],
                             reach_distance: float = 400.0) -> Dict[float, List[Weld]]:
        """
        Assign each weld to the nearest gantry stop(s).
        A weld can be assigned to multiple stops if it spans them.
        
        Args:
            welds: List of welds to assign
            stops: List of gantry X positions
            reach_distance: How far robot can reach from gantry position
        """
        assignments = {stop: [] for stop in stops}
        
        for weld in welds:
            weld_x_center = (weld.x_start + weld.x_end) / 2
            
            # Find closest stop(s)
            for stop in stops:
                # Check if weld is reachable from this stop
                distance = abs(weld_x_center - stop)
                if distance <= reach_distance:
                    assignments[stop].append(weld)
        
        return assignments
    
    def schedule_robot_tasks_at_stop(self, welds: List[Weld], 
                                    robots: List[Robot],
                                    stop_x: float) -> List[WeldTask]:
        """
        Schedule robot tasks at a single gantry stop position.
        Optimizes for parallel execution and collision avoidance.
        """
        # Group welds by side
        x_plus_welds = sorted([w for w in welds if w.side == "x_plus"], 
                             key=lambda w: w.y)
        x_minus_welds = sorted([w for w in welds if w.side == "x_minus"], 
                              key=lambda w: w.y)
        
        # Get robots for each side
        x_plus_robots = sorted([r for r in robots if r.side == "x_plus"],
                              key=lambda r: r.workspace_center)
        x_minus_robots = sorted([r for r in robots if r.side == "x_minus"],
                               key=lambda r: r.workspace_center)
        
        tasks = []
        
        # Schedule x_plus side
        if x_plus_welds and x_plus_robots:
            tasks.extend(self._schedule_side_tasks(
                x_plus_welds, x_plus_robots, stop_x, "x_plus"
            ))
        
        # Schedule x_minus side
        if x_minus_welds and x_minus_robots:
            tasks.extend(self._schedule_side_tasks(
                x_minus_welds, x_minus_robots, stop_x, "x_minus"
            ))
        
        return tasks
    
    def _schedule_side_tasks(self, welds: List[Weld], robots: List[Robot],
                            stop_x: float, side: str) -> List[WeldTask]:
        """
        Schedule tasks for robots on one side of gantry.
        Uses greedy assignment: closest robot to each weld.
        """
        tasks = []
        robot_workloads = {r.id: 0.0 for r in robots}
        
        for weld in welds:
            # Find best robot (can reach + minimum workload)
            best_robot = None
            best_score = float('inf')
            
            for robot in robots:
                if not robot.can_reach(weld.y):
                    continue
                
                # Score = distance to weld + current workload
                distance = abs(weld.y - robot.workspace_center)
                workload = robot_workloads[robot.id]
                score = distance + workload * 10  # Weight workload higher
                
                if score < best_score:
                    best_score = score
                    best_robot = robot
            
            if best_robot:
                # Calculate time
                move_time = abs(weld.y - best_robot.workspace_center) / self.robot_y_speed
                weld_time = weld.length / best_robot.tcp_speed
                total_time = move_time + weld_time
                
                task = WeldTask(
                    robot_id=best_robot.id,
                    weld=weld,
                    mode=WeldMode.SAW,
                    start_x=stop_x,
                    y_position=weld.y,
                    estimated_time=total_time
                )
                tasks.append(task)
                robot_workloads[best_robot.id] += total_time
        
        return tasks
    
    def optimize_task_sequence(self, tasks: List[WeldTask], 
                              robots: List[Robot]) -> List[WeldTask]:
        """
        Optimize the sequence of tasks to minimize collisions and wait time.
        Uses priority rules from collision manager.
        """
        # Group tasks by robot
        robot_tasks = {}
        for task in tasks:
            if task.robot_id not in robot_tasks:
                robot_tasks[task.robot_id] = []
            robot_tasks[task.robot_id].append(task)
        
        # Sort each robot's tasks by Y position (minimize travel)
        for robot_id in robot_tasks:
            robot = next(r for r in robots if r.id == robot_id)
            robot_tasks[robot_id].sort(
                key=lambda t: abs(t.y_position - robot.workspace_center)
            )
        
        # Check for collision zones and adjust priorities
        optimized = []
        for robot_id, rtasks in robot_tasks.items():
            for task in rtasks:
                # Check if task is in collision zone
                zone = self.collision_manager.get_zone_for_y(robot_id, task.y_position)
                if zone:
                    # Mark priority
                    task.priority = zone.priority_robot == robot_id
                else:
                    task.priority = True
            optimized.extend(rtasks)
        
        return optimized
    
    def create_saw_plan(self, welds: List[Weld], robots: List[Robot],
                       gantry_x_length: float = 6000.0) -> WeldPlan:
        """
        Create complete SAW execution plan.
        
        Returns:
            WeldPlan with tasks organized by gantry stops
        """
        # Step 1: Determine gantry stops
        stops = self.determine_gantry_stops(welds, gantry_x_length)
        
        if not stops:
            return WeldPlan(tasks=[], mode=WeldMode.SAW, saw_stops=[])
        
        # Step 2: Assign welds to stops
        stop_assignments = self.assign_welds_to_stops(welds, stops)
        
        all_tasks = []
        
        # Step 3: Schedule tasks at each stop
        for stop_x in stops:
            stop_welds = stop_assignments[stop_x]
            if not stop_welds:
                continue
            
            stop_tasks = self.schedule_robot_tasks_at_stop(
                stop_welds, robots, stop_x
            )
            
            # Optimize sequence
            stop_tasks = self.optimize_task_sequence(stop_tasks, robots)
            
            all_tasks.extend(stop_tasks)
        
        # Step 4: Calculate total time
        total_time = self._estimate_saw_total_time(all_tasks, stops, robots)
        
        return WeldPlan(
            tasks=all_tasks,
            mode=WeldMode.SAW,
            estimated_total_time=total_time,
            saw_stops=stops
        )
    
    def _estimate_saw_total_time(self, tasks: List[WeldTask], 
                                stops: List[float],
                                robots: List[Robot]) -> float:
        """
        Estimate total time for SAW execution.
        Time = gantry movement + parallel robot work at each stop
        """
        # Gantry movement time
        gantry_time = 0.0
        for i in range(len(stops) - 1):
            distance = abs(stops[i+1] - stops[i])
            gantry_time += distance / 100.0  # Assume 100 mm/s gantry speed
        
        # Work time at each stop (robots work in parallel)
        work_time = 0.0
        stop_tasks = {}
        for task in tasks:
            if task.start_x not in stop_tasks:
                stop_tasks[task.start_x] = []
            stop_tasks[task.start_x].append(task)
        
        for stop_x, stop_task_list in stop_tasks.items():
            # Group by robot
            robot_times = {}
            for task in stop_task_list:
                if task.robot_id not in robot_times:
                    robot_times[task.robot_id] = 0.0
                robot_times[task.robot_id] += task.estimated_time
            
            # Maximum time (robots work in parallel)
            if robot_times:
                work_time += max(robot_times.values())
        
        return gantry_time + work_time


class HybridStrategy:
    """
    Hybrid strategy that uses both WOM and SAW based on workpiece characteristics.
    """
    
    def __init__(self, wom_strategy: SAWStrategy, saw_strategy: SAWStrategy,
                 wom_threshold: float = 7.0):
        """
        Args:
            wom_strategy: WOM strategy instance
            saw_strategy: SAW strategy instance
            wom_threshold: Aspect ratio threshold for WOM mode
        """
        self.wom_strategy = wom_strategy
        self.saw_strategy = saw_strategy
        self.wom_threshold = wom_threshold
    
    def classify_welds(self, welds: List[Weld]) -> Tuple[List[Weld], List[Weld]]:
        """
        Classify welds into WOM-suitable and SAW-suitable.
        
        Returns:
            (wom_welds, saw_welds)
        """
        wom_welds = []
        saw_welds = []
        
        for weld in welds:
            if self.wom_strategy.is_wom_suitable(weld):
                wom_welds.append(weld)
            else:
                saw_welds.append(weld)
        
        return wom_welds, saw_welds
    
    def create_hybrid_plan(self, welds: List[Weld], robots: List[Robot],
                          gantry_x_length: float = 6000.0) -> WeldPlan:
        """
        Create plan using both WOM and SAW strategies.
        """
        wom_welds, saw_welds = self.classify_welds(welds)
        
        all_tasks = []
        total_time = 0.0
        
        # Execute WOM welds first (usually more efficient)
        if wom_welds:
            wom_plan = self.wom_strategy.create_wom_plan(wom_welds, robots)
            all_tasks.extend(wom_plan.tasks)
            total_time += wom_plan.estimated_total_time
        
        # Then SAW welds
        if saw_welds:
            saw_plan = self.saw_strategy.create_saw_plan(saw_welds, robots, gantry_x_length)
            all_tasks.extend(saw_plan.tasks)
            total_time += saw_plan.estimated_total_time
        
        # Determine primary mode
        primary_mode = WeldMode.WOM if len(wom_welds) > len(saw_welds) else WeldMode.SAW
        
        return WeldPlan(
            tasks=all_tasks,
            mode=primary_mode,
            estimated_total_time=total_time
        )
