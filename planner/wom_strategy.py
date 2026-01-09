# planner/wom_strategy.py
"""
Weld-on-Move (WOM) Strategy

Key concept: For long welds (Lx/Ly > 7), fix robots at Y positions and move gantry.

ENHANCED FEATURES:
1. Bidirectional gantry movement - allows back/forth motion for uneven workloads
2. Collision zone work splitting - balances workload when welds are in s1/s2 zones
3. Flexible robot participation - 1-4 robots can work, not required to be all 4

Algorithm:
1. Group welds by proximity (not requiring all robots)
2. Assign welds with collision zone optimization
3. Split collision zone welds to balance workload between robot pairs
4. Calculate optimal Y positions
5. Allow gantry bidirectional movement
"""

from typing import List, Tuple, Dict, Optional
from planner.data_model import Weld, Robot, WeldTask, WeldMode, WeldPlan
from planner.collision_rules import CollisionManager
from planner.collision_zone_splitter import CollisionZoneWorkSplitter
from planner.bidirectional_wom import BidirectionalWOMStrategy
import math


class WOMStrategy:
    """
    Weld-on-Move strategy planner.
    Optimizes for long, parallel welds along X-axis.
    """
    
    def __init__(self, collision_manager: CollisionManager, 
                 gantry_speed: float = 100.0,
                 length_ratio_threshold: float = 0.3,
                 use_bidirectional: bool = True,
                 use_collision_splitting: bool = True):
        """
        Args:
            collision_manager: Handles collision detection
            gantry_speed: Speed of gantry movement (mm/s)
            length_ratio_threshold: Max difference ratio for grouping welds
            use_bidirectional: Enable bidirectional gantry movement
            use_collision_splitting: Enable collision zone work splitting
        """
        self.collision_manager = collision_manager
        self.gantry_speed = gantry_speed
        self.length_ratio_threshold = length_ratio_threshold
        self.use_bidirectional = use_bidirectional
        self.use_collision_splitting = use_collision_splitting
        
        # Initialize advanced features
        if use_collision_splitting:
            self.zone_splitter = CollisionZoneWorkSplitter(collision_manager)
        
        if use_bidirectional:
            self.bidirectional_strategy = BidirectionalWOMStrategy(
                collision_manager, gantry_speed
            )
    
    def is_wom_suitable(self, weld: Weld) -> bool:
        """
        Determine if a weld is suitable for WOM mode.
        
        With bidirectional WOM and smart grouping, we're more permissive.
        Short welds can be included if they're reachable during a pass.
        """
        length_x = abs(weld.x_end - weld.x_start)
        
        # Primary WOM welds: Long welds > 1500mm
        if length_x > 1500:
            return True
        
        # Secondary WOM welds: Medium welds > 300mm
        # These can be included in WOM passes if within robot reach
        if length_x > 300:
            return True
        
        # Very short welds (< 300mm) better suited for SAW
        return False
    
    def group_welds_into_windows(self, welds: List[Weld]) -> List[List[Weld]]:
        """
        Group welds into windows based on similar X-length and X-position.
        Welds in same window can be welded simultaneously.
        """
        if not welds:
            return []
        
        # Sort by X-start position
        sorted_welds = sorted(welds, key=lambda w: w.x_start)
        
        windows = []
        
        while sorted_welds:
            # Start new window with first weld
            base_weld = sorted_welds.pop(0)
            current_window = [base_weld]
            
            # Find welds with similar characteristics
            remaining = []
            for weld in sorted_welds:
                # Check if similar length
                length_diff = abs(weld.length - base_weld.length)
                length_ratio = length_diff / base_weld.length if base_weld.length > 0 else 1
                
                # Check if similar X-span (for parallel welding)
                x_overlap = self._calculate_x_overlap(base_weld, weld)
                
                if length_ratio < self.length_ratio_threshold and x_overlap > 0.7:
                    current_window.append(weld)
                else:
                    remaining.append(weld)
            
            windows.append(current_window)
            sorted_welds = remaining
        
        return windows
    
    def _calculate_x_overlap(self, weld1: Weld, weld2: Weld) -> float:
        """Calculate X-axis overlap ratio between two welds"""
        x1_min, x1_max = min(weld1.x_start, weld1.x_end), max(weld1.x_start, weld1.x_end)
        x2_min, x2_max = min(weld2.x_start, weld2.x_end), max(weld2.x_start, weld2.x_end)
        
        overlap_start = max(x1_min, x2_min)
        overlap_end = min(x1_max, x2_max)
        overlap_length = max(0, overlap_end - overlap_start)
        
        # Ratio of overlap to shorter weld
        shorter_length = min(x1_max - x1_min, x2_max - x2_min)
        return overlap_length / shorter_length if shorter_length > 0 else 0
    
    def assign_welds_to_robots(self, welds: List[Weld], robots: List[Robot]) -> Dict[str, List[Weld]]:
        """
        Assign welds in a window to available robots.
        Considers Y-position and side constraints.
        """
        assignments = {r.id: [] for r in robots}
        
        # Group welds by side
        x_plus_welds = [w for w in welds if w.side == "x_plus"]
        x_minus_welds = [w for w in welds if w.side == "x_minus"]
        
        # Assign to x_plus robots
        x_plus_robots = [r for r in robots if r.side == "x_plus"]
        if x_plus_welds and x_plus_robots:
            self._assign_by_y_position(x_plus_welds, x_plus_robots, assignments)
        
        # Assign to x_minus robots
        x_minus_robots = [r for r in robots if r.side == "x_minus"]
        if x_minus_welds and x_minus_robots:
            self._assign_by_y_position(x_minus_welds, x_minus_robots, assignments)
        
        return assignments
    
    def _assign_by_y_position(self, welds: List[Weld], robots: List[Robot], 
                              assignments: Dict[str, List[Weld]]):
        """Assign welds to robots based on Y-position proximity"""
        # Sort welds by Y position
        sorted_welds = sorted(welds, key=lambda w: w.y)
        
        # Sort robots by Y range center
        sorted_robots = sorted(robots, key=lambda r: (r.y_range[0] + r.y_range[1]) / 2)
        
        # Distribute welds among robots
        for i, weld in enumerate(sorted_welds):
            # Find best robot (closest Y range)
            best_robot = None
            best_distance = float('inf')
            
            for robot in sorted_robots:
                if robot.can_reach(weld.y):
                    # Calculate distance from robot's current center
                    distance = abs(weld.y - robot.workspace_center)
                    if distance < best_distance:
                        best_distance = distance
                        best_robot = robot
            
            if best_robot:
                assignments[best_robot.id].append(weld)
    
    def calculate_optimal_y_positions(self, robot_id: str, welds: List[Weld], 
                                     robot: Robot) -> float:
        """
        Calculate optimal Y position for robot to weld assigned seams.
        For WOM mode, robot stays at one Y position.
        """
        if not welds:
            return robot.workspace_center
        
        # Simple approach: average Y position of all welds
        avg_y = sum(w.y for w in welds) / len(welds)
        
        # Clamp to robot's range
        y_min, y_max = robot.y_range
        optimal_y = max(y_min, min(y_max, avg_y))
        
        return optimal_y
    
    def create_wom_plan(self, welds: List[Weld], robots: List[Robot], 
                       gantry_start_x: float = 0.0) -> WeldPlan:
        """
        Create complete WOM execution plan with enhanced features.
        
        Enhanced features:
        - Bidirectional gantry movement for uneven workloads
        - Collision zone work splitting for balanced loads
        - Flexible robot participation (1-4 robots)
        
        Returns:
            WeldPlan with tasks organized by WOM windows
        """
        # Use bidirectional strategy if enabled
        if self.use_bidirectional:
            print("\nUsing Bidirectional WOM Strategy")
            return self.bidirectional_strategy.create_bidirectional_wom_plan(
                welds, robots, gantry_start_x
            )
        
        # Otherwise use standard WOM with optional collision splitting
        print("\nUsing Standard WOM Strategy")
        return self._create_standard_wom_plan(welds, robots, gantry_start_x)
    
    def _create_standard_wom_plan(self, welds: List[Weld], robots: List[Robot],
                                 gantry_start_x: float = 0.0) -> WeldPlan:
        """
        Standard WOM plan (original algorithm) with optional collision splitting.
        """
        # Step 1: Group welds into windows
        windows = self.group_welds_into_windows(welds)
        
        all_tasks = []
        window_groups = []
        
        current_x = gantry_start_x
        
        # Step 2: Process each window
        for window_idx, window_welds in enumerate(windows):
            # Assign welds to robots
            assignments = self.assign_welds_to_robots(window_welds, robots)
            
            # Apply collision zone splitting if enabled
            if self.use_collision_splitting:
                print(f"\nWindow {window_idx + 1}: Applying collision zone optimization")
                assignments = self.zone_splitter.apply_to_all_zones(assignments)
            
            window_tasks = []
            robot_positions = {}
            
            # Step 3: Calculate optimal Y positions for each robot
            for robot in robots:
                robot_welds = assignments.get(robot.id, [])
                if robot_welds:
                    optimal_y = self.calculate_optimal_y_positions(
                        robot.id, robot_welds, robot
                    )
                    robot_positions[robot.id] = optimal_y
            
            # Step 4: Resolve collisions in Y positions
            robot_positions = self._resolve_position_collisions(robot_positions, robots)
            
            # Step 5: Create tasks for this window
            window_start_x = min(w.x_start for w in window_welds)
            window_end_x = max(w.x_end for w in window_welds)
            
            for robot in robots:
                robot_welds = assignments.get(robot.id, [])
                if robot_welds:
                    for weld in robot_welds:
                        # Estimate time: based on weld length and gantry speed
                        weld_time = weld.length / self.gantry_speed
                        
                        task = WeldTask(
                            robot_id=robot.id,
                            weld=weld,
                            mode=WeldMode.WOM,
                            start_x=window_start_x,
                            y_position=robot_positions[robot.id],
                            estimated_time=weld_time,
                            wom_group=window_idx
                        )
                        window_tasks.append(task)
                        all_tasks.append(task)
            
            window_groups.append(window_tasks)
            current_x = window_end_x
        
        # Calculate total time
        total_time = self._estimate_wom_total_time(window_groups, robots)
        
        return WeldPlan(
            tasks=all_tasks,
            mode=WeldMode.WOM,
            estimated_total_time=total_time,
            wom_windows=window_groups
        )
    
    def _resolve_position_collisions(self, robot_positions: Dict[str, float], 
                                    robots: List[Robot]) -> Dict[str, float]:
        """
        Adjust robot Y positions to avoid collisions in shared zones.
        """
        adjusted = robot_positions.copy()
        
        # Check each collision zone
        for zone in self.collision_manager.zones:
            r1_id, r2_id = zone.robot_pair
            
            if r1_id not in adjusted or r2_id not in adjusted:
                continue
            
            y1, y2 = adjusted[r1_id], adjusted[r2_id]
            
            # Check if both in collision zone and too close
            if zone.contains_y(y1) and zone.contains_y(y2):
                if abs(y1 - y2) < zone.safe_distance:
                    # Separate them
                    midpoint = (y1 + y2) / 2
                    offset = zone.safe_distance / 2 + 50  # Add buffer
                    
                    # Move them apart
                    adjusted[r1_id] = midpoint - offset
                    adjusted[r2_id] = midpoint + offset
                    
                    # Validate still in ranges
                    for rid in [r1_id, r2_id]:
                        robot = next(r for r in robots if r.id == rid)
                        adjusted[rid] = max(robot.y_range[0], 
                                          min(robot.y_range[1], adjusted[rid]))
        
        return adjusted
    
    def _estimate_wom_total_time(self, window_groups: List[List[WeldTask]], 
                                 robots: List[Robot]) -> float:
        """
        Estimate total time for WOM execution.
        Time = sum of window times + repositioning time
        """
        total_time = 0.0
        
        for window_tasks in window_groups:
            if not window_tasks:
                continue
            
            # All tasks in window execute in parallel
            # Time is max of all task times (they happen simultaneously)
            window_time = max(task.estimated_time for task in window_tasks)
            total_time += window_time
            
            # Add repositioning time between windows
            # (robots move to new Y positions)
            total_time += 5.0  # Estimated 5 seconds for repositioning
        
        return total_time


def split_wom_windows(welds: List[Weld], length_ratio: float = 0.3) -> List[List[Weld]]:
    """
    Legacy function for backward compatibility.
    Groups welds by similar length.
    """
    if not welds:
        return []
    
    sorted_welds = sorted(welds, key=lambda w: w.x_start)
    windows = []
    
    while sorted_welds:
        base = sorted_welds.pop(0)
        group = [base]
        
        remaining = []
        for w in sorted_welds:
            length_diff = abs(w.length - base.length)
            if length_diff / base.length < length_ratio:
                group.append(w)
            else:
                remaining.append(w)
        
        windows.append(group)
        sorted_welds = remaining
    
    return windows
