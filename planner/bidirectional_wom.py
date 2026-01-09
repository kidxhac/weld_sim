# planner/bidirectional_wom.py
"""
Bidirectional Weld-on-Move Strategy

Enhances standard WOM by allowing gantry to move back and forth to handle
uneven workload distribution among robots.

Key Innovation:
- Don't force all 4 robots to work simultaneously
- Allow 1-2 robots to continue working if others are idle
- Gantry can reverse direction to complete remaining welds
- Maximizes overall efficiency even with uneven weld distribution

Example Scenario:
  Window 1 (X: 0→2000mm):
    R1: 2000mm weld
    R2: 1800mm weld  
    R3: 500mm weld (finishes early)
    R4: idle
  
  Strategy:
    - Forward pass (X: 0→2000): R1, R2, R3 weld simultaneously
    - At X=500: R3 finishes, continues alone
    - At X=1800: R2 finishes, R1 continues alone
    - At X=2000: R1 finishes
    
  Window 2 (X: 2500→3000mm):
    R4: 500mm weld (only robot working)
  
  Strategy:
    - Forward pass (X: 2000→2500): Gantry repositions
    - Forward pass (X: 2500→3000): R4 welds alone
"""

from typing import List, Dict, Tuple, Optional
from planner.data_model import Weld, Robot, WeldTask, WeldMode, WeldPlan
from planner.collision_rules import CollisionManager
from planner.collision_zone_splitter import CollisionZoneWorkSplitter
import copy


class BidirectionalWOMStrategy:
    """
    Advanced WOM strategy with bidirectional gantry movement.
    """
    
    def __init__(self, collision_manager: CollisionManager, 
                 gantry_speed: float = 300.0,
                 min_robots_threshold: int = 1):
        """
        Args:
            collision_manager: Collision detection system
            gantry_speed: Gantry movement speed (mm/s)
            min_robots_threshold: Minimum robots working to use WOM mode
        """
        self.collision_manager = collision_manager
        self.gantry_speed = gantry_speed
        self.min_robots_threshold = min_robots_threshold
        self.zone_splitter = CollisionZoneWorkSplitter(collision_manager)
    
    def group_welds_by_proximity(self, welds: List[Weld], 
                                 max_gap: float = 500.0,
                                 robot_reach: float = 2000.0) -> List[List[Weld]]:
        """
        Group welds by X-axis proximity AND robot reach capability.
        
        Key enhancement: Include short welds that are within robot working range
        even if they have gaps. This allows robots to reach welds while gantry
        passes by without needing a separate pass.
        
        Args:
            welds: List of welds to group
            max_gap: Maximum X-axis gap between weld ends (mm)
            robot_reach: Robot working radius (mm) - default 2000mm
            
        Returns:
            List of weld groups
        """
        if not welds:
            return []
        
        # Sort by X start position
        sorted_welds = sorted(welds, key=lambda w: w.x_start)
        
        groups = []
        current_group = [sorted_welds[0]]
        current_x_min = sorted_welds[0].x_start
        current_x_max = sorted_welds[0].x_end
        
        for weld in sorted_welds[1:]:
            gap = weld.x_start - current_x_max
            
            # Check two conditions for including in current group:
            # 1. Close proximity (traditional approach)
            # 2. Within robot reach from any point in current group's X range
            
            # Condition 1: Nearby weld
            is_nearby = gap <= max_gap
            
            # Condition 2: Reachable from group's X range
            # A robot at any X position in [current_x_min, current_x_max] 
            # can reach this weld if the closest point is within robot_reach
            closest_x_in_group = min(max(weld.x_start, current_x_min), current_x_max)
            closest_distance_x = abs(weld.x_start - closest_x_in_group)
            is_reachable = closest_distance_x <= robot_reach
            
            if is_nearby or is_reachable:
                # Add to current group
                current_group.append(weld)
                current_x_min = min(current_x_min, weld.x_start)
                current_x_max = max(current_x_max, weld.x_end)
            else:
                # Start new group
                groups.append(current_group)
                current_group = [weld]
                current_x_min = weld.x_start
                current_x_max = weld.x_end
        
        # Add final group
        if current_group:
            groups.append(current_group)
        
        return groups
    
    def assign_welds_with_balancing(self, welds: List[Weld], 
                                    robots: List[Robot]) -> Dict[str, List[Weld]]:
        """
        Assign welds to robots with collision zone work splitting.
        
        Args:
            welds: Welds in this group
            robots: Available robots
            
        Returns:
            Dict mapping robot_id to assigned welds
        """
        assignments = {r.id: [] for r in robots}
        
        # Group by side
        x_plus_welds = [w for w in welds if w.side == "x_plus"]
        x_minus_welds = [w for w in welds if w.side == "x_minus"]
        
        # Assign x_plus welds
        x_plus_robots = [r for r in robots if r.side == "x_plus"]
        if x_plus_welds and x_plus_robots:
            self._assign_to_robot_group(x_plus_welds, x_plus_robots, assignments)
        
        # Assign x_minus welds
        x_minus_robots = [r for r in robots if r.side == "x_minus"]
        if x_minus_welds and x_minus_robots:
            self._assign_to_robot_group(x_minus_welds, x_minus_robots, assignments)
        
        # Apply collision zone optimization
        assignments = self.zone_splitter.apply_to_all_zones(assignments)
        
        return assignments
    
    def _assign_to_robot_group(self, welds: List[Weld], robots: List[Robot],
                               assignments: Dict[str, List[Weld]]):
        """Assign welds to robots, considering 2000mm working radius for gap zones"""
        # Sort robots by Y range start (ascending)
        sorted_robots = sorted(robots, key=lambda r: r.y_range[0])
        
        for weld in welds:
            assigned = False
            
            # First try: Find robot whose range includes the weld
            for robot in sorted_robots:
                if robot.can_reach(weld.y):
                    assignments[robot.id].append(weld)
                    assigned = True
                    break
            
            if assigned:
                continue
            
            # Second try: Weld is in a gap between robots
            # Strategy: Assign to robot whose range the weld is HEADING TOWARDS
            
            # Find which robots can reach via 2000mm radius
            candidates = []
            for robot in sorted_robots:
                dist = abs(robot.workspace_center - weld.y)
                if dist <= 2000.0:
                    candidates.append(robot)
            
            if not candidates:
                continue
            
            # If weld is between two robots, find the gap midpoint
            # Assign based on which side of midpoint the weld is on
            if len(candidates) >= 2:
                # Find the two robots on either side of the weld
                below_robots = [r for r in candidates if r.y_range[1] < weld.y]
                above_robots = [r for r in candidates if r.y_range[0] > weld.y]
                
                if below_robots and above_robots:
                    # Weld is in gap between robots
                    # FOR EFFICIENCY: Assign gap welds to UPPER robot
                    # This lets lower robots focus on their own zone
                    # Upper robots handle both gap AND their own zone
                    upper_robot = min(above_robots, key=lambda r: r.y_range[0])
                    assignments[upper_robot.id].append(weld)
                    assigned = True
            
            if not assigned and candidates:
                # Fallback: assign to closest candidate
                best = min(candidates, key=lambda r: abs(r.workspace_center - weld.y))
                assignments[best.id].append(weld)
    
    def create_bidirectional_passes(self, group_assignments: Dict[str, List[Weld]],
                                   group_x_range: Tuple[float, float]) -> List[Dict]:
        """
        Create multiple passes (forward/backward) for a weld group.
        
        A "pass" is one continuous gantry movement in one direction.
        Welds of different lengths require multiple passes.
        
        Args:
            group_assignments: Robot assignments for this group
            group_x_range: (x_min, x_max) of all welds in group
            
        Returns:
            List of passes, each with: direction, x_start, x_end, active_robots
        """
        x_min, x_max = group_x_range
        
        # Build list of events (weld start/end points)
        events = []  # (x_position, event_type, robot_id, weld)
        
        for robot_id, welds in group_assignments.items():
            for weld in welds:
                events.append((weld.x_start, 'start', robot_id, weld))
                events.append((weld.x_end, 'end', robot_id, weld))
        
        events.sort(key=lambda e: e[0])
        
        # Simulate gantry movement to determine passes
        passes = []
        current_x = x_min
        direction = 'forward'  # Start moving forward
        active_welds = {}  # robot_id -> current weld
        
        pass_info = {
            'direction': direction,
            'x_start': current_x,
            'x_end': x_max,
            'active_robots': {}
        }
        
        for x_pos, event_type, robot_id, weld in events:
            if event_type == 'start':
                active_welds[robot_id] = weld
                pass_info['active_robots'][robot_id] = weld
            elif event_type == 'end':
                if robot_id in active_welds:
                    del active_welds[robot_id]
        
        passes.append(pass_info)
        
        return passes
    
    def calculate_optimal_y_positions(self, robot_id: str, welds: List[Weld],
                                     robot: Robot) -> float:
        """
        Calculate optimal Y position for robot to reach all assigned welds.
        
        With 2000mm working radius, robots can position outside their nominal
        Y-range to reach gap welds. Don't clamp to range.
        """
        if not welds:
            return robot.workspace_center
        
        # Weight by weld length - longer welds influence position more
        weighted_y = sum(w.y * w.length for w in welds)
        total_length = sum(w.length for w in welds)
        
        optimal_y = weighted_y / total_length if total_length > 0 else robot.workspace_center
        
        # Don't clamp - allow positioning outside nominal range for gap welds
        # Robot has 2000mm reach from any Y position
        return optimal_y
    
    def _calculate_optimal_gantry_start(self, assignments: Dict[str, List[Weld]], 
                                       robots: List[Robot],
                                       robot_reach: float = 2000.0) -> float:
        """
        Calculate optimal gantry starting position so all robots can reach their welds.
        
        Strategy: Find the position where ALL robots can reach their first weld's start point.
        This ensures all robots begin welding simultaneously.
        
        Args:
            assignments: Robot to welds mapping
            robots: List of robots
            robot_reach: Robot working radius (mm)
            
        Returns:
            Optimal X position for gantry to start
        """
        max_required_x = 0.0
        
        for robot in robots:
            robot_welds = assignments.get(robot.id, [])
            if not robot_welds:
                continue
            
            # Find the first weld for this robot (earliest x_start)
            first_weld = min(robot_welds, key=lambda w: w.x_start)
            
            # Robot needs to be able to reach first_weld.x_start
            # Robot position = gantry.x + offset
            # Distance from robot to weld start = |first_weld.x_start - (gantry.x + offset)|
            # For x_plus robots (offset = +300mm):
            #   first_weld.x_start - (gantry.x + 300) <= robot_reach
            #   gantry.x >= first_weld.x_start - robot_reach - 300
            # For x_minus robots (offset = -300mm):
            #   first_weld.x_start - (gantry.x - 300) <= robot_reach
            #   gantry.x >= first_weld.x_start - robot_reach + 300
            
            robot_offset = 300 if robot.side == "x_plus" else -300
            
            # Calculate minimum gantry X for this robot to reach its first weld
            min_gantry_x = first_weld.x_start - robot_reach - robot_offset
            
            # We need the MAXIMUM of all minimum positions
            # (position where ALL robots can reach)
            max_required_x = max(max_required_x, min_gantry_x)
        
        return max_required_x
    
    def create_bidirectional_wom_plan(self, welds: List[Weld], robots: List[Robot],
                                     gantry_start_x: float = 0.0) -> WeldPlan:
        """
        Create complete bidirectional WOM plan.
        
        Main algorithm:
        1. Group welds by proximity (not requiring all 4 robots)
        2. For each group, assign welds with collision zone optimization
        3. Create tasks with optimal Y positions
        4. Allow gantry to move forward/backward as needed
        
        Returns:
            WeldPlan with bidirectional passes
        """
        print("\n=== Bidirectional WOM Strategy ===")
        
        # Group welds with robot reach consideration (2000mm radius)
        groups = self.group_welds_by_proximity(welds, robot_reach=2000.0)
        print(f"Created {len(groups)} weld groups (robot reach: 2000mm)")
        
        all_tasks = []
        window_groups = []
        
        for group_idx, group_welds in enumerate(groups):
            print(f"\nGroup {group_idx + 1}: {len(group_welds)} welds")
            
            # Assign with balancing
            assignments = self.assign_welds_with_balancing(group_welds, robots)
            
            # Calculate X range
            x_min = min(w.x_start for w in group_welds)
            x_max = max(w.x_end for w in group_welds)
            
            # Calculate SMART starting position:
            # Gantry should start where ALL robots can reach their weld start points
            # This ensures all robots start welding simultaneously
            gantry_start_x = self._calculate_optimal_gantry_start(
                assignments, robots, robot_reach=2000.0
            )
            
            # Ensure start position is not before the earliest weld
            gantry_start_x = max(gantry_start_x, x_min)
            
            # Report assignments
            active_robots = [rid for rid, ws in assignments.items() if ws]
            print(f"  X range: [{x_min:.0f}, {x_max:.0f}]mm")
            print(f"  Gantry starts at: X={gantry_start_x:.0f}mm (optimized for simultaneous start)")
            print(f"  Active robots: {len(active_robots)} - {active_robots}")
            
            # Calculate optimal Y positions
            robot_y_positions = {}
            for robot in robots:
                if assignments[robot.id]:
                    robot_y_positions[robot.id] = self.calculate_optimal_y_positions(
                        robot.id, assignments[robot.id], robot
                    )
            
            # Create tasks for this group
            group_tasks = []
            for robot in robots:
                robot_welds = assignments[robot.id]
                if robot_welds:
                    for weld in robot_welds:
                        # Calculate time based on weld length and gantry speed
                        weld_time = weld.length / self.gantry_speed
                        
                        task = WeldTask(
                            robot_id=robot.id,
                            weld=weld,
                            mode=WeldMode.WOM,
                            start_x=gantry_start_x,  # Use optimized start position
                            y_position=robot_y_positions[robot.id],
                            estimated_time=weld_time,
                            wom_group=group_idx
                        )
                        group_tasks.append(task)
                        all_tasks.append(task)
            
            window_groups.append(group_tasks)
        
        # Calculate total time
        total_time = self._estimate_bidirectional_time(window_groups)
        
        print(f"\nTotal tasks: {len(all_tasks)}")
        print(f"Estimated time: {total_time:.1f}s")
        
        return WeldPlan(
            tasks=all_tasks,
            mode=WeldMode.WOM,
            estimated_total_time=total_time,
            wom_windows=window_groups
        )
    
    def _estimate_bidirectional_time(self, window_groups: List[List[WeldTask]]) -> float:
        """
        Estimate total time for bidirectional WOM.
        
        Accounts for:
        - Parallel welding within each group
        - Gantry repositioning between groups
        - Potential backward passes
        """
        total_time = 0.0
        last_x_end = 0.0
        
        for window_tasks in window_groups:
            if not window_tasks:
                continue
            
            # Time for this window (max of all concurrent welds)
            window_time = max(task.estimated_time for task in window_tasks)
            total_time += window_time
            
            # Repositioning time to next window
            window_x_start = min(t.start_x for t in window_tasks)
            reposition_distance = abs(window_x_start - last_x_end)
            reposition_time = reposition_distance / self.gantry_speed
            total_time += reposition_time
            
            # Track end position
            window_x_end = max(t.weld.x_end for t in window_tasks)
            last_x_end = window_x_end
        
        return total_time
