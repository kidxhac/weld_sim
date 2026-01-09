# planner/collision_zone_splitter.py
"""
Advanced Collision Zone Work Splitting

Optimizes workload distribution when welds are in collision zones (s1, s2).
Instead of blocking one robot while the other works, this splits the work
in the collision zone to balance total workload.

Strategy:
- Calculate total workload for each robot (including collision zone welds)
- If imbalanced, split collision zone welds at optimal partition point
- Robot 1 completes first part, exits zone
- Robot 2 enters and completes remaining part
"""

from typing import List, Tuple, Dict, Optional
from planner.data_model import Weld, Robot, WeldTask, WeldMode
from planner.collision_rules import CollisionZone, CollisionManager
import copy


class CollisionZoneWorkSplitter:
    """
    Intelligently splits welds in collision zones to balance workload.
    """
    
    def __init__(self, collision_manager: CollisionManager):
        """
        Args:
            collision_manager: Collision detection system
        """
        self.collision_manager = collision_manager
        self.split_welds_cache = {}  # Cache split welds by original weld ID
    
    def analyze_workload_imbalance(self, robot_assignments: Dict[str, List[Weld]], 
                                   zone: CollisionZone) -> Tuple[float, float, bool]:
        """
        Analyze workload imbalance between two robots sharing a collision zone.
        
        Args:
            robot_assignments: Dict mapping robot_id to assigned welds
            zone: CollisionZone to analyze
            
        Returns:
            (robot1_load, robot2_load, needs_splitting)
        """
        r1_id, r2_id = zone.robot_pair
        
        # Calculate total weld length for each robot
        r1_load = sum(w.length for w in robot_assignments.get(r1_id, []))
        r2_load = sum(w.length for w in robot_assignments.get(r2_id, []))
        
        # Check for imbalance (>20% difference)
        if r1_load == 0 or r2_load == 0:
            return r1_load, r2_load, False
        
        imbalance_ratio = abs(r1_load - r2_load) / max(r1_load, r2_load)
        needs_splitting = imbalance_ratio > 0.20  # 20% threshold
        
        return r1_load, r2_load, needs_splitting
    
    def find_collision_zone_welds(self, welds: List[Weld], 
                                  zone: CollisionZone) -> List[Weld]:
        """
        Find all welds that pass through a collision zone.
        
        Args:
            welds: List of all welds
            zone: CollisionZone to check
            
        Returns:
            List of welds in the collision zone
        """
        zone_welds = []
        for weld in welds:
            if zone.contains_y(weld.y):
                zone_welds.append(weld)
        return zone_welds
    
    def split_weld_at_point(self, weld: Weld, split_x: float, 
                           part1_robot: str, part2_robot: str) -> Tuple[Weld, Weld]:
        """
        Split a weld into two parts at specified X coordinate.
        
        Args:
            weld: Original weld to split
            split_x: X coordinate where to split
            part1_robot: Robot ID for first part
            part2_robot: Robot ID for second part
            
        Returns:
            (weld_part1, weld_part2)
        """
        # Ensure split point is within weld bounds
        split_x = max(weld.x_start, min(weld.x_end, split_x))
        
        # Create two new weld objects
        weld_part1 = Weld(
            id=weld.id * 1000 + 1,  # Unique ID for part 1
            x_start=weld.x_start,
            x_end=split_x,
            y=weld.y,
            side=weld.side
        )
        weld_part1.assigned_robot = part1_robot
        
        weld_part2 = Weld(
            id=weld.id * 1000 + 2,  # Unique ID for part 2
            x_start=split_x,
            x_end=weld.x_end,
            y=weld.y,
            side=weld.side
        )
        weld_part2.assigned_robot = part2_robot
        
        # Cache the split for later reference
        self.split_welds_cache[weld.id] = (weld_part1, weld_part2)
        
        return weld_part1, weld_part2
    
    def calculate_optimal_split_point(self, weld: Weld, 
                                      r1_current_load: float,
                                      r2_current_load: float) -> float:
        """
        Calculate optimal X coordinate to split weld for balanced workload.
        
        Strategy: Split so that final loads are as equal as possible.
        
        Args:
            weld: Weld in collision zone to split
            r1_current_load: Current workload of robot 1 (excluding this weld)
            r2_current_load: Current workload of robot 2 (excluding this weld)
            
        Returns:
            Optimal X coordinate for split point
        """
        total_load = r1_current_load + r2_current_load + weld.length
        target_r1_load = total_load / 2  # Aim for 50/50 balance
        
        # How much of this weld should go to robot 1?
        r1_weld_portion = target_r1_load - r1_current_load
        r1_weld_portion = max(0, min(weld.length, r1_weld_portion))
        
        # Calculate split X position
        weld_length = weld.x_end - weld.x_start
        if weld_length > 0:
            split_fraction = r1_weld_portion / weld_length
            split_x = weld.x_start + (weld_length * split_fraction)
        else:
            split_x = (weld.x_start + weld.x_end) / 2
        
        # Ensure minimum segment length (100mm)
        min_segment = 100.0
        if split_x - weld.x_start < min_segment:
            split_x = weld.x_start + min_segment
        if weld.x_end - split_x < min_segment:
            split_x = weld.x_end - min_segment
        
        return split_x
    
    def optimize_collision_zone_assignments(self, 
                                           robot_assignments: Dict[str, List[Weld]],
                                           zone: CollisionZone) -> Dict[str, List[Weld]]:
        """
        Optimize work assignments in a collision zone by splitting welds.
        
        IMPORTANT: If robots don't overlap (have a gap), skip balancing!
        Example: R1 (0-1000) and R3 (2000-3000) with zone (1000-2000)
                 → R1 can't reach zone, only R3 can → NO collision possible
        
        Main algorithm:
        1. Check if robots can both reach the zone
        2. If not, skip balancing (no collision possible)
        3. If yes, identify welds in collision zone
        4. Calculate workload imbalance
        5. If imbalanced, split collision zone welds optimally
        6. Update assignments with split welds
        
        Args:
            robot_assignments: Current assignments
            zone: CollisionZone to optimize
            
        Returns:
            Optimized assignments with split welds (or unchanged if no collision)
        """
        r1_id, r2_id = zone.robot_pair
        
        # Get robot objects to check their ranges
        # We need to check if robots can actually both reach this zone
        # If not, there's no collision possibility - skip balancing
        
        # For now, check if both robots have any welds in the zone
        r1_zone_welds = []
        r2_zone_welds = []
        
        for weld in robot_assignments.get(r1_id, []):
            if zone.contains_y(weld.y):
                r1_zone_welds.append(weld)
        
        for weld in robot_assignments.get(r2_id, []):
            if zone.contains_y(weld.y):
                r2_zone_welds.append(weld)
        
        # If only ONE robot has welds in zone, they can't collide - skip balancing
        if not r1_zone_welds or not r2_zone_welds:
            print(f"\n  Skipping collision zone {zone.name}: Only one robot has welds in zone")
            return robot_assignments  # No collision possible, skip balancing
        
        # Both robots have welds in zone - proceed with normal balancing
        # Analyze current workload
        r1_load, r2_load, needs_splitting = self.analyze_workload_imbalance(
            robot_assignments, zone
        )
        
        if not needs_splitting:
            return robot_assignments  # Already balanced
        
        print(f"\n  Optimizing collision zone {zone.name}:")
        print(f"    Current: {r1_id}={r1_load:.0f}mm, {r2_id}={r2_load:.0f}mm")
        
        # Find welds in collision zone - ONLY from robots in this zone's pair
        zone_welds = []
        for robot_id in [r1_id, r2_id]:
            robot_welds = robot_assignments.get(robot_id, [])
            for weld in robot_welds:
                if zone.contains_y(weld.y) and weld not in zone_welds:
                    zone_welds.append(weld)
        
        if not zone_welds:
            return robot_assignments  # No welds to split
        
        # Create new assignments
        optimized = copy.deepcopy(robot_assignments)
        
        # Remove collision zone welds from assignments
        for robot_id in [r1_id, r2_id]:
            optimized[robot_id] = [w for w in optimized[robot_id] 
                                  if w not in zone_welds]
        
        # Calculate loads without collision zone welds
        r1_load_no_zone = sum(w.length for w in optimized[r1_id])
        r2_load_no_zone = sum(w.length for w in optimized[r2_id])
        
        # Split each collision zone weld optimally
        for weld in zone_welds:
            split_x = self.calculate_optimal_split_point(
                weld, r1_load_no_zone, r2_load_no_zone
            )
            
            part1, part2 = self.split_weld_at_point(weld, split_x, r1_id, r2_id)
            
            optimized[r1_id].append(part1)
            optimized[r2_id].append(part2)
            
            # Update loads
            r1_load_no_zone += part1.length
            r2_load_no_zone += part2.length
            
            print(f"    Split W{weld.id} at X={split_x:.0f}: "
                  f"{r1_id} gets {part1.length:.0f}mm, "
                  f"{r2_id} gets {part2.length:.0f}mm")
        
        # Report final balance
        final_r1 = sum(w.length for w in optimized[r1_id])
        final_r2 = sum(w.length for w in optimized[r2_id])
        print(f"    Balanced: {r1_id}={final_r1:.0f}mm, {r2_id}={final_r2:.0f}mm")
        
        return optimized
    
    def apply_to_all_zones(self, 
                          robot_assignments: Dict[str, List[Weld]]) -> Dict[str, List[Weld]]:
        """
        Apply collision zone optimization to all zones.
        
        Args:
            robot_assignments: Initial assignments
            
        Returns:
            Optimized assignments with collision zone work splitting
        """
        optimized = robot_assignments
        
        for zone in self.collision_manager.zones:
            optimized = self.optimize_collision_zone_assignments(optimized, zone)
        
        return optimized
