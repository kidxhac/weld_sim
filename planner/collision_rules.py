# planner/collision_rules.py
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass

@dataclass
class CollisionZone:
    """Defines a collision zone between two robots"""
    name: str
    robot_pair: Tuple[str, str]  # (robot1_id, robot2_id)
    y_range: Tuple[float, float]  # (y_min, y_max)
    safe_distance: float = 150.0  # Minimum safe distance (mm)
    priority_robot: str = None    # Robot with priority in this zone
    
    def __post_init__(self):
        # First robot in pair gets priority by default
        if self.priority_robot is None:
            self.priority_robot = self.robot_pair[0]
    
    def contains_y(self, y: float) -> bool:
        """Check if Y coordinate is within this collision zone"""
        return self.y_range[0] <= y <= self.y_range[1]
    
    def robots_in_zone(self, robot_positions: dict) -> Set[str]:
        """Return which robots from the pair are currently in the zone"""
        in_zone = set()
        for robot_id in self.robot_pair:
            if robot_id in robot_positions:
                y_pos = robot_positions[robot_id]
                if self.contains_y(y_pos):
                    in_zone.add(robot_id)
        return in_zone
    
    def check_collision(self, y1: float, y2: float) -> bool:
        """Check if two Y positions would collide in this zone"""
        # Both must be in zone and too close
        if not (self.contains_y(y1) and self.contains_y(y2)):
            return False
        return abs(y1 - y2) < self.safe_distance


class CollisionManager:
    """Manages collision detection and mutex locks for all zones"""
    
    def __init__(self, zones: List[CollisionZone]):
        self.zones = zones
        # Mutex locks: zone_name -> robot_id (None if unlocked)
        self.locks = {zone.name: None for zone in zones}
        # Track which robot is waiting for which zone
        self.waiting = {}  # robot_id -> zone_name
    
    def get_zone_for_robots(self, robot1_id: str, robot2_id: str) -> Optional[CollisionZone]:
        """Find the collision zone between two robots"""
        for zone in self.zones:
            if set([robot1_id, robot2_id]) == set(zone.robot_pair):
                return zone
        return None
    
    def get_zone_for_y(self, robot_id: str, y: float) -> Optional[CollisionZone]:
        """Find if the robot at position Y is in any collision zone"""
        for zone in self.zones:
            if robot_id in zone.robot_pair and zone.contains_y(y):
                return zone
        return None
    
    def try_acquire_lock(self, robot_id: str, y: float) -> bool:
        """
        Try to acquire lock for a robot at position Y.
        Returns True if successful, False if blocked.
        """
        zone = self.get_zone_for_y(robot_id, y)
        if zone is None:
            # Not in any collision zone, no lock needed
            return True
        
        lock_holder = self.locks[zone.name]
        
        if lock_holder is None:
            # Lock is free, acquire it
            self.locks[zone.name] = robot_id
            return True
        elif lock_holder == robot_id:
            # Already holding the lock
            return True
        else:
            # Another robot holds the lock
            self.waiting[robot_id] = zone.name
            return False
    
    def release_lock(self, robot_id: str):
        """Release any locks held by this robot"""
        for zone_name, holder in self.locks.items():
            if holder == robot_id:
                self.locks[zone_name] = None
        
        # Remove from waiting
        if robot_id in self.waiting:
            del self.waiting[robot_id]
    
    def check_priority(self, robot_id: str, y: float) -> bool:
        """
        Check if robot has priority at this position.
        Returns True if robot can proceed.
        """
        zone = self.get_zone_for_y(robot_id, y)
        if zone is None:
            return True  # No collision zone
        
        # Check if the priority robot is already working here
        lock_holder = self.locks[zone.name]
        
        if lock_holder is None:
            # No one holds lock, check priority
            return robot_id == zone.priority_robot
        else:
            # Someone holds lock
            return lock_holder == robot_id
    
    def get_collision_risk(self, robot_positions: dict) -> List[Tuple[str, str]]:
        """
        Check all zones for collision risks.
        Returns list of (robot1, robot2) pairs at risk.
        """
        risks = []
        for zone in self.zones:
            robots_in_zone = zone.robots_in_zone(robot_positions)
            if len(robots_in_zone) >= 2:
                robot_list = list(robots_in_zone)
                for i, r1 in enumerate(robot_list):
                    for r2 in robot_list[i+1:]:
                        y1 = robot_positions[r1]
                        y2 = robot_positions[r2]
                        if zone.check_collision(y1, y2):
                            risks.append((r1, r2))
        return risks
    
    def get_safe_y_position(self, robot_id: str, target_y: float, 
                           other_positions: dict) -> float:
        """
        Adjust target Y position to avoid collision.
        Returns a safe Y position near the target.
        """
        zone = self.get_zone_for_y(robot_id, target_y)
        if zone is None:
            return target_y
        
        # Find other robot in same zone
        other_robot = None
        for rid in zone.robot_pair:
            if rid != robot_id and rid in other_positions:
                other_robot = rid
                break
        
        if other_robot is None:
            return target_y
        
        other_y = other_positions[other_robot]
        
        # If too close, adjust position
        if abs(target_y - other_y) < zone.safe_distance:
            # Move away from other robot
            if target_y > other_y:
                safe_y = other_y + zone.safe_distance + 50  # Add buffer
            else:
                safe_y = other_y - zone.safe_distance - 50
            
            # Make sure still in robot's range
            # (caller should validate this)
            return safe_y
        
        return target_y
    
    def get_status_report(self) -> dict:
        """Get current status of all zones"""
        return {
            "locks": self.locks.copy(),
            "waiting": self.waiting.copy(),
            "zones": [
                {
                    "name": z.name,
                    "robots": z.robot_pair,
                    "y_range": z.y_range,
                    "locked_by": self.locks[z.name]
                }
                for z in self.zones
            ]
        }


def create_standard_collision_zones(scene_config: dict) -> List[CollisionZone]:
    """
    Create standard collision zones from scene configuration.
    Assumes standard 4-robot setup with s1 and s2 zones.
    """
    zones = []
    
    interference = scene_config.get("interference", {})
    robots = scene_config.get("robots", [])
    
    # Find robots on each side
    x_plus_robots = [r["id"] for r in robots if r["side"] == "x_plus"]
    x_minus_robots = [r["id"] for r in robots if r["side"] == "x_minus"]
    
    # Create s1 zone for x_plus side (R1 and R3)
    if "s1" in interference and len(x_plus_robots) >= 2:
        zones.append(CollisionZone(
            name="s1",
            robot_pair=(x_plus_robots[0], x_plus_robots[1]),
            y_range=interference["s1"],
            priority_robot=x_plus_robots[0]  # R1 has priority
        ))
    
    # Create s2 zone for x_minus side (R2 and R4)
    if "s2" in interference and len(x_minus_robots) >= 2:
        zones.append(CollisionZone(
            name="s2",
            robot_pair=(x_minus_robots[0], x_minus_robots[1]),
            y_range=interference["s2"],
            priority_robot=x_minus_robots[0]  # R2 has priority
        ))
    
    return zones
