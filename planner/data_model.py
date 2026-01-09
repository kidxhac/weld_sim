# planner/data_model.py
from dataclasses import dataclass, field
from typing import Optional, List
from enum import Enum

class WeldMode(Enum):
    WOM = "weld_on_move"      # Weld-on-Move
    SAW = "stop_and_weld"     # Stop-and-Weld

class RobotState(Enum):
    IDLE = "IDLE"
    MOVING_Y = "MOVING_Y"      # Moving to weld position
    WELDING = "WELDING"
    WAIT_MUTEX = "WAIT_MUTEX"  # Waiting for collision zone
    COMPLETED = "COMPLETED"

@dataclass
class Weld:
    """Represents a single weld seam"""
    id: int
    x_start: float
    x_end: float
    y: float
    side: str                   # "x_plus" or "x_minus"
    
    # Computed properties
    length: float = 0.0
    done: float = 0.0           # Progress
    assigned_robot: Optional[str] = None
    mode: Optional[WeldMode] = None
    
    def __post_init__(self):
        if self.length == 0.0:
            self.length = abs(self.x_end - self.x_start)
    
    @property
    def is_completed(self) -> bool:
        return self.done >= self.length
    
    @property
    def progress_percent(self) -> float:
        return (self.done / self.length * 100) if self.length > 0 else 0

@dataclass
class Robot:
    """Represents a welding robot"""
    id: str
    side: str                   # "x_plus" or "x_minus"
    y_range: tuple              # (min_y, max_y)
    tcp_speed: float            # Tool center point speed (mm/s)
    
    # State tracking
    state: RobotState = RobotState.IDLE
    current_weld: Optional[Weld] = None
    current_y: float = 0.0      # Current Y position
    target_y: Optional[float] = None
    
    # Task queue for SAW mode
    weld_queue: List[Weld] = field(default_factory=list)
    
    # Statistics
    total_weld_time: float = 0.0
    total_move_time: float = 0.0
    welds_completed: int = 0
    
    def __post_init__(self):
        # Initialize at center of Y range
        self.current_y = (self.y_range[0] + self.y_range[1]) / 2
    
    @property
    def is_busy(self) -> bool:
        return self.state not in [RobotState.IDLE, RobotState.COMPLETED]
    
    @property
    def workspace_center(self) -> float:
        return (self.y_range[0] + self.y_range[1]) / 2
    
    def can_reach(self, y: float) -> bool:
        """Check if robot can reach the given Y coordinate"""
        return self.y_range[0] <= y <= self.y_range[1]

@dataclass
class Gantry:
    """Represents the gantry system"""
    x: float = 0.0              # Current X position
    speed: float = 100.0        # Speed (mm/s)
    x_length: float = 6000.0    # Total travel length
    
    # State
    is_moving: bool = False
    target_x: Optional[float] = None

@dataclass
class WeldTask:
    """Represents a planned welding task"""
    robot_id: str
    weld: Weld
    mode: WeldMode
    start_x: float              # Gantry X position when starting
    y_position: float           # Robot Y position for this weld
    estimated_time: float = 0.0 # Estimated completion time
    
    # For WOM mode
    wom_group: Optional[int] = None  # Which WOM window this belongs to

@dataclass
class WeldPlan:
    """Complete welding plan"""
    tasks: List[WeldTask]
    mode: WeldMode
    estimated_total_time: float = 0.0
    
    # For WOM mode
    wom_windows: List[List[WeldTask]] = field(default_factory=list)
    
    # For SAW mode
    saw_stops: List[float] = field(default_factory=list)  # X positions

@dataclass
class SimulationState:
    """Complete state of the simulation"""
    time: float = 0.0
    gantry: Gantry = field(default_factory=Gantry)
    robots: List[Robot] = field(default_factory=list)
    welds: List[Weld] = field(default_factory=list)
    plan: Optional[WeldPlan] = None
    
    # Statistics
    total_welds_completed: int = 0
    active_robots: int = 0
    
    @property
    def is_complete(self) -> bool:
        return all(w.is_completed for w in self.welds)
    
    @property
    def overall_progress(self) -> float:
        if not self.welds:
            return 100.0
        completed = sum(1 for w in self.welds if w.is_completed)
        return (completed / len(self.welds)) * 100
