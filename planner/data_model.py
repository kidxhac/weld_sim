# planner/data_model.py
from dataclasses import dataclass
from typing import Optional

@dataclass
class Weld:
    id: int
    x_start: float
    x_end: float
    y: float
    length: float
    side: str
    done: float = 0.0

@dataclass
class Robot:
    id: str
    side: str
    y_range: tuple
    tcp_speed: float
    state: str = "IDLE"          # IDLE / WELDING / WAIT_MUTEX
    current_weld: Optional[Weld] = None

@dataclass
class Gantry:
    x: float
    speed: float
