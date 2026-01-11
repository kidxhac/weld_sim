# simulator/simulator.py
"""
Welding Simulation Engine

Simulates the execution of welding plans with realistic physics:
- Gantry movement along X-axis
- Robot movement along Y-axis  
- Collision detection and mutex handling
- Time-accurate simulation
"""

from typing import List, Dict, Optional
from planner.data_model import (
    Robot, Gantry, Weld, WeldPlan, WeldTask, WeldMode, 
    RobotState, SimulationState
)
from planner.collision_rules import CollisionManager
import copy


class Simulator:
    """
    Time-stepped simulation of welding operations.
    """
    
    def __init__(self, plan: WeldPlan, state: SimulationState,
                 collision_manager: CollisionManager, dt: float = 0.1):
        """
        Args:
            plan: Complete welding plan to execute
            state: Initial simulation state
            collision_manager: Handles collision detection
            dt: Time step in seconds
        """
        self.plan = plan
        self.state = state
        self.collision_manager = collision_manager
        self.dt = dt
        
        # Execution tracking
        self.current_task_idx = 0
        self.active_tasks: Dict[str, WeldTask] = {}  # robot_id -> current task
        self.completed_tasks: List[WeldTask] = []
        self.task_queue: List[WeldTask] = plan.tasks.copy()
        
        # Mode-specific state
        if plan.mode == WeldMode.WOM:
            self.current_window = 0
            self.window_start_time = 0.0
            self._initialize_wom_mode()
        elif plan.mode == WeldMode.SAW:
            self.current_stop_idx = 0
            self.at_stop = False
            self._initialize_saw_mode()
        
        # Statistics
        self.stats = {
            'total_weld_length': sum(w.length for w in state.welds),
            'completed_weld_length': 0.0,
            'robot_idle_time': {r.id: 0.0 for r in state.robots},
            'robot_weld_time': {r.id: 0.0 for r in state.robots},
            'robot_move_time': {r.id: 0.0 for r in state.robots},
            'collision_waits': 0
        }
        
        print(f"\nSimulator initialized: {plan.mode.value} mode")
        print(f"  Robots: {len(state.robots)} - {[r.id for r in state.robots]}")
        print(f"  Tasks: {len(plan.tasks)}")
        print(f"  Welds: {len(state.welds)}")
        print(f"  Total weld length: {self.stats['total_weld_length']:.0f}mm")
        print(f"  Gantry at X={state.gantry.x:.0f}mm, speed={state.gantry.speed:.0f}mm/s")
        for r in state.robots:
            print(f"  {r.id}: Y={r.current_y:.0f}mm, state={r.state.value}")
    
    def _initialize_wom_mode(self):
        """Initialize for WOM execution"""
        if not self.plan.wom_windows:
            return
        
        # Initialize all robots to their workspace centers first
        for robot in self.state.robots:
            robot.current_y = robot.workspace_center
            robot.state = RobotState.IDLE
        
        # Position robots for first window
        first_window = self.plan.wom_windows[0]
        for task in first_window:
            robot = self._get_robot(task.robot_id)
            robot.target_y = task.y_position
            robot.state = RobotState.MOVING_Y
        
        # Set gantry to start position (will move to window start)
        if first_window:
            window_start_x = min(t.start_x for t in first_window)
            self.state.gantry.target_x = window_start_x
            self.state.gantry.is_moving = True
            print(f"WOM mode initialized: {len(self.plan.wom_windows)} windows")
            print(f"  Window 1 starts at X={window_start_x:.0f}mm")
            print(f"  Active robots: {[t.robot_id for t in first_window]}")
    
    def _initialize_saw_mode(self):
        """Initialize for SAW execution"""
        if not self.plan.saw_stops:
            return
        
        # Initialize all robots to IDLE state at their workspace centers
        for robot in self.state.robots:
            robot.state = RobotState.IDLE
            robot.current_y = robot.workspace_center
            robot.weld_queue = []
        
        # Move to first stop
        first_stop = self.plan.saw_stops[0]
        self.state.gantry.target_x = first_stop
        self.state.gantry.is_moving = True
        
        print(f"SAW mode initialized: {len(self.plan.saw_stops)} stops, first at X={first_stop:.0f}mm")
    
    def step(self) -> bool:
        """
        Execute one simulation time step.
        
        Returns:
            True if simulation should continue, False if complete
        """
        if self.state.is_complete:
            return False
        
        # Update simulation time
        self.state.time += self.dt
        
        # Execute based on mode
        if self.plan.mode == WeldMode.WOM:
            self._step_wom()
        elif self.plan.mode == WeldMode.SAW:
            self._step_saw()
        
        # Update statistics
        self._update_stats()
        
        return not self.state.is_complete
    
    def _step_wom(self):
        """Execute one step of WOM mode"""
        # Check if current window is complete
        if self.current_window >= len(self.plan.wom_windows):
            self.state.gantry.is_moving = False
            return
        
        current_window_tasks = self.plan.wom_windows[self.current_window]
        
        # Get window X range
        window_x_start = min(t.start_x for t in current_window_tasks)
        window_x_end = max(t.weld.x_end for t in current_window_tasks)
        
        # Phase 0: Move gantry to window start position (if not there yet)
        if self.state.gantry.x < window_x_start - 1.0:
            # Gantry hasn't reached window start yet
            distance_to_start = abs(self.state.gantry.x - window_x_start)
            max_move = self.state.gantry.speed * self.dt
            actual_move = min(max_move, distance_to_start)
            self.state.gantry.x += actual_move
            return  # Don't start welding until gantry is at start position
        
        # Phase 1: Position robots at target Y (happens in parallel with Phase 0)
        all_positioned = True
        for task in current_window_tasks:
            robot = self._get_robot(task.robot_id)
            
            if robot.state == RobotState.MOVING_Y:
                # Move robot to target Y without overshooting
                distance_to_target = abs(robot.current_y - task.y_position)
                if distance_to_target > 1.0:
                    direction = 1 if task.y_position > robot.current_y else -1
                    max_move = robot.tcp_speed * self.dt
                    actual_move = min(max_move, distance_to_target)  # Don't overshoot
                    robot.current_y += direction * actual_move
                    all_positioned = False
                else:
                    robot.current_y = task.y_position
                    robot.state = RobotState.IDLE
        
        # Phase 2: Arc ignition ONLY when ALL robots positioned (NO MOVING ARC IGNITION rule)
        # Gantry must be STOPPED during positioning and arc ignition
        if all_positioned:
            # All robots are now at their Y positions
            # Check if any robot is still IDLE (hasn't ignited arc yet)
            any_not_welding = any(
                self._get_robot(t.robot_id).state == RobotState.IDLE and t.weld.done == 0
                for t in current_window_tasks
            )
            
            if any_not_welding:
                # Ignite all arcs SIMULTANEOUSLY (gantry still stopped)
                for task in current_window_tasks:
                    robot = self._get_robot(task.robot_id)
                    
                    if robot.state == RobotState.IDLE and task.weld.done == 0:
                        # Check collision zone
                        if self.collision_manager.try_acquire_lock(robot.id, robot.current_y):
                            robot.state = RobotState.WELDING
                            robot.current_weld = task.weld
                            self.active_tasks[robot.id] = task
                        else:
                            robot.state = RobotState.WAIT_MUTEX
                            self.stats['collision_waits'] += 1
                
                # After arc ignition, wait one step before moving gantry
                return
        
        # Phase 3: Move gantry while welding (happens continuously at WELDING SPEED)
        if self.state.gantry.x < window_x_end:
            # Still welding, keep moving gantry
            any_welding = any(r.state == RobotState.WELDING for r in self.state.robots)
            if any_welding:
                # CRITICAL: Gantry must move at robot TCP speed, not at gantry max speed!
                # In WOM mode, gantry speed = welding speed
                welding_robots = [r for r in self.state.robots if r.state == RobotState.WELDING]
                if welding_robots:
                    # Use the slowest welding robot's speed to ensure sync
                    wom_speed = min(r.tcp_speed for r in welding_robots)
                    distance_remaining = window_x_end - self.state.gantry.x
                    max_move = wom_speed * self.dt  # Use WELDING speed, not gantry speed!
                    actual_move = min(max_move, distance_remaining)
                    self.state.gantry.x += actual_move
        
        # Phase 4: Update welding progress
        for robot in self.state.robots:
            if robot.state == RobotState.WELDING:
                weld = robot.current_weld
                if weld:
                    weld.done += robot.tcp_speed * self.dt
                    
                    if weld.done >= weld.length:
                        # Weld complete
                        weld.done = weld.length
                        robot.state = RobotState.IDLE
                        robot.current_weld = None  # Clear current weld
                        robot.welds_completed += 1
                        self.collision_manager.release_lock(robot.id)
                        
                        if robot.id in self.active_tasks:
                            self.completed_tasks.append(self.active_tasks[robot.id])
                            del self.active_tasks[robot.id]
            
            elif robot.state == RobotState.WAIT_MUTEX:
                # Try to acquire lock again
                if self.collision_manager.try_acquire_lock(robot.id, robot.current_y):
                    robot.state = RobotState.WELDING
        
        # Phase 5: Check if window is complete
        all_welds_done = all(t.weld.is_completed for t in current_window_tasks)
        if all_welds_done and self.state.gantry.x >= window_x_end - 1.0:
            self._complete_wom_window()
    
    def _complete_wom_window(self):
        """Complete current WOM window and setup next"""
        print(f"  Window {self.current_window + 1} completed at t={self.state.time:.1f}s")
        
        # Release all locks
        for robot in self.state.robots:
            robot.state = RobotState.IDLE
            robot.current_weld = None
            self.collision_manager.release_lock(robot.id)
        
        self.current_window += 1
        
        if self.current_window < len(self.plan.wom_windows):
            # Setup next window
            next_window = self.plan.wom_windows[self.current_window]
            for task in next_window:
                robot = self._get_robot(task.robot_id)
                robot.target_y = task.y_position
                robot.state = RobotState.MOVING_Y
    
    def _step_saw(self):
        """Execute one step of SAW mode"""
        # Check if all stops processed
        if self.current_stop_idx >= len(self.plan.saw_stops):
            self.state.gantry.is_moving = False
            return
        
        current_stop = self.plan.saw_stops[self.current_stop_idx]
        
        # Phase 1: Move gantry to stop
        if self.state.gantry.is_moving:
            distance_to_stop = abs(self.state.gantry.x - current_stop)
            
            # If close enough, snap to position
            if distance_to_stop <= 1.0:
                print(f"    Gantry arrived at stop {self.current_stop_idx+1}: X={current_stop:.1f}")
                self.state.gantry.x = current_stop
                self.state.gantry.is_moving = False
                self.at_stop = True
                self._assign_tasks_at_stop(current_stop)
            else:
                # Move towards stop (don't overshoot)
                direction = 1 if current_stop > self.state.gantry.x else -1
                max_move = self.state.gantry.speed * self.dt
                actual_move = min(max_move, distance_to_stop)  # Don't overshoot
                self.state.gantry.x += direction * actual_move
        
        # Phase 2: Execute tasks at stop
        if self.at_stop:
            all_complete = True
            
            for robot in self.state.robots:
                if robot.state == RobotState.IDLE and robot.weld_queue:
                    # Get next task
                    next_weld = robot.weld_queue[0]
                    
                    # Move to position
                    if abs(robot.current_y - next_weld.y) > 1.0:
                        direction = 1 if next_weld.y > robot.current_y else -1
                        robot.current_y += direction * robot.tcp_speed * 0.5 * self.dt
                        robot.state = RobotState.MOVING_Y
                        all_complete = False
                    else:
                        # At position, try to start welding
                        if self.collision_manager.try_acquire_lock(robot.id, robot.current_y):
                            robot.current_y = next_weld.y
                            robot.current_weld = next_weld
                            robot.state = RobotState.WELDING
                            all_complete = False
                        else:
                            robot.state = RobotState.WAIT_MUTEX
                            self.stats['collision_waits'] += 1
                            all_complete = False
                
                elif robot.state == RobotState.WELDING:
                    weld = robot.current_weld
                    if weld:
                        weld.done += robot.tcp_speed * self.dt
                        
                        if weld.done >= weld.length:
                            # Weld complete
                            weld.done = weld.length
                            robot.weld_queue.pop(0)
                            robot.state = RobotState.IDLE
                            robot.welds_completed += 1
                            self.collision_manager.release_lock(robot.id)
                        else:
                            all_complete = False
                
                elif robot.state == RobotState.MOVING_Y:
                    all_complete = False
                
                elif robot.state == RobotState.WAIT_MUTEX:
                    # Try again
                    if robot.weld_queue:
                        next_weld = robot.weld_queue[0]
                        if self.collision_manager.try_acquire_lock(robot.id, next_weld.y):
                            robot.state = RobotState.IDLE
                    all_complete = False
            
            # Check if stop is complete
            if all_complete and all(not r.weld_queue for r in self.state.robots):
                self._complete_saw_stop()
    
    def _assign_tasks_at_stop(self, stop_x: float):
        """Assign tasks to robots at current stop"""
        stop_tasks = [t for t in self.plan.tasks if abs(t.start_x - stop_x) < 1.0]
        
        for task in stop_tasks:
            robot = self._get_robot(task.robot_id)
            if task.weld not in robot.weld_queue:
                robot.weld_queue.append(task.weld)
    
    def _complete_saw_stop(self):
        """Complete current SAW stop and move to next"""
        print(f"  Stop {self.current_stop_idx + 1} completed at t={self.state.time:.1f}s")
        
        self.current_stop_idx += 1
        self.at_stop = False
        
        if self.current_stop_idx < len(self.plan.saw_stops):
            # Move to next stop
            next_stop = self.plan.saw_stops[self.current_stop_idx]
            self.state.gantry.target_x = next_stop
            self.state.gantry.is_moving = True
    
    def _get_robot(self, robot_id: str) -> Robot:
        """Get robot by ID"""
        for robot in self.state.robots:
            if robot.id == robot_id:
                return robot
        raise ValueError(f"Robot {robot_id} not found")
    
    def _update_stats(self):
        """Update simulation statistics"""
        self.stats['completed_weld_length'] = sum(w.done for w in self.state.welds)
        
        for robot in self.state.robots:
            if robot.state == RobotState.IDLE:
                self.stats['robot_idle_time'][robot.id] += self.dt
            elif robot.state == RobotState.WELDING:
                self.stats['robot_weld_time'][robot.id] += self.dt
            elif robot.state == RobotState.MOVING_Y:
                self.stats['robot_move_time'][robot.id] += self.dt
    
    def get_progress(self) -> float:
        """Get overall progress percentage"""
        if self.stats['total_weld_length'] == 0:
            return 100.0
        return (self.stats['completed_weld_length'] / 
                self.stats['total_weld_length'] * 100)
    
    def get_stats_summary(self) -> str:
        """Generate statistics summary"""
        lines = [
            "\n" + "="*60,
            "SIMULATION STATISTICS",
            "="*60,
            f"Total time: {self.state.time:.1f}s",
            f"Progress: {self.get_progress():.1f}%",
            f"Collision waits: {self.stats['collision_waits']}",
            "",
            "Robot Performance:",
            "-"*60
        ]
        
        for robot in self.state.robots:
            rid = robot.id
            weld_time = self.stats['robot_weld_time'][rid]
            idle_time = self.stats['robot_idle_time'][rid]
            move_time = self.stats['robot_move_time'][rid]
            total_time = weld_time + idle_time + move_time
            
            utilization = (weld_time / total_time * 100) if total_time > 0 else 0
            
            lines.append(
                f"  {rid}: {robot.welds_completed} welds, "
                f"{utilization:.1f}% utilization"
            )
            lines.append(
                f"       Weld: {weld_time:.1f}s | "
                f"Move: {move_time:.1f}s | "
                f"Idle: {idle_time:.1f}s"
            )
        
        lines.append("="*60 + "\n")
        
        return '\n'.join(lines)
