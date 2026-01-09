# ui/renderer.py
"""
Visualization renderer for welding simulation.
Displays gantry, robots, welds, and collision zones in real-time.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
from planner.data_model import RobotState


class Renderer:
    """
    Animated visualization of welding simulation.
    """
    
    def __init__(self, simulator, scene_config):
        """
        Args:
            simulator: Simulator instance
            scene_config: Scene configuration dict
        """
        self.sim = simulator
        self.scene = scene_config
        
        # Setup figure
        self.fig, (self.ax_main, self.ax_stats) = plt.subplots(
            2, 1, figsize=(14, 10), 
            gridspec_kw={'height_ratios': [3, 1]}
        )
        
        # Color scheme
        self.state_colors = {
            RobotState.IDLE: "gray",
            RobotState.WELDING: "red",
            RobotState.MOVING_Y: "blue",
            RobotState.WAIT_MUTEX: "orange",
            RobotState.COMPLETED: "green"
        }
        
        self._setup_main_view()
        self._setup_stats_view()
        
        # Animation state
        self.frame_count = 0
        self.paused = False
    
    def _setup_main_view(self):
        """Setup main simulation view"""
        gantry = self.scene["gantry"]
        
        # Set axis limits
        self.ax_main.set_xlim(-200, gantry["x_length"] + 200)
        self.ax_main.set_ylim(-200, gantry["y_span"] + 200)
        
        self.ax_main.set_xlabel("X (Gantry axis) [mm]", fontsize=11)
        self.ax_main.set_ylabel("Y [mm]", fontsize=11)
        self.ax_main.set_title(
            f"Multi-Robot Welding Simulation - {self.sim.plan.mode.value.upper()}", 
            fontsize=13, fontweight='bold'
        )
        self.ax_main.grid(True, alpha=0.3)
        
        # Define gantry width FIRST (needed by other methods)
        self.gantry_width = 150  # Physical width of gantry beam in X direction
        
        # Draw interference zones (needs gantry_width)
        self._draw_interference_zones()
        
        # Draw welds (as lines)
        self._draw_welds()
        
        # Initialize gantry visualization (beam with width)
        self.gantry_rect = Rectangle(
            (0, 0), self.gantry_width, gantry["y_span"],
            facecolor='steelblue', alpha=0.3, edgecolor='darkblue',
            linewidth=2, zorder=8
        )
        self.ax_main.add_patch(self.gantry_rect)
        
        # Also keep center line for clarity
        self.gantry_line, = self.ax_main.plot(
            [], [], color="darkblue", linewidth=3, label="Gantry", zorder=10
        )
        
        # Robot side offsets (robots are mounted on edges of gantry)
        self.robot_x_offsets = {
            "x_plus": 300,   # x_plus side robots 300mm right of center
            "x_minus": -300  # x_minus side robots 300mm left of center
        }
        
        # Robot working range visualization
        self.robot_range_radius = 2000  # mm - robot reach
        self.robot_range_circles = []
        for _ in range(len(self.sim.state.robots)):
            circle = Circle(
                (0, 0), self.robot_range_radius,
                facecolor='lightblue', alpha=0.1, 
                edgecolor='blue', linewidth=1, linestyle='--',
                zorder=2
            )
            self.ax_main.add_patch(circle)
            self.robot_range_circles.append(circle)
        
        # Initialize robot markers
        self.robot_scatter = self.ax_main.scatter(
            [], [], s=250, zorder=15, edgecolors='black', linewidths=2
        )
        
        # Initialize robot labels
        self.robot_texts = []
        
        # Add legend
        self.ax_main.legend(loc='upper right', fontsize=9)
    
    def _draw_interference_zones(self):
        """Draw collision zones - these move with the gantry"""
        interference = self.scene.get("interference", {})
        
        # Store collision zone rectangles to update them during animation
        self.collision_zone_rects = []
        
        for name, (y_min, y_max) in interference.items():
            # Draw as semi-transparent rectangle that will move with gantry
            # Width matches gantry width, positioned at gantry center
            rect = Rectangle(
                (0, y_min), self.gantry_width, y_max - y_min,
                facecolor='yellow', alpha=0.15, edgecolor='orange',
                linewidth=1, linestyle='--', zorder=7
            )
            self.ax_main.add_patch(rect)
            self.collision_zone_rects.append((rect, name, y_min, y_max))
            
            # Add label (will also move with gantry)
            label_text = self.ax_main.text(
                self.gantry_width / 2, (y_min + y_max) / 2, name,
                fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5),
                zorder=9, ha='center'
            )
            self.collision_zone_rects.append((label_text, name, y_min, y_max))
    
    def _draw_welds(self):
        """Draw all weld seams"""
        for weld in self.sim.state.welds:
            # Draw weld line
            self.ax_main.plot(
                [weld.x_start, weld.x_end], [weld.y, weld.y],
                color='lightgreen', linewidth=6, alpha=0.3,
                solid_capstyle='round', zorder=3
            )
            
            # Add weld ID label
            mid_x = (weld.x_start + weld.x_end) / 2
            self.ax_main.text(
                mid_x, weld.y + 80, f"W{weld.id}",
                fontsize=8, ha='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7),
                zorder=4
            )
    
    def _setup_stats_view(self):
        """Setup statistics panel"""
        self.ax_stats.axis('off')
        self.stats_text = self.ax_stats.text(
            0.05, 0.5, '', fontsize=10, verticalalignment='center',
            family='monospace'
        )
    
    def _update(self, frame):
        """Update function for animation"""
        if self.paused:
            return []
        
        # Run simulation step
        cont = self.sim.step()
        if not cont:
            print("\nSimulation complete!")
            print(self.sim.get_stats_summary())
            return []
        
        self.frame_count += 1
        
        # Update gantry position (both rectangle and center line)
        x = self.sim.state.gantry.x
        y_span = self.scene["gantry"]["y_span"]
        
        # Update center line
        self.gantry_line.set_data([x, x], [0, y_span])
        
        # Update gantry rectangle to follow gantry position
        self.gantry_rect.set_x(x - self.gantry_width / 2)
        
        # Update collision zone positions to follow gantry
        for item, *rest in self.collision_zone_rects:
            if isinstance(item, Rectangle):
                # Update rectangle position
                item.set_x(x - self.gantry_width / 2)
            else:
                # Update text label position
                name, y_min, y_max = rest
                item.set_position((x, (y_min + y_max) / 2))
        
        # Update robot positions with side offsets
        robot_positions = []
        robot_colors = []
        
        for robot in self.sim.state.robots:
            # Get side offset for this robot
            side_offset = self.robot_x_offsets.get(robot.side, 0)
            
            # Position robot at gantry X + side offset
            robot_x = x + side_offset
            robot_positions.append([robot_x, robot.current_y])
            robot_colors.append(self.state_colors.get(robot.state, "black"))
        
        robot_positions = np.array(robot_positions)
        self.robot_scatter.set_offsets(robot_positions)
        self.robot_scatter.set_facecolors(robot_colors)
        
        # Update robot working range circles
        for i, (robot, pos) in enumerate(zip(self.sim.state.robots, robot_positions)):
            if i < len(self.robot_range_circles):
                # Move circle to robot position
                self.robot_range_circles[i].center = (pos[0], pos[1])
        
        # Update robot labels
        for txt in self.robot_texts:
            txt.remove()
        self.robot_texts.clear()
        
        for robot, pos in zip(self.sim.state.robots, robot_positions):
            # Label with robot ID and state
            label = f"{robot.id}\n{robot.state.value}"
            txt = self.ax_main.text(
                pos[0] + 150, pos[1], label,
                fontsize=8, va='center',
                bbox=dict(boxstyle='round,pad=0.4', 
                         facecolor='white', alpha=0.8, edgecolor='black'),
                zorder=20
            )
            self.robot_texts.append(txt)
        
        # Update weld progress (redraw with progress overlay)
        for weld in self.sim.state.welds:
            if weld.done > 0:
                # Draw completed portion in darker green
                progress_x = weld.x_start + (weld.x_end - weld.x_start) * (weld.done / weld.length)
                self.ax_main.plot(
                    [weld.x_start, progress_x], [weld.y, weld.y],
                    color='darkgreen', linewidth=8, alpha=0.7,
                    solid_capstyle='round', zorder=5
                )
        
        # Update statistics
        self._update_stats_display()
        
        return [self.gantry_line, self.gantry_rect, self.robot_scatter] + self.robot_texts
    
    def _update_stats_display(self):
        """Update statistics text"""
        stats = []
        stats.append("=" * 80)
        stats.append(f"Time: {self.sim.state.time:6.1f}s  |  "
                    f"Progress: {self.sim.get_progress():5.1f}%  |  "
                    f"Mode: {self.sim.plan.mode.value.upper()}")
        stats.append("-" * 80)
        
        # Robot status
        stats.append("Robot Status:")
        for robot in self.sim.state.robots:
            state_str = robot.state.value.ljust(12)
            weld_info = ""
            if robot.current_weld:
                progress = robot.current_weld.progress_percent
                weld_info = f"W{robot.current_weld.id} ({progress:.0f}%)"
            
            stats.append(
                f"  {robot.id}: {state_str} | "
                f"Welds: {robot.welds_completed}  | "
                f"{weld_info}"
            )
        
        stats.append("-" * 80)
        stats.append(f"Collision waits: {self.sim.stats['collision_waits']}  |  "
                    f"Frame: {self.frame_count}")
        stats.append("=" * 80)
        
        self.stats_text.set_text('\n'.join(stats))
    
    def _on_key(self, event):
        """Handle keyboard events"""
        if event.key == ' ':
            self.paused = not self.paused
            print("Paused" if self.paused else "Resumed")
        elif event.key == 'q':
            plt.close(self.fig)
    
    def show(self, interval: int = 50, save_animation: bool = False):
        """
        Start the animation.
        
        Args:
            interval: Time between frames in milliseconds
            save_animation: If True, save animation to file
        """
        print("\nStarting animation...")
        print("Controls: SPACE=pause/resume, Q=quit")
        
        # Connect keyboard handler
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        
        # Create animation
        self.anim = FuncAnimation(
            self.fig,
            self._update,
            interval=interval,
            blit=False,
            cache_frame_data=False,
            repeat=False
        )
        
        if save_animation:
            print("Saving animation (this may take a while)...")
            self.anim.save('welding_simulation.mp4', fps=20, 
                          extra_args=['-vcodec', 'libx264'])
            print("Animation saved to welding_simulation.mp4")
        
        plt.tight_layout()
        plt.show()
