import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Renderer:
    def __init__(self, simulator, scene):
        self.sim = simulator
        self.scene = scene

        self.fig, self.ax = plt.subplots(figsize=(10, 6))

        self.color_map = {
            "IDLE": "gray",
            "WELDING": "red",
            "WAIT_MUTEX": "orange"
        }

        # 坐标轴范围
        self.ax.set_xlim(0, self.sim.gantry.length)
        self.ax.set_ylim(scene["y_range"][0], scene["y_range"][1])

        self.ax.set_xlabel("X (gantry)")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Multi-Robot Welding Simulation")

        # 干涉区
        self._draw_interference_zones()

        # 龙门（竖线）
        self.gantry_line, = self.ax.plot([], [], color="blue", linewidth=2)

        # 机器人散点
        self.robot_scatter = self.ax.scatter([], [], s=120)

        # 机器人文字
        self.robot_texts = []

    # =============================
    # 干涉区背景
    # =============================
    def _draw_interference_zones(self):
        zones = self.scene.get("interference", {})
        for name, yr in zones.items():
            self.ax.axhspan(yr[0], yr[1], color="lightgray", alpha=0.4)
            self.ax.text(
                10,
                (yr[0] + yr[1]) / 2,
                name,
                fontsize=10,
                verticalalignment="center"
            )

    # =============================
    # 动画初始化
    # =============================
    def _init_anim(self):
        self.gantry_line.set_data([], [])
        # ⚠ 关键修复：必须是 (0, 2)
        self.robot_scatter.set_offsets(np.empty((0, 2)))
        return [self.gantry_line, self.robot_scatter]

    # =============================
    # 每一帧更新
    # =============================
    def _update(self, frame):
        cont = self.sim.step()
        if not cont:
            return []

        # 龙门位置
        x = self.sim.gantry.x
        y0, y1 = self.scene["y_range"]
        self.gantry_line.set_data([x, x], [y0, y1])

        # 机器人位置
        xs, ys, colors = [], [], []
        for r in self.sim.robots:
            xs.append(x)
            ys.append((r.y_range[0] + r.y_range[1]) / 2)
            colors.append(self.color_map.get(r.state, "black"))

        offsets = np.column_stack((xs, ys))
        self.robot_scatter.set_offsets(offsets)
        self.robot_scatter.set_color(colors)

        # 更新文字
        for t in self.robot_texts:
            t.remove()
        self.robot_texts.clear()

        for r, rx, ry in zip(self.sim.robots, xs, ys):
            self.robot_texts.append(
                self.ax.text(
                    rx + 15,
                    ry,
                    f"{r.id}\n{r.state}",
                    fontsize=9,
                    verticalalignment="center"
                )
            )

        return [self.gantry_line, self.robot_scatter] + self.robot_texts

    # =============================
    # 启动动画
    # =============================
    def show(self):
        self.anim = FuncAnimation(
            self.fig,
            self._update,
            init_func=self._init_anim,
            interval=50,
            blit=False,
            cache_frame_data=False   # ← 关闭缓存，消除 warning
        )
        plt.show()
