from planner.data_model import Weld, Robot, Gantry
from planner.weld_planner import WeldPlanner
from simulator.simulator import Simulator
from ui.renderer import Renderer


def main():
    # =============================
    # 1. 场景参数（scene）
    # =============================
    scene = {
        "y_range": (-2000, 2000),

        # 同侧干涉区（s1 / s2）
        "interference": {
            "s1": (-300, 300),     # x+ 侧 w1 / w3
            "s2": (-300, 300)      # x- 侧 w2 / w4
        }
    }

    # =============================
    # 2. 龙门
    # =============================
    gantry = Gantry(
        x=0.0,
        speed=50.0
    )
    gantry.length = 4000

    # =============================
    # 3. 机器人（4 台）
    # =============================
    robots = [
        Robot("W1", side="x+", y_range=(-1800, -300), tcp_speed=20),
        Robot("W3", side="x+", y_range=(300, 1800), tcp_speed=20),
        Robot("W2", side="x-", y_range=(-1800, -300), tcp_speed=20),
        Robot("W4", side="x-", y_range=(300, 1800), tcp_speed=20),
    ]

    # =============================
    # 4. 焊缝（示例）
    # =============================
    welds = [
        Weld(1, 0, 3000, -1000, 3000, "x+"),
        Weld(2, 500, 2500, 1000, 2000, "x-"),
        Weld(3, 800, 1800, 0, 1000, "x+"),
    ]

    # =============================
    # 5. 规划
    # =============================
    planner = WeldPlanner(welds, robots)
    plan = planner.plan()

    # =============================
    # 6. 仿真 + 渲染
    # =============================
    sim = Simulator(plan, gantry, robots, scene)
    renderer = Renderer(sim, scene)
    renderer.show()


if __name__ == "__main__":
    main()
