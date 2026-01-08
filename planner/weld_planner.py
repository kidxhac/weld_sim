from planner.data_model import Weld, Robot


class WeldPlanner:
    def __init__(self, welds: list[Weld], robots: list[Robot]):
        """
        welds  : 所有待焊焊缝
        robots : 全部机器人资源
        """
        print("weld_planner loaded")
        self.welds = welds
        self.robots = robots

    def plan(self):
        """
        输出一个最简单的初始计划：
        - 每条焊缝分配给 side 匹配的机器人
        - 暂不考虑时间，仅生成任务序列
        """

        plan = []

        for w in self.welds:
            candidates = [
                r for r in self.robots
                if r.side == w.side
            ]

            if not candidates:
                continue

            # 简单策略：选第一个（后续会替换为 WOM / 负载均衡）
            robot = candidates[0]

            plan.append({
                "robot": robot,
                "weld": w,
                "type": "WELD"
            })

        return plan
