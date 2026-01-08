# simulator/simulator.py
#时间推进
class Mutex:
    def __init__(self):
        self.owner = None

    def try_acquire(self, robot_id):
        if self.owner is None:
            self.owner = robot_id
            return True
        return self.owner == robot_id

    def release(self, robot_id):
        if self.owner == robot_id:
            self.owner = None

class Simulator:
    def __init__(self, plan, gantry, robots, scene):
        self.plan = plan
        self.robots = robots
        self.gantry = gantry
        self.scene = scene

        self.mutex_s1 = Mutex()
        self.mutex_s2 = Mutex()
        self.dt = 0.05
        self.t = 0.0
        self.current_step = 0

    def step(self):
        if self.current_step >= len(self.plan):
            return False

        step = self.plan[self.current_step]

        # 1. 龙门 WOM 推进
        self.gantry.x += self.gantry.speed * self.dt

        # 2. 更新每个机器人
        for r in self.robots:

            # 正在焊接
            if r.state == "WELDING":
                w = r.current_weld
                w.done += r.tcp_speed * self.dt
                if w.done >= w.length:
                    r.state = "IDLE"
                    r.current_weld = None

                    # 释放互斥锁
                    if in_zone(w.y, self.scene["interference"]["s1"]):
                        self.mutex_s1.release(r.id)
                    if in_zone(w.y, self.scene["interference"]["s2"]):
                        self.mutex_s2.release(r.id)

        # 3. 分配新焊缝（并行）
        for rid, welds in step["assign"].items():
            robot = next(r for r in self.robots if r.id == rid)

            if robot.state != "IDLE" or not welds:
                continue

            w = welds[0]

            # 判断是否需要互斥
            if robot.side == "x_plus" and in_zone(w.y, self.scene["interference"]["s1"]):
                if not self.mutex_s1.try_acquire(robot.id):
                    robot.state = "WAIT_MUTEX"
                    continue

            if robot.side == "x_minus" and in_zone(w.y, self.scene["interference"]["s2"]):
                if not self.mutex_s2.try_acquire(robot.id):
                    robot.state = "WAIT_MUTEX"
                    continue

            # 成功开始焊接
            welds.pop(0)
            robot.current_weld = w
            robot.state = "WELDING"

        self.t += self.dt
        return True

    #检测是否存在干涉区
    def in_zone(y, zone):
        return zone[0] <= y <= zone[1]

