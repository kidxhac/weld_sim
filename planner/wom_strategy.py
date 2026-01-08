# planner/wom_strategy.py
# WOM 分窗策略
def split_wom_windows(welds, length_ratio=0.3):
    welds = sorted(welds, key=lambda w: w.x_start)
    windows = []

    while welds:
        base = welds.pop(0)
        group = [base]

        for w in welds[:]:
            if abs(w.length - base.length) / base.length < length_ratio:
                group.append(w)
                welds.remove(w)

        windows.append(group)

    return windows
