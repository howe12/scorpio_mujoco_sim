#!/usr/bin/env python3
"""生成「三条线路关系 + 开发路线图」示意图。"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon

plt.rcParams["font.sans-serif"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130
OUT = "/develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim/docs/figures"


def panel_stack(ax):
    """左：系统栈分层 + 三条线路的作用位置。"""
    ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.axis("off")

    # 栈层（自上而下）
    layers = [
        ("配置层", "Q/R/Rd、max_speed、replan_interval\ninflation_radius、traj_publish_step", 8.15, "#ffeaa7"),
        ("规划层", "Kinodynamic A* → 轨迹优化 → 验收\n(SDF / 姿态 / 风险)", 6.35, "#74b9ff"),
        ("控制层", "MPC 参考提取 → SQP → cmd_vel\n重规划时基 / 限幅", 4.55, "#ff9f9f"),
        ("执行层", "MuJoCo 物理 + 阿克曼底盘", 2.95, "#dfe6e9"),
    ]
    for name, desc, y, col in layers:
        ax.add_patch(FancyBboxPatch((0.3, y - 0.62), 6.0, 1.24,
                                    boxstyle="round,pad=0.06", fc=col, ec="#2d3436", lw=1.3))
        ax.text(0.55, y + 0.30, name, fontsize=10.5, weight="bold", va="center")
        ax.text(0.55, y - 0.22, desc, fontsize=7.8, va="center", color="#2d3436",
                linespacing=1.5)

    # 三条线路的标注（右侧）。控制层有两条线路，上下错开以免重叠。
    tags = [
        ("线路 2  参数搜索", "改这里（数值）", 8.15, "#f0932b", "-"),
        ("线路 1  修缺陷", "改这里（逻辑）", 5.02, "#c0392b", "-"),
        ("线路 3  学习策略", "加在这里（旁路）", 3.98, "#8e44ad", "--"),
    ]
    for txt, sub, y, col, ls in tags:
        ax.add_patch(FancyBboxPatch((6.95, y - 0.40), 2.85, 0.80,
                                    boxstyle="round,pad=0.06", fc="white", ec=col,
                                    lw=1.8, ls=ls))
        ax.text(8.37, y + 0.13, txt, fontsize=8.8, ha="center", weight="bold", color=col)
        ax.text(8.37, y - 0.19, sub, fontsize=7.2, ha="center", color=col)
        ax.add_patch(FancyArrowPatch((6.95, y), (6.34, y), arrowstyle="-|>",
                                     mutation_scale=14, color=col, lw=1.7, ls=ls))
    ax.text(8.37, 6.90, "规划层保持不变\n（线路 3 不替换它）", fontsize=7.2,
            ha="center", va="center", color="#636e72", style="italic",
            bbox=dict(boxstyle="round,pad=0.28", fc="#f8f9fa", ec="#dfe6e9"))

    ax.text(5.0, 9.72, "三条线路作用在系统栈的不同层", fontsize=12, ha="center",
            weight="bold")
    ax.text(5.0, 1.72,
            "线路 1 与线路 2 是「串行依赖」：逻辑有缺陷时，参数再优也会被缺陷吞掉\n"
            "线路 3 是「旁路增强」：不替换规划层，只在其输出上叠加小修正",
            fontsize=8.4, ha="center", va="top", color="#2d3436",
            linespacing=1.7,
            bbox=dict(boxstyle="round,pad=0.45", fc="#f8f9fa", ec="#b2bec3"))


def panel_dag(ax):
    """右：依赖关系 + 阶段验收。"""
    ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.axis("off")
    ax.text(5.0, 9.72, "依赖顺序：不能跳步", fontsize=12, ha="center", weight="bold")

    nodes = [
        ("阶段 0\n修绕圈/航向发散", 1.55, 7.55, "#ff9f9f", "半日~1 日"),
        ("阶段 1\n评估锁定（重复2次）", 1.55, 5.35, "#74b9ff", "约 2 小时"),
        ("阶段 2\n参数搜索 CMA-ES", 1.55, 3.15, "#ffeaa7", "半日"),
        ("阶段 3\nPPO + RSI 残差", 1.55, 0.95, "#d5b3ff", "1~2 日"),
    ]
    for txt, x, y, col, cost in nodes:
        ax.add_patch(FancyBboxPatch((x - 0.15, y - 0.62), 4.6, 1.24,
                                    boxstyle="round,pad=0.06", fc=col, ec="#2d3436", lw=1.3))
        ax.text(x + 0.08, y + 0.18, txt, fontsize=9.4, va="center", weight="bold",
                linespacing=1.5)
        ax.text(x + 4.30, y - 0.34, cost, fontsize=7.4, ha="right", color="#636e72")

    # 串行箭头
    for y1, y2 in [(6.93, 5.97), (4.73, 3.77), (2.53, 1.57)]:
        ax.add_patch(FancyArrowPatch((2.15, y1), (2.15, y2), arrowstyle="-|>",
                                     mutation_scale=16, color="#2d3436", lw=1.8))

    # 旁注：验收门槛
    gates = [
        (7.55, "成功率 ≥ 80%\n3 次耗时标准差 < 20%"),
        (5.35, "10 次运行方差达标\nbaseline JSON 版本化"),
        (3.15, "均速 +15%，成功率不降\n碰撞仍为 0"),
        (0.95, "再 +15%，碰撞为 0\n仅当阶段 2 到瓶颈"),
    ]
    for y, txt in gates:
        ax.text(6.30, y, txt, fontsize=7.6, va="center", color="#2d3436",
                linespacing=1.6,
                bbox=dict(boxstyle="round,pad=0.32", fc="#f8f9fa", ec="#b2bec3"))
        ax.add_patch(FancyArrowPatch((4.45, y), (6.05, y), arrowstyle="-|>",
                                     mutation_scale=12, color="#b2bec3", lw=1.2,
                                     ls=":"))

    # 禁止跳步：画在左侧留白，避免与阶段框交叠
    ax.add_patch(FancyArrowPatch((0.62, 7.55), (0.62, 0.95), arrowstyle="-",
                                 color="#c0392b", lw=2.0, ls=(0, (5, 4))))
    for y in (7.55, 0.95):
        ax.plot([0.50, 0.74], [y, y], color="#c0392b", lw=2.0)
    ax.text(0.22, 4.25, "跳过阶段 0 直接训练 = 在噪声上优化", fontsize=7.6,
            ha="center", color="#c0392b", rotation=90, va="center")
    ax.text(5.0, 0.05, "阶段 3 是「可选」而非「必经」：阶段 2 若把均速提到 0.35 m/s，则不值得上 PPO",
            fontsize=7.8, ha="center", color="#636e72", style="italic")


fig = plt.figure(figsize=(15.2, 5.9))
gs = fig.add_gridspec(1, 2, width_ratios=[1.12, 1.0], wspace=0.10,
                      left=0.02, right=0.985, top=0.93, bottom=0.06)
panel_stack(fig.add_subplot(gs[0]))
panel_dag(fig.add_subplot(gs[1]))
fig.suptitle("三条线路的关系：作用层次不同、存在串行依赖，而非并列选项",
             fontsize=13.5, y=0.985)
fig.savefig(f"{OUT}/fig10_three_tracks.png", bbox_inches="tight", facecolor="white")
print("saved fig10_three_tracks.png")
