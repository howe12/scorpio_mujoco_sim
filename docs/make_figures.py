#!/usr/bin/env python3
"""生成 SEB-Naver 原理示意图（论文 ↔ 本工程实现对照）。

输出到 docs/figures/：
  fig1_why_se2.png         为什么必须用 SE(2) 而非 R²（论文 Fig.2a）
  fig2_algorithm1.png      Algorithm 1 的几何原理：椭圆足迹 → 法线 → 姿态
  fig3_flatness.png        微分平坦 + s(t) 如何消除低速奇异性
  fig4_pipeline.png        完整流水线：论文模块 ↔ 本工程代码
  fig5_optimization.png    轨迹优化问题的变量/约束/目标关系

运行::

    /usr/bin/python3 docs/make_figures.py
"""

from __future__ import annotations

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch, Polygon

plt.rcParams["font.sans-serif"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
os.makedirs(OUT, exist_ok=True)


def save(fig, name: str):
    path = os.path.join(OUT, name)
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  ✅ {name}")


# ---------------------------------------------------------------------------
# 图 1: 为什么必须 SE(2)
# ---------------------------------------------------------------------------
def fig1_why_se2():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))

    # 左: R² 栅格 —— 一个格子混了所有朝向
    ax = axes[0]
    ax.set_title("传统方法：R² 栅格（每个格子一个风险值）", fontsize=11, pad=10)
    # 画一个斜的坡面
    xs = np.linspace(0, 4, 100)
    for y in np.linspace(0, 4, 9):
        ax.plot(xs, np.full_like(xs, y), color="#dfe6e9", lw=1, zorder=1)
    for x in np.linspace(0, 4, 9):
        ax.plot(np.full_like(xs, x), xs, color="#dfe6e9", lw=1, zorder=1)

    # 同一个格子，两种朝向
    cell = (2.0, 2.0)
    for (yaw, color, lbl, risk) in [
        (0.0, "#0984e3", r"$\theta_1$（车头朝上坡）", 0.25),
        (math.pi / 2, "#d63031", r"$\theta_2$（车头朝坡侧）", 0.75),
    ]:
        ax.add_patch(Ellipse(cell, 0.9, 0.62, angle=math.degrees(yaw),
                             facecolor=color, alpha=0.45, edgecolor=color, lw=2, zorder=3))
        dx, dy = 0.55 * math.cos(yaw), 0.55 * math.sin(yaw)
        ax.annotate("", xy=(cell[0] + dx, cell[1] + dy), xytext=cell,
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=2.2), zorder=4)
    ax.text(cell[0] + 0.15, cell[1] - 0.95,
            "两个朝向落在同一格\n→ 风险只能取一个值\n（保守或激进）",
            fontsize=9, color="#2d3436", ha="center",
            bbox=dict(boxstyle="round,pad=0.35", fc="#ffeaa7", ec="#fdcb6e"))
    ax.set_xlim(0, 4); ax.set_ylim(0, 4)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(handles=[
        plt.Line2D([], [], color="#0984e3", lw=6, alpha=0.45, label=r"$\theta_1$: Risk=0.25"),
        plt.Line2D([], [], color="#d63031", lw=6, alpha=0.45, label=r"$\theta_2$: Risk=0.75"),
    ], loc="upper left", fontsize=8.5, framealpha=0.95)

    # 右: SE(2) 栅格 —— 每个朝向独立
    ax = axes[1]
    ax.set_title("SEB-Naver：SE(2) 栅格（每个朝向独立评估）", fontsize=11, pad=10)
    n_yaw = 32
    risks = [0.25 + 0.5 * (0.5 - 0.5 * math.cos(2 * (i * 2 * math.pi / n_yaw)))
             for i in range(n_yaw)]
    ang = np.linspace(0, 2 * math.pi, n_yaw, endpoint=False)
    for i, a in enumerate(ang):
        c = plt.cm.RdYlGn_r(risks[i])
        ax.add_patch(plt.Rectangle((2.0 + 1.1 * math.cos(a) - 0.11,
                                    2.0 + 1.1 * math.sin(a) - 0.11),
                                   0.22, 0.22, facecolor=c, edgecolor="white", lw=1))
    ax.add_patch(plt.Circle((2.0, 2.0), 0.32, facecolor="#dfe6e9", edgecolor="#636e72"))
    ax.text(2.0, 2.0, "同一\n(x,y)", ha="center", va="center", fontsize=8.5)
    ax.text(2.0, 0.45, "每个朝向一个独立风险值\n→ 同一位置可按朝向区分",
            fontsize=9, ha="center",
            bbox=dict(boxstyle="round,pad=0.35", fc="#dff9fb", ec="#22a6b3"))
    sm = plt.cm.ScalarMappable(cmap="RdYlGn_r", norm=plt.Normalize(0, 1))
    cb = fig.colorbar(sm, ax=ax, fraction=0.045, pad=0.03)
    cb.set_label("Risk", fontsize=9)
    ax.set_xlim(0, 4); ax.set_ylim(0, 4)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]")
    ax.set_yticks([])

    fig.suptitle("图 1  为什么必须用 SE(2)：同一位置不同朝向，风险截然不同",
                 fontsize=12.5, y=1.02)
    fig.text(0.5, -0.06,
             "论文 Fig.2(a) 的核心论点：车体对横向倾斜(roll)与纵向倾斜(pitch)的容忍度不同，"
             "R² 栅格把两者混为一谈。\n"
             "代价：SE(2) 的格子数是 R² 的 32 倍（yaw 分辨率 0.2 rad）",
             ha="center", fontsize=9, color="#2d3436")
    save(fig, "fig1_why_se2.png")


# ---------------------------------------------------------------------------
# 图 2: Algorithm 1 几何原理
# ---------------------------------------------------------------------------
def fig2_algorithm1():
    fig = plt.figure(figsize=(13.0, 5.0))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.0, 1.0, 1.12], wspace=0.24,
                          left=0.04, right=0.98, top=0.80, bottom=0.16)

    # (a) 俯视图：椭圆足迹覆盖的地形点
    ax = fig.add_subplot(gs[0])
    ax.set_title("① 取椭圆足迹内的地形点", fontsize=10.5, pad=8)
    gx, gy = np.meshgrid(np.linspace(-0.7, 0.7, 60), np.linspace(-0.7, 0.7, 60))
    gz = 0.55 * gx + 0.12 * np.sin(3 * gy)
    im = ax.contourf(gx, gy, gz, levels=14, cmap="terrain", alpha=0.85)
    ax.contour(gx, gy, gz, levels=14, colors="white", linewidths=0.4, alpha=0.55)
    th = np.linspace(0, 2 * np.pi, 120)
    ax.plot(0.36 * np.cos(th), 0.27 * np.sin(th), color="#0984e3", lw=2.4,
            label="车体足迹椭圆 $e_x \\times e_y$")
    m = np.linspace(0, 2 * np.pi, 15, endpoint=False)
    sx, sy = 0.84 * 0.36 * np.cos(m), 0.84 * 0.27 * np.sin(m)
    ax.scatter(sx, sy, color="#d63031", s=26, zorder=5,
               edgecolor="white", lw=0.8, label="采样点 $P$（约 15 个）")
    ax.annotate("", xy=(0.30, 0.0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#2d3436", lw=2))
    ax.text(0.17, 0.04, "$x^b$", fontsize=9)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=7.5, loc="lower left", framealpha=0.95)
    ax.text(0.98, 0.02, "底色 = 高程 z", transform=ax.transAxes, ha="right",
            fontsize=7.6, color="#2d3436",
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#b2bec3", lw=0.6))

    # (b) 侧视图：点 → 拟合平面 → 法线 → 倾角
    ax = fig.add_subplot(gs[1])
    ax.set_title("② 协方差 → 最小特征向量即法线", fontsize=10.5, pad=8)
    xs = np.linspace(-0.7, 0.7, 40)
    ax.plot(xs, 0.45 * xs, color="#00b894", lw=2.4, label="地形切面（拟合平面）")
    pxs = np.linspace(-0.6, 0.6, 13)
    rng = np.random.default_rng(5)
    ax.scatter(pxs, 0.45 * pxs + rng.normal(0, 0.028, pxs.size),
               color="#d63031", s=30, zorder=5, edgecolor="white", lw=0.8,
               label="足迹内的点 $p_j$")
    # 法线垂直于坡面
    ang = math.atan(0.45)
    nx, ny = -math.sin(ang), math.cos(ang)
    ax.annotate("", xy=(0.80 * nx, 0.80 * ny), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#e17055", lw=2.8))
    ax.text(0.86 * nx + 0.04, 0.80 * ny - 0.02, "法线 $n$\n= 最小特征向量",
            color="#e17055", fontsize=8.0, va="center")
    # 面内两个主方向（示意）
    ax.annotate("", xy=(0.75 * math.cos(ang), 0.75 * math.sin(ang)), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#0984e3", lw=2))
    ax.text(0.55, 0.14, "$\\lambda_{max}$ 方向（面内）", color="#0984e3", fontsize=7.6)
    # 倾角标注
    ax.plot([-0.7, 0.7], [0, 0], color="#b2bec3", lw=1, ls="--")
    ax.text(0.45, -0.10, "水平", fontsize=7.5, color="#636e72")
    arc = np.linspace(0, ang, 30)
    ax.plot(0.42 * np.cos(arc), 0.42 * np.sin(arc), color="#636e72", lw=1)
    ax.text(0.47, 0.07, f"倾角 {math.degrees(ang):.0f}°", fontsize=8, color="#636e72")
    ax.text(-0.68, 0.72,
            r"$\kappa_{ter}=\frac{3\lambda_{min}}{\Sigma\lambda}$" + "\n"
            r"（曲率，几何粗糙度）",
            fontsize=8.8, va="top",
            bbox=dict(boxstyle="round,pad=0.35", fc="#fdf6e3", ec="#b2bec3"))
    ax.set_xlim(-0.75, 1.05); ax.set_ylim(-0.22, 1.02)
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=7.5, loc="lower right", framealpha=0.95)
    ax.grid(alpha=0.25)

    # (c) 公式与判定
    ax = fig.add_subplot(gs[2])
    ax.axis("off")
    ax.set_title("③ 法线 → 机体轴 → 姿态角 → Risk", fontsize=10.5, pad=8)
    ax.text(0.5, 0.94,
            r"$x^b,y^b=\mathrm{GetXbYb}(n,\ s_r)$",
            ha="center", va="top", fontsize=10.5)
    ax.text(0.5, 0.80,
            r"$\phi_x=|\arcsin(b_3^{T}x^b)|$  俯仰" + "\n" +
            r"$\phi_y=|\arcsin(b_3^{T}y^b)|$  横滚",
            ha="center", va="top", fontsize=10.5, linespacing=1.9)
    ax.text(0.5, 0.60,
            r"$r=\left[\dfrac{\kappa_{ter}}{\kappa_{max}},\ "
            r"\dfrac{\phi_x}{\phi_{x,max}},\ "
            r"\dfrac{\phi_y}{\phi_{y,max}}\right]^{T}$",
            ha="center", va="top", fontsize=11.5)
    ax.text(0.5, 0.42, r"$Risk(s_r)=r^{T}w_r\ \in[0,1]$",
            ha="center", va="top", fontsize=12,
            bbox=dict(boxstyle="round,pad=0.42", fc="#dff9fb", ec="#22a6b3", lw=1.4))
    ax.text(0.5, 0.18,
            "任一硬限被突破 → Risk = 1\n（判定为显式障碍）",
            ha="center", va="top", fontsize=9,
            bbox=dict(boxstyle="round,pad=0.35", fc="#ffeaa7", ec="#fdcb6e"))
    ax.text(0.5, 0.02,
            "本工程: planner_node.py → risk_at()",
            ha="center", va="bottom", fontsize=7.8,
            color="#0984e3", style="italic")

    fig.suptitle("图 2  Algorithm 1：从地形点到可通行性风险（几何三步）", fontsize=13, y=0.97)
    fig.text(0.5, 0.035,
             "关键：风险不是「这块地陡不陡」，而是「车以这个姿态压上去会不会翻」",
             ha="center", fontsize=9.5, color="#2d3436")
    save(fig, "fig2_algorithm1.png")


# ---------------------------------------------------------------------------
# 图 3: 微分平坦 + s(t)
# ---------------------------------------------------------------------------
def fig3_flatness():
    fig, axes = plt.subplots(1, 3, figsize=(12.4, 3.7))
    fig.subplots_adjust(left=0.06, right=0.985, top=0.80, bottom=0.24, wspace=0.30)

    t = np.linspace(0, 6, 400)
    vmax = 0.5
    # 起停: v = vmax·sin²(πt/T)，首尾为 0
    v = vmax * np.sin(np.pi * t / 6.0) ** 2

    # (a) 速度
    ax = axes[0]
    ax.plot(t, v, color="#0984e3", lw=2.2)
    ax.axhspan(0, 0.03, color="#ff7675", alpha=0.28)
    ax.set_title("① 起停时速度趋于 0", fontsize=10.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel("v [m/s]")
    ax.set_ylim(-0.02, 0.58)
    ax.text(3.0, 0.10, r"$\dot x^2+\dot y^2\to 0$" + "\n方向不可信",
            ha="center", fontsize=8.6, color="#d63031")
    ax.grid(alpha=0.3)

    # (b) 直接由速度求航向 → 低速抖动
    ax = axes[1]
    rng = np.random.default_rng(11)
    true_th = np.full_like(t, 0.60)
    noise = np.where(v < 0.08, rng.normal(0, 2.6, t.size), 0.0)
    ax.plot(t, np.degrees(true_th), color="#00b894", lw=2.4, label="真实航向 34.4°")
    ax.plot(t, np.degrees(true_th) + np.degrees(noise), color="#d63031", lw=0.8,
            alpha=0.8, label=r"$\theta=\arctan2(\dot y,\dot x)$")
    ax.set_title("② 由速度求航向 → 起停处剧烈抖动", fontsize=10.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel("θ [deg]")
    ax.set_ylim(-220, 260)
    ax.legend(fontsize=7.6, loc="upper right", framealpha=0.95)
    ax.grid(alpha=0.3)

    # (c) 用弧长参数 s → 稳定
    ax = axes[2]
    th_s = np.full_like(t, 34.4)
    ax.plot(t, th_s, color="#0984e3", lw=2.6,
            label=r"$\theta=\arctan2(\eta\dot y(s),\eta\dot x(s))$")
    ax.fill_between(t, th_s - 0.6, th_s + 0.6, color="#0984e3", alpha=0.20)
    ax.set_title("③ 改用弧长参数 s(t) → 稳定", fontsize=10.5)
    ax.set_xlabel("t [s]（s 单调递增）"); ax.set_ylabel("θ [deg]")
    ax.set_ylim(32.0, 37.0)
    ax.legend(fontsize=7.6, loc="upper right", framealpha=0.95)
    ax.text(3.0, 32.35,
            r"约束 $\dot x^2(s)+\dot y^2(s)>\delta_+$" + "\n"
            r"$\theta$ 只依赖 s，不依赖 $t$" + "\n→ 支持低速与换挡 $\eta=\pm1$",
            ha="center", fontsize=8.2,
            bbox=dict(boxstyle="round,pad=0.32", fc="#dff9fb", ec="#22a6b3"))
    ax.grid(alpha=0.3)

    fig.suptitle("图 3  微分平坦 + 中间变量 s(t)：消除低速/换挡时的 arctan2 奇异性",
                 fontsize=12.5, y=0.96)
    fig.text(0.5, 0.045,
             "论文最精妙的一步，也是本工程【未实现】的部分："
             "我们把下界写成 v²≥ε 回避奇异，代价是不支持换挡（仅单向前进）。",
             ha="center", fontsize=9, color="#d63031")
    save(fig, "fig3_flatness.png")


# ---------------------------------------------------------------------------
# 图 4: 完整流水线（论文 ↔ 代码）
# ---------------------------------------------------------------------------
def fig4_pipeline():
    fig, ax = plt.subplots(figsize=(12.6, 6.2))
    ax.set_xlim(0, 12.6); ax.set_ylim(0, 6.2); ax.axis("off")

    stages = [
        ("① 传感器", "LiDAR 3D 点云\nIMU", "#dfe6e9", 0.55, 5.42),
        ("② 定位", "FAST-LIO2\niEKF + ikd-Tree", "#dfe6e9", 2.35, 5.42),
        ("③ 局部高程图", "SE(2) 栅格 20×20m\n32 yaw × 0.1m\nKalman 融合", "#74b9ff", 4.45, 5.42),
        ("④ 可通行性", "Algorithm 1\nRisk + 地形姿态", "#74b9ff", 6.75, 5.42),
        ("⑤ 路径搜索", "Kinodynamic A*\n+ Reeds-Shepp", "#55efc4", 8.95, 5.42),
        ("⑥ 轨迹优化", "五次多项式 NLP\nEq.22–29", "#55efc4", 8.95, 3.30),
        ("⑦ 控制", "MPC 跟踪\nCasADi SQP", "#ffeaa7", 8.95, 1.35),
        ("⑧ 机器人", "阿克曼底盘\nv, δ", "#dfe6e9", 6.60, 1.35),
    ]
    W, H = 1.75, 1.05
    centers = {}
    for name, sub, col, x, y in stages:
        ax.add_patch(FancyBboxPatch((x - W / 2, y - H / 2), W, H,
                                    boxstyle="round,pad=0.06", fc=col, ec="#2d3436", lw=1.2))
        ax.text(x, y + 0.20, name, ha="center", va="center", fontsize=9.6, weight="bold")
        ax.text(x, y - 0.19, sub, ha="center", va="center", fontsize=7.6, color="#2d3436")
        centers[name] = (x, y)

    def arrow(a, b, style="-|>", col="#2d3436", ls="-", rad=0.0):
        (x1, y1), (x2, y2) = centers[a], centers[b]
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style,
                                     mutation_scale=15, color=col, lw=1.5, ls=ls,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=52, shrinkB=52))

    arrow("① 传感器", "② 定位")
    arrow("② 定位", "③ 局部高程图")
    arrow("③ 局部高程图", "④ 可通行性")
    arrow("④ 可通行性", "⑤ 路径搜索")
    arrow("⑤ 路径搜索", "⑥ 轨迹优化")
    arrow("⑥ 轨迹优化", "⑦ 控制")
    arrow("⑦ 控制", "⑧ 机器人")
    # 闭环
    ax.add_patch(FancyArrowPatch((6.60, 0.82), (2.35, 4.88), arrowstyle="-|>",
                                 mutation_scale=15, color="#636e72", lw=1.3, ls="--",
                                 connectionstyle="arc3,rad=-0.20"))
    ax.text(4.25, 2.35, "里程计 / 位姿反馈", fontsize=8, color="#636e72", rotation=22)

    # 代码标注
    notes = [
        ("① 传感器", "scorpio_ros2_publisher.py", 0.0),
        ("② 定位", "FAST_LIO (patched)\n--no-tf-odom", 0.0),
        ("③ 局部高程图", "terrain_analyzer.cpp\nfuseAll()", 0.0),
        ("④ 可通行性", "planner_node.py\nrisk_at()", 0.0),
        ("⑤ 路径搜索", "kino_astar.py", 0.0),
        ("⑥ 轨迹优化", "trajectory_optimizer.py", 0.0),
        ("⑦ 控制", "mpc_node.py", 0.0),
        ("⑧ 机器人", "MuJoCo + scorpio_bridge.py", 0.0),
    ]
    for key, txt, dy in notes:
        x, y = centers[key]
        ax.text(x, y - 0.66 - dy, txt, ha="center", va="top", fontsize=6.8,
                color="#0984e3", style="italic",
                bbox=dict(boxstyle="round,pad=0.2", fc="#f8f9fa", ec="#b2bec3", lw=0.6))

    # 左右两栏说明
    ax.text(0.12, 3.05, "论文主线（实时性靠 GPU）", fontsize=10, weight="bold", color="#2d3436")
    ax.text(0.12, 2.73,
            "• ③④ 是关键创新：\n"
            "  SE(2) 让风险按朝向区分\n"
            "  GPU 并行使其可实时\n"
            "• ⑥ 用微分平坦把\n"
            "  地形关联运动学\n"
            "  解析地写进约束",
            fontsize=8.2, va="top", color="#2d3436")

    ax.text(10.50, 2.98, "本工程实现取舍", fontsize=10, weight="bold", color="#d63031")
    ax.text(10.50, 2.66,
            "• ③ terrain_analyzer\n"
            "  仅 2D 高程（无 yaw）\n"
            "• ④ risk 改按需算\n"
            "  只算轨迹点\n"
            "• ⑥ 用 IPOPT\n"
            "  替代 PHR-ALM",
            fontsize=8.2, va="top", color="#d63031")

    fig.suptitle("图 4  完整流水线：论文模块与代码文件的对应关系", fontsize=13, y=0.99)
    save(fig, "fig4_pipeline.png")


# ---------------------------------------------------------------------------
# 图 5: 优化问题结构
# ---------------------------------------------------------------------------
def fig5_optimization():
    fig, ax = plt.subplots(figsize=(12.4, 5.8))
    ax.set_xlim(0, 12.4); ax.set_ylim(0, 5.8); ax.axis("off")

    # ---- 变量 ----
    ax.add_patch(FancyBboxPatch((0.25, 3.55), 3.4, 1.90,
                                boxstyle="round,pad=0.08", fc="#dfe6e9", ec="#2d3436", lw=1.3))
    ax.text(1.95, 5.16, "优化变量（本工程）", ha="center", fontsize=10.5, weight="bold")
    ax.text(1.95, 4.48,
            r"$c_x,\ c_y\in\mathbb{R}^6$：每段五次多项式系数" + "\n"
            r"$T_f$：总时长（$T_p=T_f/N$）" + "\n"
            r"$N=3\ldots10$ 段  →  $\approx$61 个变量",
            ha="center", va="center", fontsize=8.6, linespacing=1.7)
    ax.text(1.95, 3.72, "论文另含 P、S、e_m 及矩阵消元",
            ha="center", fontsize=7.4, color="#d63031", style="italic")

    # ---- 约束 ----
    ax.add_patch(FancyBboxPatch((4.05, 2.02), 4.15, 3.44,
                                boxstyle="round,pad=0.08", fc="#dff9fb", ec="#22a6b3", lw=1.4))
    ax.text(6.12, 5.20, "约束（论文 Eq.23–29）", ha="center", fontsize=10.5, weight="bold")
    rows = [
        ("Eq.23-24", "段间 C⁴ 连续 + 起终点边界", "等价", "#00b894"),
        ("Eq.25", r"$\dot x^2+\dot y^2>\delta_+,\ T_f>0$", "近似", "#e17055"),
        ("Eq.26", r"$\delta^2\leq\delta_{max}^2,\ v_x^2\leq v_{max}^2$", "等价", "#00b894"),
        ("Eq.27", r"$a_x^2+a_y^2\leq a_{max}^2$", "等价", "#00b894"),
        ("Eq.27", r"$\phi_x^2,\ \phi_y^2$ 姿态上限", "近似", "#e17055"),
        ("Eq.28", r"$Risk\leq r_{max}$", "近似", "#e17055"),
        ("Eq.29", r"$G_c(s_r)\geq d_{min}$（SDF 避障）", "近似", "#e17055"),
    ]
    y = 4.74
    for tag, desc, status, col in rows:
        ax.text(4.30, y, tag, fontsize=7.8, color="#0a3d62", weight="bold")
        ax.text(5.28, y, desc, fontsize=7.8, color="#2d3436")
        ax.text(8.02, y, status, fontsize=7.4, ha="right", color=col, weight="bold")
        y -= 0.395
    ax.text(6.12, 2.20,
            "等价 = 同一数学约束    近似 = 语义相同、施加方式不同",
            ha="center", fontsize=7.4, color="#636e72")

    # ---- 目标 ----
    ax.add_patch(FancyBboxPatch((8.62, 3.55), 3.45, 1.90,
                                boxstyle="round,pad=0.08", fc="#ffeaa7", ec="#fdcb6e", lw=1.4))
    ax.text(10.35, 5.16, "目标（Eq.22）", ha="center", fontsize=10.5, weight="bold")
    ax.text(10.35, 4.42,
            r"$\min\ \int\|\mathbf{j}(t)\|^2 dt$" + "\n"
            r"$\quad+\ \rho_t\,T_f$" + "\n"
            r"$\quad+\ \rho_r\int Risk^2 dt$",
            ha="center", va="center", fontsize=10.0, linespacing=1.85)

    # ---- 求解器（独立一行，不与输出重叠）----
    ax.add_patch(FancyBboxPatch((4.05, 1.15), 4.15, 0.58,
                                boxstyle="round,pad=0.06", fc="#a29bfe", ec="#2d3436", lw=1.2))
    ax.text(6.12, 1.44, "求解器：CasADi + IPOPT（论文用 PHR-ALM）",
            ha="center", va="center", fontsize=9.2, color="white", weight="bold")

    # ---- 输出 / 兜底（同一行，左右分开）----
    ax.add_patch(FancyBboxPatch((0.25, 0.30), 3.4, 0.58,
                                boxstyle="round,pad=0.06", fc="#55efc4", ec="#2d3436", lw=1.2))
    ax.text(1.95, 0.59, "输出: x(t), y(t) → θ(t) 采样轨迹",
            ha="center", va="center", fontsize=8.6)

    ax.add_patch(FancyBboxPatch((8.62, 0.30), 3.45, 0.58,
                                boxstyle="round,pad=0.06", fc="#ff7675", ec="#d63031", lw=1.2))
    ax.text(10.35, 0.59, "验收不过 → 回退 A* 粗路径",
            ha="center", va="center", fontsize=8.6, color="white", weight="bold")

    # ---- 箭头 ----
    for (x1, y1), (x2, y2) in [((3.65, 4.50), (4.05, 4.50)),
                               ((8.20, 4.50), (8.62, 4.50)),
                               ((6.12, 2.05), (6.12, 1.73)),
                               ((6.12, 1.15), (6.12, 0.88)),
                               ((6.12, 0.59), (3.65, 0.59))]:
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                     mutation_scale=14, color="#2d3436", lw=1.5))

    fig.suptitle("图 5  轨迹优化问题的结构：变量 / 约束 / 目标，以及每项的落实程度",
                 fontsize=12.8, y=0.97)
    save(fig, "fig5_optimization.png")


if __name__ == "__main__":
    print("生成原理图 →", OUT)
    fig1_why_se2()
    fig2_algorithm1()
    fig3_flatness()
    fig4_pipeline()
    fig5_optimization()
    print("完成")
