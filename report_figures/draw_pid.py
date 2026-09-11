"""
图2: 嵌套PID控制链路 — 三环闭环控制架构
matplotlib 绘制，导出 300dpi PNG，可直接插入 Word
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Arc
import numpy as np
import os

# ── 中文字体 ──
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(1, 1, figsize=(20, 10), dpi=300)
ax.set_xlim(0, 20)
ax.set_ylim(0, 10)
ax.set_aspect('equal')
ax.axis('off')

# ═══════════════════════════════════════════════
# 颜色方案
# ═══════════════════════════════════════════════
C_POS    = '#1a5276'  # 位置环 深蓝
C_SPEED  = '#117864'  # 速度环 深绿
C_LINE   = '#935116'  # 巡线环 深棕
C_MOTOR  = '#922b21'  # 执行+反馈 深红
C_ARROW  = '#2c3e50'  # 箭头
C_BG     = '#f8f9fa'  # 背景
C_BOX_BG = '#ffffff'  # 块背景

fig.patch.set_facecolor(C_BG)
ax.set_facecolor(C_BG)

# ═══════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════
def draw_box(x, y, w, h, title, subtitle="", color=C_SPEED, fontsize_title=9, fontsize_sub=7,
             edge_lw=2.0, alpha=0.15):
    """画圆角矩形块"""
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                         boxstyle="round,pad=0.12", facecolor=color, edgecolor=color,
                         linewidth=edge_lw, alpha=alpha, zorder=2)
    ax.add_patch(box)
    ax.text(x, y + 0.08, title, ha='center', va='center', fontsize=fontsize_title,
            fontweight='bold', color=color, zorder=3)
    if subtitle:
        ax.text(x, y - 0.22, subtitle, ha='center', va='center', fontsize=fontsize_sub,
                color='#555555', zorder=3)

def draw_clamp(x, y, text, color='#7f8c8d'):
    """画钳位/限幅标志"""
    ax.plot([x-0.2, x+0.2], [y+0.3, y-0.3], color=color, lw=2.5, zorder=3)
    ax.plot([x+0.2, x-0.2], [y+0.3, y-0.3], color=color, lw=2.5, zorder=3)
    ax.text(x, y - 0.55, text, ha='center', va='center', fontsize=7, color=color, fontweight='bold')

def draw_sum(x, y, color='#2c3e50'):
    """画求和节点 ⊕"""
    c = plt.Circle((x, y), 0.2, facecolor='white', edgecolor=color, lw=2.2, zorder=3)
    ax.add_patch(c)
    ax.text(x, y, '+', ha='center', va='center', fontsize=11, fontweight='bold', color=color, zorder=4)

def arrow(x1, y1, x2, y2, color=C_ARROW, lw=1.8, style='simple', zorder=1,
          head_width=0.18, head_length=0.25, label="", label_offset=(0, 0.15)):
    """画箭头"""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                               connectionstyle="arc3,rad=0"),
                zorder=zorder)
    if label:
        mx, my = (x1 + x2) / 2 + label_offset[0], (y1 + y2) / 2 + label_offset[1]
        ax.text(mx, my, label, ha='center', va='center', fontsize=6.5, color=color,
                style='italic')

def bracket_label(x, y, w, text, color, fontsize=10):
    """区域标签（大括号式）"""
    ax.text(x + w/2, y, text, ha='center', va='center', fontsize=fontsize,
            fontweight='bold', color=color, alpha=0.5)

# ═══════════════════════════════════════════════
# 区域背景
# ═══════════════════════════════════════════════
# 巡线环区域
rect_lf = mpatches.Rectangle((0.2, 7.2), 19.6, 2.0, facecolor='#fdebd0', edgecolor='#e67e22',
                               linewidth=1.5, linestyle='--', alpha=0.5, zorder=0)
ax.add_patch(rect_lf)
ax.text(0.6, 9.0, '巡线环  (仅 Line-Follow 模式, 10ms)', fontsize=10, fontweight='bold',
        color='#935116', zorder=1)

# 位置环区域
rect_pos = mpatches.Rectangle((0.2, 4.8), 19.6, 2.2, facecolor='#d6eaf8', edgecolor='#2980b9',
                                linewidth=1.5, linestyle='--', alpha=0.5, zorder=0)
ax.add_patch(rect_pos)
ax.text(0.6, 6.8, '位置环  (20ms)', fontsize=10, fontweight='bold', color='#1a5276', zorder=1)

# 速度环区域
rect_vel = mpatches.Rectangle((0.2, 0.5), 19.6, 4.1, facecolor='#d5f5e3', edgecolor='#27ae60',
                                linewidth=1.5, linestyle='--', alpha=0.5, zorder=0)
ax.add_patch(rect_vel)
ax.text(0.6, 4.4, '速度环  (10ms)', fontsize=10, fontweight='bold', color='#117864', zorder=1)

# 执行反馈区域
rect_mtr = mpatches.Rectangle((13.2, 0.5), 6.6, 9.0, facecolor='#fadbd8', edgecolor='#e74c3c',
                                linewidth=1.5, linestyle='--', alpha=0.4, zorder=0)
ax.add_patch(rect_mtr)
ax.text(16.5, 9.3, '执行 + 反馈', fontsize=10, fontweight='bold', color='#922b21', zorder=1)

# ═══════════════════════════════════════════════
# 巡线环 (上方)
# ═══════════════════════════════════════════════
draw_box(2.5, 8.3, 2.8, 1.2, '5路巡线传感器', 'PC0-PC3, PA4\n加权误差 ±2', C_LINE, fontsize_title=8.5, fontsize_sub=6.5)
draw_box(6.5, 8.3, 2.6, 1.2, '巡线 PID', 'Kp=180 Ki=2.0 Kd=2.0\n死区±0.1 积分分离', C_LINE, fontsize_title=8.5, fontsize_sub=6.5)
draw_clamp(9.5, 8.3, '±600\n钳位', C_LINE)

# 传感器 → PID
arrow(3.9, 8.3, 5.2, 8.3, C_LINE, label='传感器状态')

# PID → 钳位
arrow(7.8, 8.3, 8.7, 8.3, C_LINE)

# 钳位 → 求和节点 (向下注入)
arrow(9.5, 7.8, 9.5, 4.8, C_LINE, label='turn\nleft+=turn\nright−=turn')

# ═══════════════════════════════════════════════
# 位置环 (中间)
# ═══════════════════════════════════════════════
draw_box(2.5, 5.8, 2.4, 1.1, '设定位置', 'encoder counts\n目标距离', C_POS, fontsize_title=8.5, fontsize_sub=6.5)
draw_box(5.5, 5.8, 2.4, 1.1, '位置式 PID', 'Kp=0.5 Ki=0.001\nKd=0', C_POS, fontsize_title=8.5, fontsize_sub=6.5)
draw_clamp(8.0, 5.8, '±600\n钳位', '#7f8c8d')

# 设定 → PID
arrow(3.7, 5.8, 4.3, 5.8, C_POS, label='pos_set')
# PID → 钳位
arrow(6.7, 5.8, 7.4, 5.8, C_POS)

# 钳位 → 求和节点 (向下)
arrow(8.0, 5.3, 8.0, 4.5, C_POS)

# ═══════════════════════════════════════════════
# 速度环 (下方) — 核心通路
# ═══════════════════════════════════════════════

# 求和节点
draw_sum(9.5, 3.5)

# 速度 PID
draw_box(11.5, 3.5, 2.6, 1.2, '增量式 PID', 'Motor0,1: Kp=5.5 Ki=0.16\nMotor2,3: Kp=5.0 Ki=0.14', C_SPEED, fontsize_title=8.5, fontsize_sub=6.5)

# motor_dir 校正
draw_box(14.2, 3.5, 2.2, 1.0, 'motor_dir 校正', '{1, −1, 1, −1}\n正速度=前进', '#7f8c8d', fontsize_title=8, fontsize_sub=6.5)

# PWM 输出
draw_box(16.5, 3.5, 1.8, 1.0, 'PWM 输出', '0–999\n1kHz', C_MOTOR, fontsize_title=8, fontsize_sub=6.5)

# 电机
draw_box(18.5, 3.5, 1.8, 1.0, '4路直流电机', '左前 左后\n右前 右后', C_MOTOR, fontsize_title=8, fontsize_sub=6.5)

# 位置PID钳位 → 求和
arrow(8.0, 5.3, 9.3, 3.7, C_POS)

# 巡线钳位 → 求和
arrow(9.5, 7.2, 9.5, 3.7, C_LINE)

# 求和 → 速度PID
arrow(9.7, 3.5, 10.2, 3.5, C_SPEED, label='速度指令')

# 速度PID → dir
arrow(12.8, 3.5, 13.1, 3.5, C_SPEED, label='±1000')

# dir → PWM
arrow(15.3, 3.5, 15.6, 3.5, '#7f8c8d')

# PWM → 电机
arrow(17.4, 3.5, 17.6, 3.5, C_MOTOR, label='0–999')

# ═══════════════════════════════════════════════
# 编码器反馈
# ═══════════════════════════════════════════════
draw_box(18.5, 2.0, 1.8, 0.9, '编码器×4', '(Motor3 损坏)', C_MOTOR, fontsize_title=8, fontsize_sub=6)

# 电机 → 编码器 (向下)
arrow(18.5, 3.0, 18.5, 2.45, C_MOTOR, lw=1.5)

# ═══════════════════════════════════════════════
# 速度反馈回路 (编码器 → 速度PID)
# ═══════════════════════════════════════════════
# 从编码器绕回到速度PID的负输入端
# 绘制: 编码器左端(17.6, 2.0) → 向下绕 → 向左 → 向上 → 到求和节点(9.5, 3.3)底部

ax.annotate('', xy=(9.5, 3.3), xytext=(17.6, 2.0),
            arrowprops=dict(arrowstyle='simple', color=C_SPEED, lw=1.8,
                           connectionstyle="arc3,rad=0.35"),
            zorder=1)
# 速度反馈标签
ax.text(13.5, 1.1, '速度反馈 (−)', ha='center', va='center', fontsize=7.5,
        color=C_SPEED, fontweight='bold', style='italic')

# ═══════════════════════════════════════════════
# 位置反馈回路 (编码器 → 位置PID)
# ═══════════════════════════════════════════════
# 从编码器积分得到位置 → 回馈到位置PID
# 绘制回路: 编码器左端(17.6, 2.0) → 向上绕 → 左 → 向下到位置PID底部

ax.annotate('', xy=(5.5, 5.25), xytext=(17.6, 2.5),
            arrowprops=dict(arrowstyle='simple', color=C_POS, lw=1.8,
                           connectionstyle="arc3,rad=-0.30"),
            zorder=1)
ax.text(13.0, 7.0, '位置反馈 (−) 积分累加', ha='center', va='center', fontsize=7.5,
        color=C_POS, fontweight='bold', style='italic')

# ═══════════════════════════════════════════════
# 图例
# ═══════════════════════════════════════════════
legend_items = [
    mpatches.Patch(color=C_SPEED, alpha=0.3, label='速度环 (增量式 PID 10ms)'),
    mpatches.Patch(color=C_POS, alpha=0.3, label='位置环 (位置式 PID 20ms)'),
    mpatches.Patch(color=C_LINE, alpha=0.3, label='巡线环 (位置式 PID 10ms, 仅巡线模式)'),
    mpatches.Patch(color=C_MOTOR, alpha=0.3, label='执行 + 传感反馈'),
]
ax.legend(handles=legend_items, loc='lower left', fontsize=8, framealpha=0.9,
          ncol=2, bbox_to_anchor=(0.02, -0.05))

# ═══════════════════════════════════════════════
# 底部注释
# ═══════════════════════════════════════════════
note = ('注: ① 位置环输出经 ±600 钳位后作为速度指令; '
        '② 巡线环计算左右轮差速转向 (left+turn, right−turn), 输出钳位 [0, 1000] (仅前进); '
        '③ Motor 3 编码器损坏 → 开环补偿 (right−20)')
ax.text(10, -0.3, note, ha='center', va='center', fontsize=7, color='#7f8c8d', style='italic')

# ═══════════════════════════════════════════════
# 标题
# ═══════════════════════════════════════════════
ax.text(10, 9.85, '图2  嵌套 PID 三环控制链路', ha='center', va='center', fontsize=14,
        fontweight='bold', color='#2c3e50')

# 保存
out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    '图2_嵌套PID控制链路_matplotlib.png')
plt.tight_layout(pad=1.5)
plt.savefig(out, dpi=300, bbox_inches='tight', facecolor=C_BG, edgecolor='none')
plt.close()
print('ok: ' + out)
