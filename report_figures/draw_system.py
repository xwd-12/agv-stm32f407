"""
图1: 系统整体框图 — 感知层→决策层→执行层 三明治架构
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import os

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(18, 9), dpi=300)
ax.set_xlim(0, 18)
ax.set_ylim(0, 18)
ax.set_aspect('equal')
ax.axis('off')

C_PER = '#2471a3'  # 感知层 蓝
C_DEC = '#1e8449'  # 决策层 绿
C_ACT = '#b03a2e'  # 执行层 红
C_LINK = '#5d6d7e'

fig.patch.set_facecolor('#fafafa')
ax.set_facecolor('#fafafa')

def box(ax, x, y, w, h, title, lines, color, alpha=0.88):
    """圆角矩形 + 标题 + 多行文字"""
    r = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                       facecolor=color, edgecolor='white', linewidth=0, alpha=alpha)
    ax.add_patch(r)
    # 文字叠加层
    ax.text(x + w/2, y + h/2 + len(lines)*4.5, title, ha='center', va='center',
            fontsize=8, fontweight='bold', color='white')
    for i, line in enumerate(lines):
        ax.text(x + w/2, y + h/2 + (len(lines)/2 - i - 0.5)*5.5, line,
                ha='center', va='center', fontsize=6.2, color='white', alpha=0.92)

def arrow(ax, x1, y1, x2, y2, label="", lw=1.5):
    """中心箭头"""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=C_LINK, lw=lw))
    if label:
        mx, my = (x1+x2)/2, (y1+y2)/2
        ax.text(mx, my+0.2, label, ha='center', va='bottom', fontsize=6,
                color=C_LINK, style='italic')

# ═══════════════ 感知层 ═══════════════
rect_p = mpatches.Rectangle((0.3, 14.2), 17.4, 3.2, facecolor='#d4e6f1',
                             edgecolor=C_PER, lw=2, linestyle='-', alpha=0.3, zorder=0)
ax.add_patch(rect_p)
ax.text(9, 17.2, '感  知  层', fontsize=12, fontweight='bold', color=C_PER, ha='center')

# 感知模块
box(ax, 1.0, 14.7, 4.2, 2.2, 'OpenMV 摄像头',
    ['AprilTag 位姿检测', 'QR 二维码解码', 'AI 分类 (MobileNetV2)',
     'USART3 115200bps'], C_PER)
box(ax, 6.2, 14.7, 4.2, 2.2, '5路巡线传感器',
    ['PC0 PC1 PC2 PC3 PA4', '低电平有效', '加权误差 -2~+2',
     '3帧去抖 + 死区±0.1'], C_PER)
box(ax, 11.8, 14.7, 4.5, 2.2, '编码器 (正交)',
    ['TIM5 PA0/1 · TIM2 PA15/PB3', 'TIM4 PB6/7 · TIM8 PC6/7',
     '(Motor3 编码器损坏)', 'IIR 低通 + 16bit 回绕处理'], C_PER)

# ═══════════════ 决策层 ═══════════════
rect_d = mpatches.Rectangle((0.3, 6.5), 17.4, 7.3, facecolor='#d5f5e3',
                             edgecolor=C_DEC, lw=2, linestyle='-', alpha=0.3, zorder=0)
ax.add_patch(rect_d)
ax.text(9, 13.6, '决  策  层   STM32F407ZGTx  168MHz', fontsize=12, fontweight='bold',
        color=C_DEC, ha='center')

# TIM6 心跳
box(ax, 7.0, 12.0, 4.0, 1.2, 'TIM6 100Hz 主控心跳',
    ['PVD 0,0 | UART5 0,0 | TIM6 1,0 | USART3 2,0 | EXTI1 3,3'], '#1abc9c')

# 控制环路块
box(ax, 1.0,  8.0, 5.2, 3.2, '控制环路',
    ['巡线 PID  10ms | Kp=180 Ki=2 Kd=2',
     '位置 PID  20ms | Kp=0.5 Ki=0.001',
     '速度 PID  10ms | Kp=5.5 Ki=0.16',
     '视觉伺服双 PID  100Hz | 横向+纵向'], C_DEC)

# 状态机块
box(ax, 7.0, 8.0, 5.5, 3.2, '状态机调度 (6个)',
    ['ArmSM: ISR 机械臂锁 + 30s超时',
     'AI Pipeline: main loop Phase 0-3',
     'TaskQueue: N步脚本化任务链',
     'VisionTask: 颜色/QR/AI 50ms',
     'VisualServo: ISR 双PID对接',
     'TaskNav: main loop 遗留 go-grasp-place'], C_DEC)

# OpenMV 协议处理
box(ax, 13.2, 8.0, 4.2, 3.2, '协议处理',
    ['$TAG: AprilTag 位姿数据',
     '$QR: 二维码解码文本',
     '$CLS: AI 分类 (class+conf)',
     '$HB: 心跳存活检测'], C_DEC)

# ═══════════════ 执行层 ═══════════════
rect_a = mpatches.Rectangle((0.3, 0.5), 17.4, 5.6, facecolor='#fadbd8',
                             edgecolor=C_ACT, lw=2, linestyle='-', alpha=0.3, zorder=0)
ax.add_patch(rect_a)
ax.text(9, 5.9, '执  行  层', fontsize=12, fontweight='bold', color=C_ACT, ha='center')

# 电机驱动
box(ax, 1.0, 1.5, 4.8, 3.8, '4路直流电机驱动',
    ['Motor 0 左前 | PA7 TIM3_CH2',
     'Motor 1 右前 | PA6 TIM3_CH1',
     'Motor 2 左后 | PB1 TIM3_CH4',
     'Motor 3 右后 | PB15 TIM12_CH2',
     'PWM 1kHz, 占空比 0-999',
     'motor_dir: {1,-1,1,-1}',
     '正速度 = 前进'], C_ACT)

# 舵机
box(ax, 6.8, 1.5, 4.5, 3.8, '3路舵机 (夹爪禁用)',
    ['腰座 Waist  | PA8 TIM1_CH1  0-180°',
     '大臂 Shoulder | PA9 TIM1_CH2  0-180°',
     '小臂 Elbow  | PA2 TIM9_CH1  0-180°',
     '夹爪 Gripper | PA11 (卡死禁用)',
     'PWM 50Hz, 脉宽 500-2500',
     'Quintic 平滑插值, 180°/s 限速',
     '断电管理: 腰座关/大臂常开/小臂周期'], C_ACT)

# 调试+通信
box(ax, 12.3, 1.5, 5.0, 3.8, '通信与调试',
    ['UART5  调试 | PC12/PD2 115200',
     'USART3 OpenMV | PC10/PC11 115200',
     'USART2 蓝牙 | PD5/PD6 (已禁用)',
     '串口命令: 50+ 命令 (PID/伺服/任务)',
     'CSV 遥测: 50ms 四路电机数据',
     'LLM PID Tuner: Python 自动调参',
     'Ring Buffer ISR + 主循环轮询'], C_ACT)

# ═══════════════ 跨层箭头 ═══════════════
# 感知 → 决策
arrow(ax, 4.0, 14.7, 5.5, 13.5, 'USART3')
arrow(ax, 9.5, 14.7, 9.0, 13.3, 'GPIO')
arrow(ax, 12.5, 14.7, 12.0, 13.5, 'TIM2/4/5/8')

# 决策 → 执行
arrow(ax, 4.0, 8.0, 3.5, 5.7, 'TIM3_CH1/2/4')
arrow(ax, 8.0, 8.0, 7.5, 5.7, 'TIM1/9')
arrow(ax, 15.0, 8.0, 15.0, 5.7, 'UART5/DMA')

# ═══════════════ 标题 ═══════════════
ax.text(9, 17.85, '图1  系统整体框图 — 感知 · 决策 · 执行 三层次架构',
        ha='center', fontsize=13, fontweight='bold', color='#2c3e50')

# 图例
leg = [mpatches.Patch(color=C_PER, alpha=0.6, label='感知层 (传感器数据采集)'),
       mpatches.Patch(color=C_DEC, alpha=0.6, label='决策层 (控制算法 + 状态机)'),
       mpatches.Patch(color=C_ACT, alpha=0.6, label='执行层 (电机 + 舵机 + 通信)')]
ax.legend(handles=leg, loc='lower right', fontsize=7, framealpha=0.9,
          bbox_to_anchor=(0.98, -0.02), ncol=3)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '图1_系统整体框图_matplotlib.png')
plt.subplots_adjust(top=0.96, bottom=0.02, left=0.02, right=0.98)
plt.savefig(out, dpi=300, bbox_inches='tight', facecolor='#fafafa', edgecolor='none')
plt.close()
print('ok: ' + out)
