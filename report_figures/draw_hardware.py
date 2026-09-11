"""
图5: 硬件连接拓扑图 — MCU中心 + 外设辐射型
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import os

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(18, 12), dpi=300)
ax.set_xlim(0, 18)
ax.set_ylim(0, 18)
ax.set_aspect('equal')
ax.axis('off')

C_MCU   = '#1a5276'
C_MOTOR = '#b03a2e'
C_SERVO = '#b7950b'
C_ENC   = '#6c3483'
C_COMM  = '#117864'
C_SENS  = '#d35400'
C_LINK  = '#5d6d7e'

fig.patch.set_facecolor('#fafafa')
ax.set_facecolor('#fafafa')

def mcu_pin(ax, x, y, text, color='#2c3e50', fs=6.2):
    """引脚/接口标签"""
    ax.text(x, y, text, ha='center', va='center', fontsize=fs, color=color,
            fontfamily='monospace', fontweight='bold')

def periph_box(ax, x, y, w, h, title, rows, color, pin_side='left'):
    """外设模块框"""
    r = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08",
                       facecolor=color, edgecolor='white', linewidth=0, alpha=0.85)
    ax.add_patch(r)
    ty = y + h - 0.4
    ax.text(x + w/2, ty, title, ha='center', va='top', fontsize=8,
            fontweight='bold', color='white')
    for i, row in enumerate(rows):
        ax.text(x + w/2, ty - 0.55 - i*0.42, row, ha='center', va='top',
                fontsize=5.8, color='white', alpha=0.9)

def link(ax, x1, y1, x2, y2, label="", color=C_LINK, lw=1.3):
    """连接线"""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw))
    if label:
        mx, my = (x1+x2)/2, (y1+y2)/2
        dx, dy = x2-x1, y2-y1
        # 垂直偏移
        if abs(dx) > abs(dy):
            ax.text(mx, my+0.22, label, ha='center', va='bottom', fontsize=5.5,
                    color=color, style='italic')
        else:
            ax.text(mx+0.25, my, label, ha='left', va='center', fontsize=5.5,
                    color=color, style='italic')

# ═══════════════ 中央 MCU ═══════════════
mcu_cx, mcu_cy, mcu_w, mcu_h = 9, 9, 3.5, 4.5
r = FancyBboxPatch((mcu_cx - mcu_w/2, mcu_cy - mcu_h/2), mcu_w, mcu_h,
                   boxstyle="round,pad=0.15", facecolor=C_MCU, edgecolor='#0e3a5c',
                   linewidth=3, alpha=0.95)
ax.add_patch(r)
ax.text(mcu_cx, mcu_cy + 1.6, 'STM32F407ZGTx', ha='center', fontsize=11,
        fontweight='bold', color='white')
ax.text(mcu_cx, mcu_cy + 0.7, 'Cortex-M4  168MHz', ha='center', fontsize=7.5, color='#aed6f1')
ax.text(mcu_cx, mcu_cy - 0.1, 'Flash 1MB  RAM 192KB', ha='center', fontsize=7, color='#aed6f1')
ax.text(mcu_cx, mcu_cy - 0.8, 'LQFP144  SWD PA13/PA14', ha='center', fontsize=7, color='#aed6f1')
# 边界框标注
ax.text(mcu_cx, mcu_cy - 1.8, '(PA15/PB3 由JTAG释放为GPIO)', ha='center', fontsize=6,
        color='#f1948a', style='italic')

# ═══════════════ 电机驱动  (左上) ═══════════════
periph_box(ax, 0.5, 13.0, 4.5, 4.5, '电机驱动  1kHz PWM',
           ['M0 左前 | PWM: PA7 TIM3_CH2',
            '            IN1: PE3  IN2: PE4',
            'M1 右前 | PWM: PA6 TIM3_CH1',
            '            IN1: PE5  IN2: PE6',
            'M2 左后 | PWM: PB1 TIM3_CH4',
            '            IN1: PC8  IN2: PC4',
            'M3 右后 | PWM: PB15 TIM12_CH2 [!]',
            '            IN1: PD3  IN2: PD4',
            'motor_dir: {1, -1, 1, -1}',
            '正速度 = 前进'], C_MOTOR)

link(ax, 5.0, 15.2, 7.25, 12.2, 'TIM3/12')

# ═══════════════ 编码器  (左下) ═══════════════
periph_box(ax, 0.5, 6.5, 4.5, 4.0, '编码器  正交解码',
           ['Enc0 (M0) | TIM5  PA0  PA1',
            'Enc1 (M1) | TIM2  PA15 PB3',
            'Enc2 (M2) | TIM4  PB6  PB7',
            'Enc3 (M3) | TIM8  PC6  PC7 [!]损坏',
            'encoder_dir: {-1,-1,-1,-1}',
            'IIR 低通 + 故障检测',
            '16bit 回绕处理'], C_ENC)

link(ax, 5.0, 8.5, 7.25, 6.8, 'TIM2/4/5/8')

# ═══════════════ 舵机  (右上) ═══════════════
periph_box(ax, 13.0, 13.0, 4.5, 4.5, '舵机  50Hz PWM',
           ['腰  Waist   | PA8 TIM1_CH1  0-180°',
            '大臂 Shoulder | PA9 TIM1_CH2  0-180°',
            '小臂 Elbow   | PA2 TIM9_CH1  0-180°',
            '夹爪 Gripper | PA11 TIM1_CH4 [!]卡死',
            '脉冲: 500-2500us',
            'Quintic 平滑 180°/s 限速',
            '断电管理 (省电)',
            'PA10 残留定义 (已弃用)'], C_SERVO)

link(ax, 10.75, 15.2, 13.0, 12.2, 'TIM1/9')

# ═══════════════ 通信接口  (右下) ═══════════════
periph_box(ax, 13.0, 6.5, 4.5, 4.0, '通信接口',
           ['UART5  调试  | PC12 TX PD2 RX',
            '            115200 主控心跳打印',
            'USART3 OpenMV | PC10 TX PC11 RX',
            '            115200 $TAG/$QR/$CLS',
            'USART2 蓝牙 | PD5 PD6 (已禁用)',
            '            9600 NVIC冲突',
            'Ring Buffer ISR 收 + 主循环发'], C_COMM)

link(ax, 10.75, 8.5, 13.0, 6.8, 'UART5/USART2/3')

# ═══════════════ 传感器 + 保护  (上) ═══════════════
periph_box(ax, 4.5, 13.5, 5.0, 2.0, '传感器 & 系统保护',
           ['巡线传感器: PC0 PC1 PC2 PC3 PA4 (GPIO, 低电平有效)',
            'PVD 电压监测: 2.9V 阈值 最高优先级 0,0',
            'Button: PB1/11 (未使用)  CountSensor: EXTI1 PB1 (空ISR)'], C_SENS)

link(ax, 7.0, 14.5, 7.25, 12.2, 'GPIO/PVD/PB')

# ═══════════════ 中断优先级表 (右下角) ═══════════════
tbl_x, tbl_y = 10.5, 2.0
ax.text(tbl_x, tbl_y + 1.6, 'NVIC 中断优先级 (数值越小越高)', fontsize=7,
        fontweight='bold', color='#2c3e50', ha='center')
table_data = [
    ('PVD',     '0,0', '掉电紧急制动'),
    ('UART5',   '0,0', '调试串口 RX (可抢占 TIM6)'),
    ('TIM6',    '1,0', '100Hz 主控心跳'),
    ('USART3',  '2,0', 'OpenMV RX'),
    ('USART2',  '2,0', '蓝牙 RX (已禁用)'),
    ('EXTI1',   '3,3', '计数器 (空ISR)'),
]
for i, (name, pri, desc) in enumerate(table_data):
    row_y = tbl_y + 1.2 - i*0.38
    ax.text(tbl_x - 1.8, row_y, name, fontsize=6.5, fontweight='bold', color='#2c3e50', fontfamily='monospace')
    ax.text(tbl_x + 0.2, row_y, pri, fontsize=6.5, color='#c0392b', fontfamily='monospace')
    ax.text(tbl_x + 1.5, row_y, desc, fontsize=6, color='#7f8c8d')

# ═══════════════ 标题 ═══════════════
ax.text(mcu_cx, 17.5, '图5  硬件连接拓扑与引脚分配', ha='center', fontsize=13,
        fontweight='bold', color='#2c3e50')

# 图例
leg = [
    mpatches.Patch(color=C_MOTOR, alpha=0.7, label='电机驱动 (TIM3/12)'),
    mpatches.Patch(color=C_ENC,   alpha=0.7, label='编码器 (TIM2/4/5/8)'),
    mpatches.Patch(color=C_SERVO, alpha=0.7, label='舵机 (TIM1/9)'),
    mpatches.Patch(color=C_COMM,  alpha=0.7, label='通信 (UART5/USART2/3)'),
    mpatches.Patch(color=C_SENS,  alpha=0.7, label='传感器 & 保护'),
]
ax.legend(handles=leg, loc='upper right', fontsize=6.5, framealpha=0.9,
          bbox_to_anchor=(0.98, 0.97), ncol=1)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '图5_硬件连接拓扑_matplotlib.png')
plt.tight_layout(pad=1.0)
plt.savefig(out, dpi=300, bbox_inches='tight', facecolor='#fafafa', edgecolor='none')
plt.close()
print('ok: ' + out)
