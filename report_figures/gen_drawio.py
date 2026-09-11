"""
生成 5 张 .drawio 文件，用 draw.io 桌面版打开可直接编辑、导出 PNG。

每张图手动布局：精确控制方块位置/颜色/箭头走向。
"""
import xml.etree.ElementTree as ET
import os, html

DIR = os.path.dirname(os.path.abspath(__file__))

# ═══════════════════════════════════════
# drawio XML 工具
# ═══════════════════════════════════════

def new_diagram(name, w=1200, h=900):
    """创建空白图表"""
    mxfile = ET.Element('mxfile', host='app.diagrams.net', modified='2026-06-27',
                        agent='Python', version='24.0.0', type='device')
    diagram = ET.SubElement(mxfile, 'diagram', id='diag', name=name)
    model = ET.SubElement(diagram, 'mxGraphModel', dx='0', dy='0', grid='1',
                          gridSize='10', guides='1', tooltips='1', connect='1',
                          arrows='1', fold='1', page='1', pageScale='1',
                          pageWidth=str(w), pageHeight=str(h),
                          math='0', shadow='0')
    root = ET.SubElement(model, 'root')
    ET.SubElement(root, 'mxCell', id='0')
    ET.SubElement(root, 'mxCell', id='1', parent='0')
    return mxfile, root

def add_cell(root, cell_id, parent='1', value='', style='', vertex='0', edge='0',
             x=None, y=None, w=None, h=None, source=None, target=None,
             points=None):
    """添加一个 mxCell"""
    attrs = {'id': str(cell_id), 'parent': str(parent), 'style': style}
    if vertex == '1':
        attrs['vertex'] = '1'
    if edge == '1':
        attrs['edge'] = '1'
    cell = ET.SubElement(root, 'mxCell', **attrs)
    if value:
        cell.set('value', value)
    if source is not None:
        cell.set('source', str(source))
    if target is not None:
        cell.set('target', str(target))
    geo = None
    geo = None
    if x is not None or y is not None or w is not None or h is not None:
        geo = ET.SubElement(cell, 'mxGeometry', x=str(x or 0), y=str(y or 0),
                            width=str(w or 0), height=str(h or 0))
        geo.set('as', 'geometry')
    if points:
        arr = ET.SubElement(cell, 'mxGeometry', relative='1')
        arr.set('as', 'geometry')
        pts = ET.SubElement(arr, 'Array')
        pts.set('as', 'points')
        for px, py in points:
            pt = ET.SubElement(pts, 'mxPoint', x=str(px), y=str(py))
        if not geo:
            geo = ET.SubElement(cell, 'mxGeometry', width='50', height='50', relative='1')
            geo.set('as', 'geometry')
    return cell

# ═══════════════════════════════
# 样式常量
# ═══════════════════════════════

# 圆角方块基础样式
BOX_BASE = ('rounded=1;whiteSpace=wrap;html=1;arcSize=12;'
            'fontFamily=Microsoft YaHei;fontSize=12;align=center;verticalAlign=middle;')

# 颜色方案
STYLES = {
    'perception': BOX_BASE + 'fillColor=#D6EAF8;strokeColor=#2471A3;fontColor=#1A5276;fontStyle=1;',
    'decision':   BOX_BASE + 'fillColor=#D5F5E3;strokeColor=#1E8449;fontColor=#145A32;fontStyle=1;',
    'actuation':  BOX_BASE + 'fillColor=#FADBD8;strokeColor=#B03A2E;fontColor=#7B241C;fontStyle=1;',
    'line':       BOX_BASE + 'fillColor=#FDEBD0;strokeColor=#B9770E;fontColor=#7D6608;fontStyle=1;',
    'speed':      BOX_BASE + 'fillColor=#D5F5E3;strokeColor=#117864;fontColor=#0E6251;fontStyle=1;',
    'position':   BOX_BASE + 'fillColor=#D6EAF8;strokeColor=#1A5276;fontColor=#154360;fontStyle=1;',
    'motor':      BOX_BASE + 'fillColor=#F2D7D5;strokeColor=#922B21;fontColor=#641E16;fontStyle=1;',
    'servo':      BOX_BASE + 'fillColor=#FCF3CF;strokeColor=#B7950B;fontColor=#7D6608;fontStyle=1;',
    'comm':       BOX_BASE + 'fillColor=#D1F2EB;strokeColor=#0E6655;fontColor=#0B5345;fontStyle=1;',
    'encoder':    BOX_BASE + 'fillColor=#E8DAEF;strokeColor=#6C3483;fontColor=#4A235A;fontStyle=1;',
    'sum':        'ellipse;whiteSpace=wrap;html=1;aspect=fixed;fillColor=#FFFFFF;strokeColor=#2C3E50;strokeWidth=2;fontSize=16;fontStyle=1;',
    'clamp':      BOX_BASE + 'fillColor=#EAECEE;strokeColor=#7F8C8D;fontColor=#424949;fontSize=9;strokeWidth=2;dashed=1;',
    'arrow':      ('edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;'
                   'html=1;strokeColor=#5D6D7E;strokeWidth=2;fontFamily=Microsoft YaHei;fontSize=10;fontColor=#5D6D7E;'),
    'arrow_fb':   ('edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;'
                   'html=1;strokeColor=#16A085;strokeWidth=2;dashed=1;fontFamily=Microsoft YaHei;fontSize=10;fontColor=#117864;'),
    'label':      BOX_BASE + 'fillColor=none;strokeColor=none;fontColor=#2C3E50;fontSize=14;fontStyle=1;',
    'zone':       BOX_BASE + 'fillColor=#F8F9FA;strokeColor=#BDC3C7;strokeWidth=1;fontSize=11;fontStyle=1;arcSize=5;dashed=1;opacity=50;',
}

def edge_style(color='#5D6D7E', dashed='0'):
    return (f'edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;'
            f'html=1;strokeColor={color};strokeWidth=2;fontFamily=Microsoft YaHei;'
            f'fontSize=10;fontColor={color};dashed={dashed};exitX=1;exitY=0.5;exitDx=0;exitDy=0;'
            f'entryX=0;entryY=0.5;entryDx=0;entryDy=0;')

counter = [2]  # 从2开始, 0和1已用
def next_id():
    counter[0] += 1
    return counter[0]

def box(root, x, y, w, h, title, lines, style_key='decision', font_size=12):
    """多行文字方块"""
    value = f'<b>{html.escape(title)}</b>'
    for ln in lines:
        value += f'<br><font style="font-size:10px">{html.escape(ln)}</font>'
    sid = next_id()
    s = STYLES[style_key] + f'fontSize={font_size};'
    add_cell(root, sid, value=value, style=s, vertex='1', x=x, y=y, w=w, h=h)
    return sid

def arrow(root, src, tgt, label='', style_key='arrow'):
    eid = next_id()
    add_cell(root, eid, value=label, style=STYLES[style_key], edge='1', source=src, target=tgt)
    return eid

def clamp_box(root, x, y, text):
    """钳位方块"""
    sid = next_id()
    add_cell(root, sid, value=text, style=STYLES['clamp'], vertex='1', x=x, y=y, w=60, h=40)
    return sid

def sum_node(root, x, y):
    sid = next_id()
    add_cell(root, sid, value='<b>+</b>', style=STYLES['sum'], vertex='1', x=x, y=y, w=40, h=40)
    return sid

def zone_rect(root, x, y, w, h, title, color='#E8E8E8'):
    sid = next_id()
    s = STYLES['zone'].replace('#F8F9FA', color)
    add_cell(root, sid, value=f'<b>{html.escape(title)}</b>', style=s, vertex='1',
             x=x, y=y, w=w, h=h)
    return sid

def save(mxfile, name):
    path = os.path.join(DIR, name)
    tree = ET.ElementTree(mxfile)
    tree.write(path, encoding='utf-8', xml_declaration=True)
    print(f'  -> {name}')

# ═══════════════════════════════════════════════
# 图2: 嵌套PID控制链路 (核心)
# ═══════════════════════════════════════════════
def make_pid():
    mxfile, root = new_diagram('嵌套PID控制链路', 1300, 700)

    # --- 区域背景 ---
    zone_rect(root, 20, 450, 1260, 210, '巡线环 (仅 Line-Follow, 10ms)', '#FEF9E7')
    zone_rect(root, 20, 260, 1260, 180, '位置环 (20ms)', '#EBF5FB')
    zone_rect(root, 20, 20,  1260, 230, '速度环 (10ms)', '#E8F8F5')
    zone_rect(root, 1050, 20, 230, 650, '执行 + 反馈', '#FDEDEC')

    # --- 巡线环 ---
    ls    = box(root, 80,  485, 180, 80, '5路巡线传感器', ['PC0-PC3, PA4', '加权误差 ±2'], 'line')
    lfpid = box(root, 360, 485, 170, 80, '巡线 PID', ['Kp=180 Ki=2.0 Kd=2.0', '死区±0.1 积分分离'], 'line')
    lcl   = clamp_box(root, 570, 505, '转向\n±600')

    # --- 位置环 ---
    pset  = box(root, 80,  300, 140, 80, '设定位置', ['encoder counts'], 'position')
    pspid = box(root, 310, 300, 150, 80, '位置式 PID', ['Kp=0.5 Ki=0.001', 'Kd=0'], 'position')
    pcl   = clamp_box(root, 500, 310, '±600')

    # --- 速度环 + 执行 ---
    ssum  = sum_node(root, 620, 90)
    vspid = box(root, 730, 70, 180, 80, '增量式 PID', ['Motor0/1: Kp=5.5 Ki=0.16', 'Motor2/3: Kp=5.0 Ki=0.14'], 'speed')
    mdir  = box(root, 960, 75, 130, 70, 'motor_dir', ['{1,-1,1,-1}'], 'decision')
    pwm   = box(root, 1120, 75, 130, 70, 'PWM 输出', ['0-999  1kHz', 'TIM1/3/9/12'], 'motor')
    mtr   = box(root, 1120, 180, 130, 70, '4路直流电机', ['左前/右前', '左后/右后'], 'motor')
    enc   = box(root, 1120, 300, 130, 70, '编码器×4', ['TIM2/4/5/8', 'Motor3损坏'], 'encoder')

    # --- 箭头: 巡线 ---
    arrow(root, ls, lfpid)
    arrow(root, lfpid, lcl)
    arrow(root, lcl, ssum, 'turn')

    # --- 箭头: 位置 ---
    arrow(root, pset, pspid)
    arrow(root, pspid, pcl)
    arrow(root, pcl, ssum, 'pos指令')

    # --- 箭头: 速度 ---
    arrow(root, ssum, vspid)
    arrow(root, vspid, mdir)
    arrow(root, mdir, pwm)
    arrow(root, pwm, mtr)
    arrow(root, mtr, enc)

    # --- 反馈回路 ---
    # 速度反馈: 编码器→速度PID
    e_fb_v = next_id()
    add_cell(root, e_fb_v, value='速度反馈 (−)', style=edge_style('#16A085', '1'),
             edge='1', source=enc, target=vspid)
    # 位置反馈: 编码器→位置PID
    e_fb_p = next_id()
    add_cell(root, e_fb_p, value='位置反馈 (−) 积分累加', style=edge_style('#2980B9', '1'),
             edge='1', source=enc, target=pspid)

    # 标注
    title_id = next_id()
    add_cell(root, title_id,
             value='<b><font style="font-size:16px">图2  嵌套 PID 三环控制链路</font></b>',
             style=STYLES['label'], vertex='1', x=350, y=650, w=600, h=40)

    save(mxfile, '图2_嵌套PID控制链路.drawio')


# ═══════════════════════════════════════════════
# 图1: 系统整体框图 (三层架构)
# ═══════════════════════════════════════════════
def make_system():
    mxfile, root = new_diagram('系统整体框图', 1200, 900)

    # 感知层背景
    zone_rect(root, 20, 680, 1160, 190, '感  知  层', '#EBF5FB')
    # 决策层背景
    zone_rect(root, 20, 300, 1160, 370, '决  策  层  —  STM32F407ZGTx  168MHz', '#E8F8F5')
    # 执行层背景
    zone_rect(root, 20, 20,  1160, 270, '执  行  层', '#FDEDEC')

    # --- 感知层 ---
    box(root, 50,  720, 330, 130, 'OpenMV 摄像头',
        ['AprilTag 位姿检测 | QR 二维码解码', 'AI 分类 MobileNetV2 α=0.35 int8量化',
         'USART3  115200bps'], 'perception')
    box(root, 440, 720, 300, 130, '5路巡线传感器',
        ['PC0 PC1 PC2 PC3 PA4 | 低电平有效',
         '加权误差 -2 ~ +2 | 3帧去抖+死区'], 'perception')
    box(root, 800, 720, 350, 130, '编码器 (正交解码)',
        ['TIM5 PA0/1 | TIM2 PA15/PB3',
         'TIM4 PB6/7 | TIM8 PC6/7 (Motor3损坏)',
         'IIR低通 + 16bit回绕处理'], 'perception')

    # --- 决策层 ---
    box(root, 370, 580, 460, 60, 'TIM6 100Hz 主控心跳',
        ['PVD 0,0 | UART5 0,0 | TIM6 1,0 | USART3 2,0 | EXTI1 3,3'], 'decision', 11)

    box(root, 50,  340, 340, 210, '控制环路',
        ['巡线 PID  10ms | Kp=180 Ki=2 Kd=2',
         '位置 PID  20ms | Kp=0.5 Ki=0.001',
         '速度 PID  10ms | Kp=5.5 Ki=0.16 (增量式)',
         '视觉伺服双 PID  100Hz | 横向P + 纵向PID',
         '钳位: 位置±600 | 速度±1000 | 巡线[0,1000]',
         'motor_dir: {1,-1,1,-1} 正速度=前进'], 'decision')

    box(root, 430, 340, 360, 210, '状态机调度 (6个)',
        ['ArmSM: ISR内 机械臂互斥锁 + 30s超时',
         'AI Pipeline: main loop Phase 0→1→2→3',
         'TaskQueue: N步脚本化任务链 (≤16步)',
         'VisionTask: 颜色/QR/AI 单任务 50ms',
         'VisualServo: ISR内 双PID Tag对接',
         'TaskNav: main loop 遗留 go-grasp-place'], 'decision')

    box(root, 830, 340, 320, 210, '协议处理 & 调试',
        ['$TAG: AprilTag id/cx/cy/dist/angle/width',
         '$QR:  二维码解码文本 (≤64字符)',
         '$CLS: AI分类 class_id + confidence',
         '$HB:  心跳存活检测 (1Hz)',
         '50+ 串口命令 (PID/伺服/任务/模式)',
         'CSV遥测: 50ms四路电机数据到LLM Tuner'], 'decision')

    # --- 执行层 ---
    box(root, 50,  60, 340, 170, '4路直流电机驱动',
        ['M0左前 PA7 TIM3 | M1右前 PA6 TIM3',
         'M2左后 PB1 TIM3 | M3右后 PB15 TIM12 *',
         '* Motor3编码器损坏 → 开环 + 右后-20补偿',
         'PWM 1kHz  占空比 0-999  motor_dir:{1,-1,1,-1}'], 'motor')

    box(root, 430, 60, 360, 170, '3路舵机 + 夹爪(禁用)',
        ['腰座 WAIST   PA8 TIM1 | 大臂 SHOULDER PA9 TIM1',
         '小臂 ELBOW    PA2 TIM9 | 夹爪 GRIPPER PA11 (卡死)',
         'PWM 50Hz 脉宽500-2500us | Quintic平滑插值',
         '断电管理: 腰座关 | 大臂常开 | 小臂周期 | 夹爪关'], 'servo')

    box(root, 830, 60, 320, 170, '通信与调试',
        ['UART5  调试  PC12/PD2  115200 (Ring Buffer)',
         'USART3 OpenMV PC10/PC11 115200',
         'USART2 蓝牙  PD5/PD6  9600 (已禁用)',
         'LLM PID Tuner: Python上位机自动调参',
         'SWD: PA13/PA14 (JTAG释放PA15/PB3为GPIO)'], 'comm')

    # 箭头
    arrow(root, 7, 11)    # OpenMV → TIM6
    arrow(root, 8, 12)    # 巡线 → 控制环路
    arrow(root, 9, 11)    # 编码器 → 决策层

    arrow(root, 12, 17)   # 控制环路 → 电机
    arrow(root, 12, 18)   # 控制环路 → 舵机
    arrow(root, 13, 19)   # 状态机 → 通信

    title_id = next_id()
    add_cell(root, title_id,
             value='<b><font style="font-size:16px">图1  系统整体框图 — 感知 · 决策 · 执行 三层次架构</font></b>',
             style=STYLES['label'], vertex='1', x=200, y=0, w=800, h=30)

    save(mxfile, '图1_系统整体框图.drawio')


# ═══════════════════════════════════════════════
# 图3: AI Pipeline 流程图
# ═══════════════════════════════════════════════
def make_ai_pipeline():
    mxfile, root = new_diagram('AI自动扫描Pipeline', 900, 1100)

    nodes = [
        ('start',   400, 1040, 100, 40, '<b>启动/巡线</b>', 'line'),
        ('cooling', 400, 920,  100, 80, '<b>Phase 0</b><br>冷却等待<br>初始5s+间隔15s', 'perception'),
        ('scan',    400, 780,  100, 100,'<b>Phase 1</b><br>AI扫描<br>停车→腰座±60°<br>筛选class=0 conf≥50%<br>超时8s→回巡线', 'decision'),
        ('align',   400, 600,  100, 120,'<b>Phase 2</b><br>对准<br>子0:腰座比例转向<br>0.04°/px ±5°<br>子1:车身±50前后<br>cx≤15px dist≤8cm<br>10s超时→强制执行', 'position'),
        ('exec',    400, 430,  100, 80, '<b>Phase 3</b><br>执行抓取<br>保存→Action_Grasp<br>→还原腰座角度', 'motor'),
        ('back',    400, 310,  100, 40, '<b>切QRCODE→巡线</b>', 'line'),
    ]

    for nid, x, y, w, h, val, sk in nodes:
        sid = next_id()
        add_cell(root, sid, value=val, style=STYLES[sk] + 'fontSize=11;',
                 vertex='1', x=x, y=y, w=w, h=h)
        # We'll store the IDs in order
        nodes[nodes.index((nid, x, y, w, h, val, sk))] = (sid, x, y, w, h, val, sk)

    # Connect
    for i in range(len(nodes) - 1):
        s1, _, _, _, _, _, _ = nodes[i]
        s2, _, _, _, _, _, _ = nodes[i+1]
        arrow(root, s1, s2)

    # Timeout branch: Phase1 scan -> cooling (8s timeout)
    # We'll add a side note
    note_id = next_id()
    add_cell(root, note_id,
             value='<b>⚠ 关键Bug</b><br>Bug#1: Phase2超时用独立定时器<br>Bug#2: 跨任务scan状态需手动重置<br>Bug#4: Phase1腰座±20°扩大扫描<br>Bug#6: 模式切换后50ms延迟防stale',
             style=STYLES['clamp'] + 'fontSize=10;', vertex='1', x=560, y=500, w=250, h=140)

    title_id = next_id()
    add_cell(root, title_id,
             value='<b><font style="font-size:16px">图3  AI 自动扫描 Pipeline 状态机</font></b>',
             style=STYLES['label'], vertex='1', x=250, y=0, w=400, h=30)

    save(mxfile, '图3_AI自动扫描Pipeline.drawio')


# ═══════════════════════════════════════════════
# 图4: 状态机调度总览
# ═══════════════════════════════════════════════
def make_statemachine():
    mxfile, root = new_diagram('状态机调度总览', 1100, 600)

    zone_rect(root, 20, 320, 1060, 250, 'TIM6 ISR (100Hz 硬实时)', '#E8F8F5')
    zone_rect(root, 20, 50,  1060, 250, 'main() 主循环 (软实时)', '#EBF5FB')
    zone_rect(root, 830, 50, 250, 520, '触发源', '#FEF9E7')

    # ISR
    box(root, 50,  370, 220, 160, 'ArmSM',
        ['机械臂互斥锁', '30s超时保护', '50ms去抖'], 'position')
    box(root, 310, 370, 220, 160, 'VisualServo',
        ['双PID Tag对接', '横向P + 纵向PID', '200ms丢标停车'], 'perception')
    box(root, 570, 370, 220, 160, '巡线PID',
        ['传感器→转向→速度', 'line_time定时停车', '曲线计数检测'], 'line')

    # main loop
    box(root, 50,  90, 220, 190, 'AI Pipeline',
        ['Phase 0→1→2→3', '自动周期触发', 'QR跳过/15s冷却', 'class_id=0筛选'], 'decision')
    box(root, 310, 90, 220, 190, 'TaskQueue',
        ['≤16步脚本化任务', '巡线→对接→动作', '30s/10s/8s超时', 'AprilTag触发'], 'decision')
    box(root, 570, 90, 220, 190, 'VisionTask + TaskNav',
        ['颜色/QR/AI搜索', '50ms节流', 'TaskNav:遗留go-grasp', '1s丢标超时'], 'decision')

    # 触发源
    box(root, 860, 340, 200, 70, 'UART5 串口', ['50+命令'], 'comm')
    box(root, 860, 430, 200, 70, '十字路口检测', ['5路全踩触发'], 'line')
    box(root, 860, 520, 200, 70, '15s周期定时', ['自动冷却触发'], 'perception')

    # 箭头
    arrow(root, 17, 11)  # 串口→ArmSM
    arrow(root, 17, 13)  # 串口→TaskQueue
    arrow(root, 17, 14)  # 串口→VisionTask
    arrow(root, 18, 13)  # 十字→TaskQueue
    arrow(root, 19, 12)  # 定时→AI Pipeline
    arrow(root, 13, 10)  # TaskQueue→VisualServo
    arrow(root, 13, 11)  # TaskQueue→ArmSM
    arrow(root, 12, 11)  # AI→ArmSM
    arrow(root, 14, 11)  # VisionTask→ArmSM

    title_id = next_id()
    add_cell(root, title_id,
             value='<b><font style="font-size:16px">图4  状态机调度总览 (6个状态机 × 2个运行上下文)</font></b>',
             style=STYLES['label'], vertex='1', x=250, y=0, w=600, h=30)

    save(mxfile, '图4_状态机调度总览.drawio')


# ═══════════════════════════════════════════════
# 图5: 硬件连接拓扑
# ═══════════════════════════════════════════════
def make_hardware():
    mxfile, root = new_diagram('硬件连接拓扑', 1200, 900)

    # 中央MCU
    mcu = next_id()
    add_cell(root, mcu,
             value='<b><font style="font-size:14px">STM32F407ZGTx</font></b><br>'
                   '<font style="font-size:10px">Cortex-M4  168MHz<br>'
                   'Flash 1MB  RAM 192KB<br>'
                   'LQFP-144  SWD: PA13/PA14<br>'
                   'JTAG释放 PA15/PB3 → GPIO</font>',
             style=STYLES['decision'] + 'fontSize=12;', vertex='1',
             x=430, y=380, w=240, h=120)

    # 电机驱动 (左上)
    box(root, 30,  620, 320, 250, '电机驱动  1kHz PWM',
        ['M0 左前 | PA7  TIM3_CH2 | IN: PE3/PE4',
         'M1 右前 | PA6  TIM3_CH1 | IN: PE5/PE6',
         'M2 左后 | PB1  TIM3_CH4 | IN: PC8/PC4',
         'M3 右后 | PB15 TIM12_CH2 | IN: PD3/PD4 [!]',
         '[!] 编码器损坏 → 开环 fallback',
         'motor_dir: {1, -1, 1, -1} → 正=前进',
         '速度钳位 ±1000 | 占空比 0-999'], 'motor')

    # 编码器 (左下)
    box(root, 30,  50, 320, 210, '编码器  正交解码',
        ['Enc0 (M0左前) | TIM5  PA0   PA1',
         'Enc1 (M1右前) | TIM2  PA15  PB3',
         'Enc2 (M2左后) | TIM4  PB6   PB7',
         'Enc3 (M3右后) | TIM8  PC6   PC7  [!]损坏',
         'encoder_dir: {-1,-1,-1,-1}',
         'IIR低通(70%+30%) | 16bit回绕 | 故障检测'], 'encoder')

    # 舵机 (右上)
    box(root, 750, 620, 350, 250, '舵机  50Hz PWM',
        ['0 腰座 Waist   | PA8  TIM1_CH1  0-180°',
         '1 大臂 Shoulder| PA9  TIM1_CH2  0-180°',
         '2 小臂 Elbow   | PA2  TIM9_CH1  0-180°',
         '3 夹爪 Gripper | PA11 TIM1_CH4  [!]卡死禁用',
         'PWM 脉宽 500-2500us | ARR=19999 PSC=167',
         'Quintic平滑插值 | 180°/s限速',
         '断电: 腰关 肩常开 肘周期 爪关'], 'servo')

    # 通信 (右下)
    box(root, 750, 50, 350, 230, '通信接口',
        ['UART5  调试  | PC12 TX  PD2 RX  115200',
         '  Ring Buffer ISR收 + 主循环轮询',
         'USART3 OpenMV| PC10 TX  PC11 RX 115200',
         '  $TAG / $QR / $CLS / $HB 数据包',
         'USART2 蓝牙  | PD5      PD6     9600',
         '  已禁用: ISR已#if 0 NVIC冲突',
         'SWD 调试: PA13/SWDIO  PA14/SWCLK'], 'comm')

    # 传感器 (上中)
    box(root, 350, 650, 200, 140, '传感器',
        ['5路巡线: PC0-PC3 PA4',
         'GPIO输入 低电平有效',
         'PVD电压监测: 2.9V',
         'NVIC 0,0 最高优先'], 'perception')

    # NVIC 优先表 (右下角)
    box(root, 750, 340, 200, 200, 'NVIC 优先级',
        ['PVD    0,0  (掉电制动)',
         'UART5  0,0  (可抢占TIM6)',
         'TIM6   1,0  (100Hz心跳)',
         'USART3 2,0  (OpenMV)',
         'USART2 2,0  (蓝牙 已禁用)',
         'EXTI1  3,3  (计数器 空)'], 'decision')

    # 连接线
    arrow(root, mcu, 22)   # MCU→电机
    arrow(root, mcu, 23)   # MCU→编码器
    arrow(root, mcu, 24)   # MCU→舵机
    arrow(root, mcu, 25)   # MCU→通信
    arrow(root, mcu, 26)   # MCU→传感器

    title_id = next_id()
    add_cell(root, title_id,
             value='<b><font style="font-size:16px">图5  硬件连接拓扑与引脚分配</font></b>',
             style=STYLES['label'], vertex='1', x=350, y=0, w=500, h=30)

    save(mxfile, '图5_硬件连接拓扑.drawio')


# ═══════════════════════════════════════════════
# 一键生成全部
# ═══════════════════════════════════════════════
if __name__ == '__main__':
    print('生成 drawio 文件...')
    make_pid()
    make_system()
    make_ai_pipeline()
    make_statemachine()
    make_hardware()
    print('完成! 用 draw.io 桌面版打开 .drawio 文件即可编辑')
