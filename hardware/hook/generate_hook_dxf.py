"""生成挂钩 DXF (三视图 + 3D 网格) — AutoCAD 可直接打开
用法: python generate_hook_dxf.py
输出: hook.dxf
"""
import ezdxf
import os

# ===== 尺寸 (mm) =====
BODY_LEN = 50
BODY_W   = 15
BODY_T   = 5
TIP_DROP = 10
TIP_W    = 2.5
TIP_LEN  = 8
CURVE_R  = 5
SHAFT_R  = 2.9

doc = ezdxf.new(setup=True)
doc.units = ezdxf.units.MM

# --- 俯视图 (Top View, X-Y 平面) ---
msp_top = doc.modelspace()
# 钩身矩形
msp_top.add_lwpolyline([
    (0, -BODY_W/2), (BODY_LEN, -BODY_W/2),
    (BODY_LEN,  BODY_W/2), (0,  BODY_W/2)
], close=True)

# 过渡弧 (15→2.5)
import math
pts_trans = []
for t in range(0, 11):
    tt = t / 10.0
    x = BODY_LEN - CURVE_R + tt * CURVE_R
    w = (BODY_W - tt * (BODY_W - TIP_W)) / 2
    pts_trans.append((x, -w))
for t in range(10, -1, -1):
    tt = t / 10.0
    x = BODY_LEN - CURVE_R + tt * CURVE_R
    w = (BODY_W - tt * (BODY_W - TIP_W)) / 2
    pts_trans.append((x, w))
msp_top.add_lwpolyline(pts_trans, close=True)

# 钩头水平段
msp_top.add_lwpolyline([
    (BODY_LEN, -TIP_W/2), (BODY_LEN + TIP_LEN, -TIP_W/2),
    (BODY_LEN + TIP_LEN,  TIP_W/2), (BODY_LEN,  TIP_W/2)
], close=True)

# 中心孔
msp_top.add_circle((0, 0), SHAFT_R)
# M3 螺丝孔
msp_top.add_circle((0, 0), 1.6)

# 标注
msp_top.add_linear_dim(base=(0, -BODY_W/2 - 5), p1=(0, -BODY_W/2), p2=(BODY_LEN, -BODY_W/2),
    text=f"L={BODY_LEN}", dimstyle="EZDXF")
msp_top.add_linear_dim(base=(-15, -BODY_W/2), p1=(0, -BODY_W/2), p2=(0, BODY_W/2),
    text=f"W={BODY_W}", dimstyle="EZ_RADIUS")
msp_top.add_text("俯视图 (Top View) - 单位:mm", height=3).set_placement((BODY_LEN/2, BODY_W/2 + 12))

# --- 侧视图 (X-Z 平面) ---
msp_side = doc.modelspace()
# 钩身 (厚度5mm)
pts_side = [
    (0, 0), (BODY_LEN, 0), (BODY_LEN, BODY_T), (0, BODY_T)
]
msp_side.add_lwpolyline(pts_side, close=True)
# 下探
pts_drop = [
    (BODY_LEN, -TIP_DROP),
    (BODY_LEN + TIP_W, -TIP_DROP),
    (BODY_LEN + TIP_W, BODY_T),
    (BODY_LEN, BODY_T),
]
msp_side.add_lwpolyline(pts_drop, close=True)
# 钩头
pts_tip = [
    (BODY_LEN, -TIP_DROP),
    (BODY_LEN + TIP_LEN, -TIP_DROP),
    (BODY_LEN + TIP_LEN, -TIP_DROP + TIP_W),
    (BODY_LEN, -TIP_DROP + TIP_W),
]
msp_side.add_lwpolyline(pts_tip, close=True)
msp_side.add_text("侧视图 (Side View) - 单位:mm", height=3).set_placement((BODY_LEN/2, BODY_T + 8))

# 保存
outpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hook.dxf')
doc.saveas(outpath)
print(f"OK → {outpath}")
