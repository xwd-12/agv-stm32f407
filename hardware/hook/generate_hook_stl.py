"""生成挂钩舵机钩子 STL
用法: python generate_hook_stl.py
输出: hook.stl
"""
import trimesh
import numpy as np
import os

# ===== 尺寸 (mm) =====
BODY_LEN   = 50     # 总长 (舵机轴心→钩尖起点)
BODY_W     = 15     # 钩身宽
BODY_T     = 5      # 钩身厚
TIP_DROP   = 10     # 钩头下探
TIP_W      = 2.5    # 钩头宽 (适配3~4mm槽口)
TIP_LEN    = 8      # 钩头水平段长
CURVE_R    = 5      # 过渡圆弧半径
SHAFT_R    = 2.9    # 中心孔半径 (φ5.8)
SCREW_R    = 1.6    # M3螺丝孔
SCREW_HEAD_R = 3.25 # M3沉头半径
SCREW_HEAD_D = 2.5  # 沉头深度

# 原点 = 舵机轴心 (0,0,0), 钩子沿 +X 伸出
E = 'manifold'  # boolean engine
parts = []

# === 1. 钩身平板 (0,0,0)→(50,15,5) ===
body = trimesh.creation.box(extents=(BODY_LEN, BODY_W, BODY_T))
body.apply_translation((BODY_LEN/2, 0, BODY_T/2))
parts.append(body)

# === 2. 过渡弧 (15宽→2.5宽), 用 hull 近似 ===
# 在过渡区域放置一个大矩形+一个小矩形, hull 自动生成锥面
for t in np.linspace(0, 1, 6):
    x = BODY_LEN - CURVE_R + t * CURVE_R
    w = BODY_W - t * (BODY_W - TIP_W)
    seg = trimesh.creation.box(extents=(CURVE_R/6 + 0.5, w, BODY_T))
    seg.apply_translation((x, 0, BODY_T/2))
    parts.append(seg)

# === 3. 钩头下探 (垂直柱) ===
drop = trimesh.creation.box(extents=(TIP_W, TIP_W, TIP_DROP))
drop.apply_translation((BODY_LEN, 0, BODY_T - TIP_DROP/2))
parts.append(drop)

# === 4. 钩头水平段 ===
tip = trimesh.creation.box(extents=(TIP_LEN, TIP_W, TIP_W))
tip.apply_translation((BODY_LEN + TIP_LEN/2, 0, BODY_T - TIP_DROP + TIP_W/2))
parts.append(tip)

# === 5. 倒钩 ===
barb = trimesh.creation.box(extents=(3, TIP_W, 2))
barb.apply_translation((BODY_LEN + TIP_LEN - 1, 0, BODY_T - TIP_DROP - 1))
parts.append(barb)

# === 6. 钩头圆角 (小半圆柱补在转角处) ===
fillet = trimesh.creation.cylinder(radius=2, height=TIP_W, sections=8)
fillet.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, (0,1,0)))
fillet.apply_translation((BODY_LEN, 0, BODY_T - TIP_DROP + TIP_W/2))
parts.append(fillet)

# === 合并 ===
hook = trimesh.boolean.union(parts, engine=E)
print(f"union done, verts={len(hook.vertices)} faces={len(hook.faces)}")

# === 7. 中心孔 ===
hole_shaft = trimesh.creation.cylinder(radius=SHAFT_R, height=BODY_T + 2, sections=32)
hole_shaft.apply_translation((0, 0, BODY_T/2))
hook = trimesh.boolean.difference([hook, hole_shaft], engine=E)
print(f"hole_shaft done")

hole_screw = trimesh.creation.cylinder(radius=SCREW_R, height=BODY_T + 2, sections=16)
hole_screw.apply_translation((0, 0, BODY_T/2))
hook = trimesh.boolean.difference([hook, hole_screw], engine=E)
print(f"hole_screw done")

hole_head = trimesh.creation.cylinder(radius=SCREW_HEAD_R, height=SCREW_HEAD_D + 0.1, sections=16)
hole_head.apply_translation((0, 0, BODY_T - SCREW_HEAD_D/2))
hook = trimesh.boolean.difference([hook, hole_head], engine=E)
print(f"hole_head done")

# === 导出 ===
outpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hook.stl')
hook.export(outpath)
print(f"OK → {outpath}")
print(f"  顶点: {len(hook.vertices)}, 三角面: {len(hook.faces)}")
