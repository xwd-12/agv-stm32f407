# 用反推出的位序规则生成 TAG36H11 ID=1
import numpy as np
from PIL import Image
import pupil_apriltags as p
import os

# 官方编码表 (apriltag-0.0.16/tag36h11.c)
codes = {
    0: 0x0000000d5d628584,
    1: 0x0000000d97f18b49,
    2: 0x0000000dd280910e,
    3: 0x0000000e479e9c98,
    4: 0x0000000ebcbca822,
}

def render(code, scale=50):
    # 规则 (0,0,1,1): 行正序, 列正序, bit0=右下, 数据取反(1=白)
    data = np.zeros((6, 6), dtype=int)  # 1 = white
    for i in range(36):
        r = i // 6
        c = i % 6
        bit = (code >> i) & 1
        data[r, c] = bit  # bit=1 -> 白
    full = np.zeros((8, 8), dtype=np.uint8)
    for r in range(6):
        for c in range(6):
            full[1 + r, 1 + c] = 255 if data[r, c] else 0
    # 边框: 黑色(0)
    img = np.kron(full, np.ones((scale, scale), dtype=np.uint8))
    return img.astype(np.uint8)

outdir = r'E:\STM32F4AGV智能搬运机器人 - 副本\从车对接_apriltag'
det = p.Detector(families='tag36h11')

# 先验证 ID2/3 渲染正确
for tid in [2, 3, 4]:
    img = render(codes[tid])
    tags = det.detect(img)
    print('render ID', tid, '-> detected', [t.tag_id for t in tags])

# 生成 ID=1
img1 = render(codes[1])
tags1 = det.detect(img1)
print('render ID 1 -> detected', [t.tag_id for t in tags1])
if tags1 and tags1[0].tag_id == 1:
    Image.fromarray(img1).save(os.path.join(outdir, 'tag36h11_id1_50mm.png'))
    print('SAVED tag36h11_id1_50mm.png', img1.shape)
else:
    print('FAILED - not saving')
