# 生成 TAG36H11 ID=0 的 AprilTag (官方编码 0xd5d628584)
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

def render10(code):
    # 位序规则 (0,0,1,1): 行正序,列正序,bit0=右下,bit=1->白
    # 布局: 白边1 + 黑框1 + 6x6数据 + 黑框1 + 白边1 = 10x10
    g = np.full((10, 10), 255, dtype=np.uint8)  # 白底
    g[1:9, 1:9] = 0                              # 黑框
    for i in range(36):
        r = i // 6
        c = i % 6
        bit = (code >> i) & 1
        g[2 + r, 2 + c] = 255 if bit else 0      # bit=1 -> 白
    return g

outdir = r'E:\STM32F4AGV智能搬运机器人 - 副本\从车对接_apriltag'
det = p.Detector(families='tag36h11')

# 生成 ID=0 并检测验证
g0 = render10(codes[0])
big0 = np.kron(g0, np.ones((50, 50), dtype=np.uint8)).astype(np.uint8)
tags0 = det.detect(big0)
print('ID0 scaled detect:', [t.tag_id for t in tags0])

if tags0 and tags0[0].tag_id == 0:
    img = Image.fromarray(g0).resize((500, 500), Image.NEAREST)
    img.save(os.path.join(outdir, 'tag36h11_id0_50mm.png'))
    print('SAVED tag36h11_id0_50mm.png (500x500)')
else:
    print('FAILED - not saving')
