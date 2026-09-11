# 反推 TAG36H11 位序: 从官方 10x10 原始图 + 官方编码表
import numpy as np
from PIL import Image
import os

d = r'E:\STM32F4AGV智能搬运机器人 - 副本\apriltags'

def load_data(f):
    a = np.array(Image.open(os.path.join(d, f)).convert('L'))
    return (a[2:8, 2:8] < 128).astype(int)  # 6x6, 1=black

data2 = load_data('tag36_11_00002.png')
data3 = load_data('tag36_11_00003.png')
code2 = 0x0000000dd280910e
code3 = 0x0000000e479e9c98

def encode(data, rowflip, colflip, bit0pos, invert):
    bits = 0
    for i in range(36):
        r = (i // 6) if not rowflip else (5 - i // 6)
        c = (i % 6) if not colflip else (5 - i % 6)
        v = data[r][c]
        if invert:
            v = 1 - v
        i2 = (35 - i) if bit0pos else i
        if v:
            bits |= (1 << i2)
    return bits

found = []
for rf in [0, 1]:
    for cf in [0, 1]:
        for b0 in [0, 1]:
            for inv in [0, 1]:
                if encode(data2, rf, cf, b0, inv) == code2:
                    found.append((rf, cf, b0, inv))

print('ID2 matches:', found)
for (rf, cf, b0, inv) in found:
    ok = 'MATCH' if encode(data3, rf, cf, b0, inv) == code3 else 'NO'
    print('ID3 verify', (rf, cf, b0, inv), '->', ok)
