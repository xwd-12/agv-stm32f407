# train_small.py — 轻量 CNN 训练脚本 (OpenMV 内存友好)
# 用法:
#   1. pip install tensorflow pillow numpy
#   2. python train_small.py
#   3. 输出 model_small.tflite 和 labels.txt 拷到 SD 卡
#   4. main.py 中 load 路径改为 "model_small.tflite"

import os
import numpy as np
from PIL import Image
import tensorflow as tf

# ====== 配置 ======
IMG_SIZE    = 64        # 64×64 足够区分形状+颜色
BATCH_SIZE  = 16
EPOCHS      = 30
DATA_DIR    = "dataset"
MODEL_OUT   = "model_small.tflite"
LABELS_OUT  = "labels.txt"
# ==================

# 1. 扫描类别
CLASS_ORDER = ["red_hexagon", "green_circle", "yellow_rect"]
class_dirs = set([
    d for d in os.listdir(DATA_DIR)
    if os.path.isdir(os.path.join(DATA_DIR, d))
])
class_names = [c for c in CLASS_ORDER if c in class_dirs]
if not class_names:
    class_names = sorted(class_dirs)
num_classes = len(class_names)
print("Classes:", class_names, "(%d)" % num_classes)

with open(LABELS_OUT, "w") as f:
    for name in class_names:
        f.write(name + "\n")
print("Labels saved:", LABELS_OUT)

# 2. 加载数据
def load_dataset():
    images, labels = [], []
    for label_idx, name in enumerate(class_names):
        folder = os.path.join(DATA_DIR, name)
        files = [f for f in os.listdir(folder)
                 if f.lower().endswith((".jpg",".jpeg",".png",".bmp"))]
        print("  %s: %d images" % (name, len(files)))
        for fname in files:
            img = Image.open(os.path.join(folder, fname)).convert("RGB")
            img = img.resize((IMG_SIZE, IMG_SIZE))
            images.append(np.array(img, dtype=np.float32) / 255.0)
            labels.append(label_idx)
    return np.array(images), np.array(labels)

print("\nLoading...")
x, y = load_dataset()
print("Total: %d images" % len(x))

# 3. 划分训练/验证
indices = np.random.permutation(len(x))
split = int(len(x) * 0.8)
x_train, y_train = x[indices[:split]], y[indices[:split]]
x_val,   y_val   = x[indices[split:]], y[indices[split:]]

# 4. Tiny CNN — 为 OpenMV 内存量身定做
#    目标: 模型 < 100KB (INT8)
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(IMG_SIZE, IMG_SIZE, 3)),

    # Block 1: 轻量卷积
    tf.keras.layers.Conv2D(8, 3, padding='same', activation='relu'),
    tf.keras.layers.MaxPooling2D(2),    # 64→32

    # Block 2
    tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu'),
    tf.keras.layers.MaxPooling2D(2),    # 32→16

    # Block 3
    tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu'),
    tf.keras.layers.MaxPooling2D(2),    # 16→8

    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(16, activation='relu'),
    tf.keras.layers.Dense(num_classes, activation='softmax'),
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(1e-3),
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy'],
)
model.summary()

# 5. 训练
model.fit(
    x_train, y_train,
    batch_size=BATCH_SIZE,
    epochs=EPOCHS,
    validation_data=(x_val, y_val),
    verbose=1,
)

# 6. 导出 Float32 模型 (纯基础算子, 无GAP/Dropout/stridedConv)
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()
with open(MODEL_OUT, "wb") as f:
    f.write(tflite_model)

print("\nDone!  %s: %d bytes (float32)" % (MODEL_OUT, len(tflite_model)))
