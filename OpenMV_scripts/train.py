# train.py — 图片分类训练脚本
# 用法:
#   1. pip install tensorflow pillow numpy
#   2. 把 dataset/ 目录放到本脚本同级
#   3. python train.py
#   4. 输出 model.tflite 和 labels.txt 拷到 SD 卡

import os
import numpy as np
from PIL import Image
import tensorflow as tf

# ====== 配置 ======
IMG_SIZE    = 96        # 输入尺寸, 96x96 够区分形状和颜色
BATCH_SIZE  = 16
EPOCHS      = 20
DATA_DIR    = "dataset"
MODEL_OUT   = "model.tflite"
LABELS_OUT  = "labels.txt"
# ==================

# 1. 扫描类别 (按颜色顺序: 红=0, 绿=1, 黄=2, 对应 STM32 color_id = class_id+1)
CLASS_ORDER = ["red_hexagon", "green_circle", "yellow_rect"]
class_dirs = set([
    d for d in os.listdir(DATA_DIR)
    if os.path.isdir(os.path.join(DATA_DIR, d))
])
class_names = [c for c in CLASS_ORDER if c in class_dirs]
if not class_names:
    # fallback: 手动指定之外的目录也收入
    class_names = sorted(class_dirs)
num_classes = len(class_names)
print("Classes: %s (%d classes)" % (class_names, num_classes))
print("Color mapping: class0→color1(red), class1→color2(green), class2→color3(yellow)")

# 写标签文件
with open(LABELS_OUT, "w") as f:
    for name in class_names:
        f.write(name + "\n")
print("Saved: %s" % LABELS_OUT)

# 2. 加载图片
def load_dataset():
    images, labels = [], []
    for label_idx, class_name in enumerate(class_names):
        folder = os.path.join(DATA_DIR, class_name)
        files = [f for f in os.listdir(folder) if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))]
        print("  %s: %d images" % (class_name, len(files)))
        for fname in files:
            img = Image.open(os.path.join(folder, fname)).convert("RGB")
            img = img.resize((IMG_SIZE, IMG_SIZE))
            images.append(np.array(img, dtype=np.float32) / 255.0)
            labels.append(label_idx)
    return np.array(images), np.array(labels)

print("\nLoading dataset...")
x, y = load_dataset()
print("Total: %d images\n" % len(x))

# 3. 划分训练/验证集 (80/20)
indices = np.random.permutation(len(x))
split = int(len(x) * 0.8)
train_idx, val_idx = indices[:split], indices[split:]
x_train, y_train = x[train_idx], y[train_idx]
x_val,   y_val   = x[val_idx],   y[val_idx]

# 数据增强
data_aug = tf.keras.Sequential([
    tf.keras.layers.RandomFlip("horizontal"),
    tf.keras.layers.RandomRotation(0.1),
    tf.keras.layers.RandomZoom(0.1),
])

# 4. 构建模型 (轻量 MobileNetV2)
base = tf.keras.applications.MobileNetV2(
    input_shape=(IMG_SIZE, IMG_SIZE, 3),
    alpha=0.35,              # 最小变体, 模型 ~200KB
    include_top=False,
    weights="imagenet",
)
base.trainable = False       # 冻结 backbone

model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(IMG_SIZE, IMG_SIZE, 3)),
    data_aug,
    base,
    tf.keras.layers.GlobalAveragePooling2D(),
    tf.keras.layers.Dropout(0.3),
    tf.keras.layers.Dense(num_classes, activation="softmax"),
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(1e-3),
    loss="sparse_categorical_crossentropy",
    metrics=["accuracy"],
)

model.summary()

# 5. 第一阶段: 只训分类头
print("\nPhase 1: training classifier head...")
model.fit(
    x_train, y_train,
    batch_size=BATCH_SIZE,
    epochs=EPOCHS,
    validation_data=(x_val, y_val),
    verbose=1,
)

# 第二阶段: 解冻 backbone, 低学习率微调
base.trainable = True
model.compile(
    optimizer=tf.keras.optimizers.Adam(1e-5),
    loss="sparse_categorical_crossentropy",
    metrics=["accuracy"],
)
print("\nPhase 2: fine-tuning full network...")
model.fit(
    x_train, y_train,
    batch_size=BATCH_SIZE,
    epochs=10,
    validation_data=(x_val, y_val),
    verbose=1,
)

# 6. 导出量化 TFLite
def representative_dataset():
    for i in range(0, min(100, len(x_train)), 1):
        yield [x_train[i:i+1].astype(np.float32)]

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type  = tf.int8
converter.inference_output_type = tf.int8

tflite_model = converter.convert()
with open(MODEL_OUT, "wb") as f:
    f.write(tflite_model)

print("\nDone!  Output: %s  (%d bytes)" % (MODEL_OUT, len(tflite_model)))
print("Copy %s and %s to SD card root." % (MODEL_OUT, LABELS_OUT))
