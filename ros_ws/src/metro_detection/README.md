# 2D 视觉检测模块 (metro_detection)

本模块负责地铁隧道巡检过程中的 2D 图像病害检测，识别 8 类病害、覆盖轨道 / 设备 / 土建三大领域。

---

## 1. 检测模型

采用「两模型分治」策略（YOLOv8-seg 实例分割，`imgsz=640`）：

| 模型 | 领域 | 骨干 | 类别数 | 输入 |
|------|------|------|:---:|------|
| 模型 A | 土建 + 设备 | yolov8m-seg | 5 | 彩色 |
| 模型 B | 轨道扣件 | yolov8l-seg | 3 | 灰度 |

> 模型权重（`.pt`）体积较大，不随本仓库分发。训练 / 评估 / 标注代码见 `scripts/`，类别表见 `classes.yaml`。

## 2. 类别体系

| ID | 类别名 | 中文 | 领域 |
|:--:|--------|------|:----:|
| 0 | koujianwaixie | 扣件松动歪斜 | 轨道 |
| 1 | koujianduanlie | 扣件断裂 | 轨道 |
| 2 | koujianqueshi | 扣件缺失 | 轨道 |
| 3 | yiwu | 异物入侵 | 设备 |
| 4 | guanxiansongtuo | 管线支架松脱 | 设备 |
| 5 | guanpianposun | 管片破损掉块 | 土建 |
| 6 | liefeng | 裂缝 | 土建 |
| 7 | shenloushui | 渗漏水 | 土建 |

检出率目标（赛制）：土建 ≥95%、轨道 ≥95%、设备 ≥97%。

## 3. ROS 接口（联调用）

* **输入**：`/camera/image_raw`（`sensor_msgs/msg/Image`，车载相机 RGB 图）
* **输出**：`/detections`（`std_msgs/msg/String`，标准 JSON 字符串）
```json
{"type": "crack", "confidence": 0.95, "bbox": [620, 310, 760, 390]}
```
* 前期联调用假数据节点：`fake_detection_node.py`（每秒发布一条模拟检测结果，供三维同化组解算坐标联调）。

## 4. 目录结构

```
metro_detection/
├── fake_detection_node.py     # ROS 假数据节点（联调）
├── test_pipline.py            # 推理演示：YOLO + 像素→物理坐标映射
├── classes.yaml               # 8 类病害类别表 + 模型类映射
├── requirements.txt           # Python 依赖
└── scripts/                   # 训练 / 评估 / 标注脚本
    ├── train_seg.py               # 模型A 训练（土建+设备 5类）
    ├── train_fastener.py          # 模型B 训练（扣件 3类）
    ├── add_backgrounds.py         # 模型B 背景注入
    ├── validate_imagelevel.py     # 图像级检出率评估
    ├── bbox2sam_labelme.py        # BBox→SAM→Labelme
    ├── labelme2yoloseg.py         # Labelme→YOLO-seg
    ├── consolidate_5class.py      # 5类数据合并
    ├── sam_preseg_yiwu.py         # SAM 外部数据预标注
    └── sam_preseg_leakage.py      # SAM 外部数据预标注
```

## 5. 环境

```bash
conda create -n subway_cv python=3.10 -y
conda activate subway_cv
pip install -r requirements.txt
pip install torch==2.6.0 torchvision --index-url https://download.pytorch.org/whl/cu124
```

## 6. 推理 / 训练 / 评估

```python
from ultralytics import YOLO
model_a = YOLO("path/to/model_a_civil_yolov8m-seg.pt")     # 土建+设备（彩色）
model_b = YOLO("path/to/model_b_fastener_yolov8l-seg.pt")  # 扣件（灰度）
```

```bash
# 训练模型A（土建+设备 5类）
python3 scripts/train_seg.py --name train2 --model yolov8m-seg.pt
# 训练模型B（扣件 3类，含 Labelme→YOLO 转换 + 背景注入 + 过采样）
python3 scripts/train_fastener.py
# 图像级检出率评估（验证集）
python3 scripts/validate_imagelevel.py --data <data.yaml> --weights <best.pt> [--gray]
```

> 预处理细节：模型A 用彩色图推理；模型B 需先灰度化再复制为 3 通道（与训练一致）。
