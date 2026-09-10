"""
扣件病害检测训练脚本
数据集: /media/zt/A934-128D/扣件模型数据/ (3类 + 正常背景, 全灰度)
模型: YOLOv8l-seg
"""

import os, sys, json, random, shutil
from pathlib import Path
from collections import defaultdict
from ultralytics import YOLO
import torch

# ─── 路径配置 ───────────────────────────────────────
LABELME_DIR = Path("/media/zt/A934-128D/扣件模型数据")  # Labelme 格式源数据
YOLO_DIR    = Path("/media/zt/A934-128D/subway_data/fastener_v6")
PROJECT     = "/media/zt/A934-128D/subway_runs/segment"
NAME        = "fastener_v6_wiou_tuned"  # v6: WIoU v3 tuned (δ=0.8, α=1.5) — gentler on easy class

# ─── 类别映射 (扣件3类, 正常=背景) ─────────────────
CLASS_MAP = {
    "koujianwaixie":   0,
    "koujianduanlie":  1,
    "koujianqueshi":   2,
}
CLASS_NAMES = {0: "koujianwaixie", 1: "koujianduanlie", 2: "koujianqueshi"}
CLASS_CN    = {0: "扣件松动歪斜", 1: "扣件断裂", 2: "扣件缺失"}

TRAIN_RATIO = 0.80
SEED = 42


# ══════════════════════════════════════════════════════════════
# 1. Labelme → YOLO-seg 转换
# ══════════════════════════════════════════════════════════════

def convert_labelme_to_yolo(labelme_dir: Path, yolo_dir: Path, split: str, file_list: list):
    """将指定文件列表从 Labelme 转为 YOLO-seg 格式"""
    img_dst = yolo_dir / "images" / split
    lbl_dst = yolo_dir / "labels" / split
    img_dst.mkdir(parents=True, exist_ok=True)
    lbl_dst.mkdir(parents=True, exist_ok=True)

    stats = defaultdict(int)

    for folder_name, json_name, img_name in file_list:
        json_path = labelme_dir / folder_name / json_name
        img_src  = labelme_dir / folder_name / img_name

        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        img_w = data.get("imageWidth", 640)
        img_h = data.get("imageHeight", 640)

        # 写标签文件
        label_lines = []
        for shape in data.get("shapes", []):
            label = shape.get("label", "").strip().lower()
            if label not in CLASS_MAP:
                continue

            cls_id = CLASS_MAP[label]
            points = shape.get("points", [])
            shape_type = shape.get("shape_type", "polygon")

            if len(points) < 2:
                continue

            # rectangle (2 points) → 转化为 4 点 polygon
            if len(points) == 2:
                x1, y1 = points[0]
                x2, y2 = points[1]
                points = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]

            # 归一化坐标
            norm_pts = []
            for x, y in points:
                norm_pts.append(f"{x / img_w:.6f}")
                norm_pts.append(f"{y / img_h:.6f}")

            label_lines.append(f"{cls_id} " + " ".join(norm_pts))
            stats[label] += 1

        # 复制图片
        src_img = img_src
        dst_img = img_dst / img_name
        if not dst_img.exists():
            shutil.copy2(src_img, dst_img)

        # 写标签 (空标签也写, 代表正常背景)
        dst_label = lbl_dst / (Path(json_name).stem + ".txt")
        with open(dst_label, "w") as f:
            f.write("\n".join(label_lines))

    return stats


def build_dataset(labelme_dir: Path, yolo_dir: Path):
    print(f"\n{'='*55}")
    print(f"转换 Labelme → YOLO-seg")
    print(f"{'='*55}")
    print(f"源数据: {labelme_dir}")
    print(f"输出:   {yolo_dir}")
    print()

    # 清空旧数据
    if yolo_dir.exists():
        shutil.rmtree(yolo_dir)

    random.seed(SEED)

    # 收集所有图片
    all_files = []
    for folder in sorted(labelme_dir.iterdir()):
        if not folder.is_dir():
            continue
        for f in sorted(folder.iterdir()):
            if f.suffix == ".json":
                stem = f.stem
                img = None
                for ext in [".png", ".jpg", ".jpeg"]:
                    candidate = folder / (stem + ext)
                    if candidate.exists():
                        img = candidate.name
                        break
                if img:
                    all_files.append((folder.name, f.name, img))

    print(f"总图片: {len(all_files)}")
    for folder in sorted(set(f[0] for f in all_files)):
        cnt = sum(1 for f in all_files if f[0] == folder)
        print(f"  {folder}: {cnt}")

    # 打乱并分割
    random.shuffle(all_files)
    split_idx = int(len(all_files) * TRAIN_RATIO)
    train_files = [(f[0], f[1], f[2]) for f in all_files[:split_idx]]
    val_files   = [(f[0], f[1], f[2]) for f in all_files[split_idx:]]

    print(f"\nTrain: {len(train_files)}, Val: {len(val_files)}")

    # ── Class 2 (扣件缺失) 过采样: 含 class 2 的图在 train 中复制 1 次 ──
    def has_class2(folder_name, json_name):
        json_path = labelme_dir / folder_name / json_name
        with open(json_path, "r") as f:
            data = json.load(f)
        for shape in data.get("shapes", []):
            if shape.get("label", "").strip().lower() == "koujianqueshi":
                return True
        return False

    cls2_files = [(f, has_class2(f[0], f[1])) for f in train_files]
    cls2_count = sum(1 for _, ok in cls2_files if ok)
    train_files_dup = train_files + [f for f, ok in cls2_files if ok]
    print(f"Class 2 过采样: {cls2_count} 张含缺失 → 复制 → train 增至 {len(train_files_dup)}")
    train_files = train_files_dup
    # ──────────────────────────────────────────────────────────────

    # 转换
    print("\n[Train]")
    train_stats = convert_labelme_to_yolo(labelme_dir, yolo_dir, "train", train_files)
    print("[Val]")
    val_stats   = convert_labelme_to_yolo(labelme_dir, yolo_dir, "val", val_files)

    # 统计
    print(f"\n{'='*55}")
    print(f"类别分布")
    print(f"{'='*55}")
    for cls_name in sorted(CLASS_MAP.keys()):
        t = train_stats.get(cls_name, 0)
        v = val_stats.get(cls_name, 0)
        print(f"  {CLASS_CN[CLASS_MAP[cls_name]]:12s} (class {CLASS_MAP[cls_name]}): "
              f"train={t:5d}, val={v:4d}, 合计={t+v:5d}")

    # 写 data.yaml
    data_yaml = yolo_dir / "data.yaml"
    data_yaml.write_text(f"""# 扣件病害检测 — YOLOv8-seg
path: {yolo_dir}
train: images/train
val: images/val

nc: {len(CLASS_MAP)}
names:
  0: koujianwaixie      # 扣件松动歪斜
  1: koujianduanlie     # 扣件断裂
  2: koujianqueshi      # 扣件缺失
""", encoding="utf-8")

    print(f"\ndata.yaml → {data_yaml}")
    return data_yaml


# ══════════════════════════════════════════════════════════════
# 2. 训练
# ══════════════════════════════════════════════════════════════

def train(data_yaml: Path):
    print(f"\n{'='*55}")
    print(f"开始训练")
    print(f"{'='*55}")
    print(f"数据: {data_yaml}")
    print(f"模型: yolov8l-seg.pt")
    print(f"输出: {PROJECT}/{NAME}")
    print(f"设备: {'GPU' if torch.cuda.is_available() else 'CPU'}")
    print()

    model = YOLO("yolov8l-seg.pt")

    # ── WIoU v3 (tuned): δ=0.8 α=1.5 — gentler suppression for easy classes ──
    from ultralytics.utils import loss as loss_mod
    loss_mod.WIOU_MOMENTUM = 0.9
    loss_mod.WIOU_DELTA = 0.8    # lowered from 1.0: shift peak left, easier suppression
    loss_mod.WIOU_ALPHA = 1.5    # lowered from 1.9: softer curvature
    print(f"WIoU v3 tuned: momentum={loss_mod.WIOU_MOMENTUM}, "
          f"delta={loss_mod.WIOU_DELTA}, alpha={loss_mod.WIOU_ALPHA}")
    # ─────────────────────────────────────

    # v6 配置: WIoU v3 tuned (δ=0.8, α=1.5) + class-2 oversampling + bg injection
    # class 0 r≈0.83 (mild suppression), class 2 r≈1.55 (strong enhancement)
    results = model.train(
        data=str(data_yaml),
        project=PROJECT,
        name=NAME,
        exist_ok=True,
        pretrained=True,

        # 训练轮次
        epochs=200,
        patience=30,

        # 图像
        imgsz=640,
        batch=4,            # yolov8l 显存需求更高, 8→4 防 OOM (8GB GPU)
        workers=4,

        # 优化器
        lr0=0.001,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        cos_lr=True,
        warmup_epochs=3,

        # 数据增强 — 严防破坏扣件几何特征
        mosaic=1.0,            # ✅ mosaic造图多样本
        mixup=0.0,             # ❌ 避免断裂截面纹理被混合破坏
        copy_paste=0.0,        # ❌ 避免低清碎块贴上高清背景
        hsv_h=0.02,
        hsv_s=0.7,
        hsv_v=0.4,             # ✅ 强色彩抖动 (不改变几何)
        degrees=0.0,           # ❌ 锁定旋转, 保护歪斜角度判据
        scale=0.5,
        translate=0.1,         # ✅ 温和缩放平移
        shear=0.0,             # ❌ 禁止错切变形
        perspective=0.0,       # ❌ 禁止透视扭曲
        flipud=0.0,            # ❌ 扣件有重力方向, 不可上下颠倒
        fliplr=0.5,            # ✅ 左右翻转安全
        erasing=0.4,           # ✅ 随机遮挡, 提高抗干扰
        close_mosaic=15,       # 标准关闭时机

        # 损失权重 (v1: CIoU + BCE, 源码层未魔改)
        box=7.5,
        cls=1.0,
        dfl=1.5,

        # 验证
        val=True,
        save=True,
        save_period=-1,      # 只留 best/last, 防 checkpoint 堆爆磁盘
        conf=0.25,
        iou=0.6,
        max_det=300,

        # 硬件
        device=0 if torch.cuda.is_available() else "cpu",
        amp=True,
        seed=SEED,
        deterministic=False,

        # 可视化
        plots=True,
        show=False,
    )
    return results


# ══════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--convert-only", action="store_true", help="仅转换, 不训练")
    parser.add_argument("--train-only",  action="store_true", help="仅训练, 不转换")
    parser.add_argument("--skip-bg",     action="store_true", help="跳过背景注入")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--name", type=str, default=NAME)
    args = parser.parse_args()

    NAME = args.name

    if not args.train_only:
        data_yaml = build_dataset(LABELME_DIR, YOLO_DIR)
        # 注入多样化背景 (方案一: 降低背景误报)
        if not args.skip_bg:
            from add_backgrounds import add_backgrounds
            add_backgrounds(
                source_dir=Path("/home/zt/subway_project/病害训练集+正常状态/正常状态"),
                target_dir=YOLO_DIR,
                crops_per_large=15,
                crops_per_small=3,
                dry_run=False,
            )
    else:
        data_yaml = YOLO_DIR / "data.yaml"
        if not data_yaml.exists():
            print(f"ERROR: {data_yaml} 不存在，先运行 --convert-only")
            sys.exit(1)

    if not args.convert_only:
        train(data_yaml)
