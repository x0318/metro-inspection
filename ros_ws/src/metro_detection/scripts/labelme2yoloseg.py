#!/usr/bin/env python3
"""
Labelme JSON → YOLOv8 Instance Segmentation 格式转换工具

Labelme标注流程:
  1. 用 Labelme 打开图片, 选择 "Create Polygons"
  2. 沿病害边缘逐点勾勒, 标签名用英文定义好的类别名
  3. 保存为同名 .json 文件 (与图片同目录)

YOLOv8-seg 标签格式:
  每行一个实例: class_id x1 y1 x2 y2 ... xn yn
  坐标归一化到 [0, 1]

用法:
  # 单目录模式 (所有 json 在一个目录)
  python scripts/labelme2yoloseg.py \
    --json_dir  data/annotations/ \
    --output_dir data/yolo_seg/ \
    --val_ratio 0.15

  # 多子目录模式 (每个类别一个子文件夹, 如 5class_labelme)
  python scripts/labelme2yoloseg.py \
    --json_dir  /media/zt/A934-128D/5class_labelme/ \
    --multi_dir \
    --output_dir data/yolo_seg/ \
    --normal_dir 正常状态图片/ \
    --val_ratio 0.2

输出结构:
  data/yolo_seg/
  ├── images/train/    # 训练图片 (软链接或复制)
  ├── images/val/      # 验证图片
  ├── labels/train/    # 训练标签 .txt
  ├── labels/val/      # 验证标签 .txt
  └── data.yaml        # YOLOv8 数据集配置
"""

import json
import os
import sys
import shutil
import random
import argparse
from pathlib import Path
from collections import defaultdict


# ── 类别映射 ────────────────────────────────────────────
# Labelme 中使用的 label 名 → YOLO class_id
# 5类: 土建裂缝/渗漏水 + 设备异物/管线/管片 (不含轨道扣件)
CLASS_MAP = {
    "liefeng":         0,   # 裂缝
    "shenloushui":     1,   # 渗漏水
    "water_leakage":   1,   # 渗漏水 (部分数据集别名)
    "yiwu":            2,   # 异物入侵
    "guanpianposun":   3,   # 管片破损掉块
    "guanxiansongtuo": 4,   # 管线支架松脱
}

CLASS_NAMES = {
    0: "liefeng",
    1: "shenloushui",
    2: "yiwu",
    3: "guanpianposun",
    4: "guanxiansongtuo",
}

CLASS_CN = {
    0: "裂缝",
    1: "渗漏水",
    2: "异物入侵",
    3: "管片破损掉块",
    4: "管线支架松脱",
}


def parse_labelme_json(json_path: str) -> dict:
    """
    解析单个 Labelme JSON 文件
    返回:
      {
        "image_path": str,        # 原始图片路径
        "image_width": int,
        "image_height": int,
        "shapes": [
          {"label": "liefeng", "points": [[x1,y1], [x2,y2], ...]},
          ...
        ]
      }
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 图片路径: 优先 imagePath, 否则用 json 文件名猜测
    image_path = data.get("imagePath", "")
    if image_path:
        # imagePath 可能是相对路径, 相对于 JSON 所在目录解析
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(json_path), image_path)
    else:
        base = os.path.splitext(os.path.basename(json_path))[0]
        # 尝试常见扩展名
        for ext in [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]:
            candidate = os.path.join(os.path.dirname(json_path), base + ext)
            if os.path.exists(candidate):
                image_path = candidate
                break

    image_width = data.get("imageWidth", 0)
    image_height = data.get("imageHeight", 0)

    shapes = []
    for s in data.get("shapes", []):
        label = s.get("label", "").strip()
        shape_type = s.get("shape_type", "polygon")
        points = s.get("points", [])

        # 只处理多边形
        if shape_type not in ("polygon", "linestrip"):
            print(f"  ⚠ 跳过非多边形标注: {label} ({shape_type})")
            continue

        if label not in CLASS_MAP:
            print(f"  ⚠ 未知类别标签 '{label}', 跳过 (请在 CLASS_MAP 中注册)")
            continue

        if len(points) < 3:
            print(f"  ⚠ 多边形点数不足: {label} ({len(points)} 个点)")
            continue

        shapes.append({
            "label": label,
            "points": points,
        })

    return {
        "image_path": image_path,
        "image_width": image_width,
        "image_height": image_height,
        "shapes": shapes,
    }


def normalize_points(points: list, width: int, height: int) -> list:
    """将像素坐标归一化到 [0,1]"""
    normalized = []
    for x, y in points:
        nx = max(0.0, min(1.0, x / max(width, 1)))
        ny = max(0.0, min(1.0, y / max(height, 1)))
        normalized.extend([nx, ny])
    return normalized


def convert_single(json_path: str, output_label_dir: str, copy_image_to: str = None) -> dict:
    """
    转换单个 Labelme JSON → YOLOv8-seg .txt
    返回统计信息
    """
    parsed = parse_labelme_json(json_path)
    if not parsed["image_width"] or not parsed["image_height"]:
        print(f"  ❌ 缺少图片尺寸信息: {json_path}")
        return {"error": True, "shapes": 0}

    # 确定输出的 txt 文件名
    json_stem = os.path.splitext(os.path.basename(json_path))[0]
    txt_path = os.path.join(output_label_dir, f"{json_stem}.txt")

    # 写入 YOLO-seg 格式
    lines = []
    for shape in parsed["shapes"]:
        class_id = CLASS_MAP[shape["label"]]
        norm_points = normalize_points(
            shape["points"],
            parsed["image_width"],
            parsed["image_height"]
        )
        line = f"{class_id} " + " ".join(f"{v:.6f}" for v in norm_points)
        lines.append(line)

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + ("\n" if lines else ""))

    # 如果图片路径存在, 复制到目标目录
    img_copied = False
    if copy_image_to and os.path.exists(parsed["image_path"]):
        ext = os.path.splitext(parsed["image_path"])[1]
        dst = os.path.join(copy_image_to, f"{json_stem}{ext}")
        if not os.path.exists(dst):
            shutil.copy2(parsed["image_path"], dst)
        img_copied = True

    return {
        "error": False,
        "shapes": len(parsed["shapes"]),
        "classes": [CLASS_MAP[s["label"]] for s in parsed["shapes"]],
        "image_copied": img_copied,
        "image_path": parsed["image_path"],
    }


def generate_data_yaml(output_dir: str, train_images: str, val_images: str) -> str:
    """生成 YOLOv8-seg 使用的 data.yaml"""
    yaml_path = os.path.join(output_dir, "data.yaml")
    # 用相对路径
    content = f"""# YOLOv8 Instance Segmentation Dataset
# 地铁病害智能巡检 - 5类 (土建+设备, 不含轨道扣件)

path: {os.path.abspath(output_dir)}
train: images/train
val: images/val

# 类别数
nc: 5

# 类别名
names:
  0: liefeng           # 裂缝
  1: shenloushui       # 渗漏水
  2: yiwu              # 异物入侵
  3: guanpianposun     # 管片破损掉块
  4: guanxiansongtuo   # 管线支架松脱
"""
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(content)
    return yaml_path


def main():
    parser = argparse.ArgumentParser(
        description="Labelme JSON → YOLOv8-seg 实例分割格式转换"
    )
    parser.add_argument(
        "--json_dir", required=True,
        help="Labelme JSON 标注文件所在目录"
    )
    parser.add_argument(
        "--image_dir", default=None,
        help="原始图片所在目录 (默认与 --json_dir 相同)"
    )
    parser.add_argument(
        "--output_dir", default="data/yolo_seg",
        help="输出目录 (默认 data/yolo_seg)"
    )
    parser.add_argument(
        "--val_ratio", type=float, default=0.15,
        help="验证集比例 (默认 0.15)"
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="随机种子 (默认 42)"
    )
    parser.add_argument(
        "--normal_dir", default=None,
        help="正常状态图片目录, 会加入训练集 (无标注, 生成空txt)"
    )
    parser.add_argument(
        "--multi_dir", action="store_true",
        help="json_dir 为多子目录结构 (每个子文件夹为一个类别, 如 liefeng/shenloushui/...)"
    )
    args = parser.parse_args()

    json_dir = args.json_dir
    image_dir = args.image_dir or json_dir
    output_dir = args.output_dir
    random.seed(args.seed)

    # ── 创建目录 ──
    for sub in ["images/train", "images/val", "labels/train", "labels/val"]:
        os.makedirs(os.path.join(output_dir, sub), exist_ok=True)

    # ── 收集所有 JSON 文件 ──
    if args.multi_dir:
        # 多子目录模式: 递归收集所有子文件夹中的 json
        json_files = []
        for root, dirs, files in os.walk(json_dir):
            for f in files:
                if f.endswith(".json"):
                    json_files.append(os.path.join(root, f))
        json_files.sort()
    else:
        json_files = sorted([
            os.path.join(json_dir, f) for f in os.listdir(json_dir)
            if f.endswith(".json")
        ])

    if not json_files:
        print("❌ 未找到任何 Labelme JSON 文件, 请检查 --json_dir")
        sys.exit(1)

    print(f"\n📂 找到 {len(json_files)} 个 Labelme JSON 标注文件\n")

    # ── 按类别分层划分 train/val ──
    # 先扫描所有标注, 收集类别分布
    file_class_info = {}
    for jf in json_files:
        parsed = parse_labelme_json(jf)
        classes = [CLASS_MAP[s["label"]] for s in parsed["shapes"]]
        file_class_info[jf] = classes

    # 确保每个类别至少有一个样本在验证集
    val_files = set()
    class_has_val = defaultdict(int)
    all_files = list(json_files)
    random.shuffle(all_files)

    for jf in all_files:
        classes = file_class_info[jf]
        for c in classes:
            if class_has_val[c] < 1:
                val_files.add(jf)
                class_has_val[c] += 1

    # 剩余按比例分配
    remaining = [f for f in json_files if f not in val_files]
    random.shuffle(remaining)
    n_val_extra = max(0, int(len(json_files) * args.val_ratio) - len(val_files))
    val_files.update(remaining[:n_val_extra])
    train_files = [f for f in json_files if f not in val_files]

    print(f"📊 数据集划分: 训练 {len(train_files)} 张, 验证 {len(val_files)} 张")
    print(f"   验证集确保每个类别至少有 1 个样本\n")

    # ── 执行转换 ──
    stats = {"train": defaultdict(int), "val": defaultdict(int), "errors": 0}

    for split, file_list in [("train", train_files), ("val", val_files)]:
        label_dir = os.path.join(output_dir, f"labels/{split}")
        image_dst = os.path.join(output_dir, f"images/{split}")

        print(f"🔄 转换 {split} 集 ({len(file_list)} 个文件)...")

        for jf in file_list:
            result = convert_single(jf, label_dir, copy_image_to=image_dst)
            if result["error"]:
                stats["errors"] += 1
                continue
            for cls_id in result["classes"]:
                stats[split][cls_id] += 1

            if not result["image_copied"]:
                print(f"  ⚠ 图片未找到: {os.path.basename(jf)}")

    # ── 处理正常状态图片 ──
    if args.normal_dir and os.path.isdir(args.normal_dir):
        print(f"\n🔄 处理正常状态图片 (负样本)...")
        normal_count = 0
        for root, dirs, files in os.walk(args.normal_dir):
            for f in files:
                if f.lower().endswith((".jpg", ".jpeg", ".png")):
                    src = os.path.join(root, f)
                    stem = os.path.splitext(f)[0]
                    ext = os.path.splitext(f)[1]

                    # 复制到训练集
                    dst_img = os.path.join(output_dir, "images/train", f"normal_{stem}{ext}")
                    dst_label = os.path.join(output_dir, "labels/train", f"normal_{stem}.txt")

                    if not os.path.exists(dst_img):
                        shutil.copy2(src, dst_img)
                    # 空标签文件
                    open(dst_label, "w").close()
                    normal_count += 1

        # 正常样本也放一点到验证集
        normal_val_count = max(1, int(normal_count * args.val_ratio))
        normal_imgs = sorted([
            f for f in os.listdir(os.path.join(output_dir, "images/train"))
            if f.startswith("normal_")
        ])
        random.shuffle(normal_imgs)
        for nf in normal_imgs[:normal_val_count]:
            src_img = os.path.join(output_dir, "images/train", nf)
            src_label = os.path.join(output_dir, "labels/train", nf.replace(os.path.splitext(nf)[1], ".txt"))
            dst_img = os.path.join(output_dir, "images/val", nf)
            dst_label = os.path.join(output_dir, "labels/val", nf.replace(os.path.splitext(nf)[1], ".txt"))
            shutil.move(src_img, dst_img)
            shutil.move(src_label, dst_label)

        print(f"   正常样本: 训练 {normal_count - normal_val_count} 张, 验证 {normal_val_count} 张")

    # ── 生成 data.yaml ──
    yaml_path = generate_data_yaml(output_dir, "images/train", "images/val")
    print(f"\n✅ data.yaml → {yaml_path}")

    # ── 打印统计 ──
    print("\n" + "=" * 60)
    print("📊 转换完成 — 类别统计")
    print("=" * 60)
    print(f"{'ID':<4} {'类别名':<18} {'中文':<14} {'训练':<6} {'验证':<6} {'合计':<6}")
    print("-" * 60)
    total_train = total_val = 0
    for cls_id in range(5):
        t = stats["train"].get(cls_id, 0)
        v = stats["val"].get(cls_id, 0)
        total_train += t
        total_val += v
        print(f"{cls_id:<4} {CLASS_NAMES[cls_id]:<18} {CLASS_CN[cls_id]:<14} {t:<6} {v:<6} {t+v:<6}")
    print("-" * 60)
    print(f"{'':<4} {'合计':<18} {'':<14} {total_train:<6} {total_val:<6} {total_train+total_val:<6}")
    print(f"\n错误: {stats['errors']} 个文件")
    print(f"\n🚀 下一步: python scripts/train_seg.py --data {yaml_path}")


if __name__ == "__main__":
    main()
