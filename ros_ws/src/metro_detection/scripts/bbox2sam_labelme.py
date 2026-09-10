#!/usr/bin/env python3
"""
策略 B: 用现有 YOLO BBox 作为 SAM 的 box prompt → 自动分割 → Labelme JSON

工作流:
  输入:  images/ + labels/ (YOLO格式 BBox)
  处理:  BBox → SAM box prompt → mask → polygon → Labelme JSON
  输出:  annotations/ (Labelme JSON, 可直接在 Labelme 中打开精修)

用法:
  python scripts/bbox2sam_labelme.py

输出:
  为每张有标注的图片生成一个 .json 文件
  在 Labelme 中打开 → 修正边缘 → 保存 → 完成
"""

import os
import sys
import json
import cv2
import numpy as np
import torch
from pathlib import Path
from collections import defaultdict

# ── 路径配置 ────────────────────────────────────────────
BASE_DIR = "/home/zt/subway_project"
IMAGE_DIR = os.path.join(BASE_DIR, "病害训练集+正常状态/病害训练集/images")
LABEL_DIR = os.path.join(BASE_DIR, "病害训练集+正常状态/病害训练集/labels")
CLASS_FILE = os.path.join(BASE_DIR, "病害训练集+正常状态/病害训练集/class.txt")
OUTPUT_DIR = os.path.join(BASE_DIR, "data/annotations")
SAM_CHECKPOINT = os.path.join(BASE_DIR, "models/sam_vit_b_01ec64.pth")

# ── 类别映射 ────────────────────────────────────────────
CLASS_NAMES = {
    0: "koujianwaixie",    # 扣件松动歪斜
    1: "koujianduanlie",   # 扣件断裂
    2: "koujianqueshi",    # 扣件缺失
    3: "yiwu",             # 异物入侵
    4: "guanxiansongtuo",  # 管线支架松脱
    5: "guanpianposun",    # 管片破损掉块
    6: "liefeng",          # 裂缝
    7: "shenloushui",      # 渗漏水
}


def load_sam(checkpoint_path: str):
    """加载 SAM 模型"""
    from segment_anything import sam_model_registry, SamPredictor

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"  设备: {device}")

    sam = sam_model_registry["vit_b"](checkpoint=checkpoint_path)
    sam.to(device=device)
    sam.eval()

    predictor = SamPredictor(sam)
    return predictor, device


def yolo_to_pixel_bbox(norm_bbox: tuple, img_w: int, img_h: int) -> list:
    """
    将 YOLO 归一化 BBox 转为像素坐标
    YOLO: (class_id, x_center, y_center, w, h) normalized
    返回: [x1, y1, x2, y2] pixel
    """
    cls_id, xc, yc, w, h = norm_bbox
    x1 = int((xc - w / 2) * img_w)
    y1 = int((yc - h / 2) * img_h)
    x2 = int((xc + w / 2) * img_w)
    y2 = int((yc + h / 2) * img_h)
    # 边界保护
    x1 = max(0, min(x1, img_w - 1))
    y1 = max(0, min(y1, img_h - 1))
    x2 = max(x1 + 1, min(x2, img_w))
    y2 = max(y1 + 1, min(y2, img_h))
    return [x1, y1, x2, y2]


def mask_to_polygon(mask: np.ndarray, simplify_epsilon: float = 2.0) -> list:
    """
    将二值 mask 转为多边形点列表
    mask: (H, W) bool array
    返回: [[x1,y1], [x2,y2], ...]
    """
    # 找外轮廓
    mask_uint8 = mask.astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return []

    # 取面积最大的轮廓
    best = max(contours, key=cv2.contourArea)

    if len(best) < 3:
        return []

    # 简化多边形 (减少点数)
    epsilon = simplify_epsilon  # 像素级简化
    approx = cv2.approxPolyDP(best, epsilon, closed=True)
    points = approx.squeeze(axis=1).tolist()  # [[x,y], [x,y], ...]

    if len(points) < 3:
        return []

    return points


def process_single_image(image_path: str, labels: list,
                         predictor, device: str) -> dict:
    """
    处理单张图片
    labels: [(class_id, xc, yc, w, h), ...]
    返回 Labelme JSON dict
    """
    image_name = os.path.basename(image_path)

    # 读取图片
    img = cv2.imread(image_path)
    if img is None:
        print(f"  ❌ 无法读取图片: {image_path}")
        return None

    img_h, img_w = img.shape[:2]

    # 转 RGB (SAM 需要)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # 设置图像
    predictor.set_image(img_rgb)

    shapes = []
    for label in labels:
        cls_id, xc, yc, w, h = label
        cls_name = CLASS_NAMES.get(int(cls_id), f"class_{int(cls_id)}")

        # YOLO bbox → 像素 bbox
        pixel_bbox = yolo_to_pixel_bbox((cls_id, xc, yc, w, h), img_w, img_h)

        # 适当扩展 bbox 边界，给 SAM 更多上下文
        margin_x = int((pixel_bbox[2] - pixel_bbox[0]) * 0.1)
        margin_y = int((pixel_bbox[3] - pixel_bbox[1]) * 0.1)
        box_prompt = np.array([[
            max(0, pixel_bbox[0] - margin_x),
            max(0, pixel_bbox[1] - margin_y),
            min(img_w, pixel_bbox[2] + margin_x),
            min(img_h, pixel_bbox[3] + margin_y),
        ]])

        # SAM 推理
        masks, scores, _ = predictor.predict(
            box=box_prompt,
            multimask_output=False,  # 单个 mask
        )

        if masks is None or len(masks) == 0:
            print(f"  ⚠ SAM 未输出 mask: {cls_name} in {image_name}")
            continue

        mask = masks[0]
        score = scores[0]

        if score < 0.5:
            print(f"  ⚠ mask 置信度低 ({score:.2f}): {cls_name} in {image_name}")

        # mask → polygon
        points = mask_to_polygon(mask, simplify_epsilon=1.5)

        if not points:
            print(f"  ⚠ 轮廓提取失败: {cls_name} in {image_name}")
            continue

        shapes.append({
            "label": cls_name,
            "points": points,
            "group_id": None,
            "description": f"SAM auto (score={score:.3f})",
            "shape_type": "polygon",
            "flags": {},
        })

        print(f"    ✅ {cls_name}: {len(points)} 个顶点 (score={score:.3f})")

    # 如果没有成功生成任何 shape
    if not shapes:
        print(f"  ⚠ 没有生成任何有效标注")
        return None

    # 构建 Labelme JSON
    labelme_json = {
        "version": "5.5.0",
        "flags": {},
        "shapes": shapes,
        "imagePath": image_name,
        "imageData": None,
        "imageHeight": img_h,
        "imageWidth": img_w,
    }

    return labelme_json


def parse_yolo_label(label_path: str) -> list:
    """解析 YOLO 标签文件"""
    labels = []
    with open(label_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            cls_id = float(parts[0])
            xc = float(parts[1])
            yc = float(parts[2])
            w = float(parts[3])
            h = float(parts[4])
            labels.append((cls_id, xc, yc, w, h))
    return labels


def find_image_for_label(label_path: str, image_dir: str) -> str:
    """
    根据 label 文件名找到对应的图片
    label: 0001.txt → image: 0001.png / 0001.jpg / ...
    """
    stem = os.path.splitext(os.path.basename(label_path))[0]

    # 按类别命名的 label (如 0601.txt) 对应 0601.{ext}
    for ext in [".png", ".jpg", ".jpeg", ".PNG", ".JPG", ".JPEG"]:
        img_path = os.path.join(image_dir, stem + ext)
        if os.path.exists(img_path):
            return img_path

    # 如果找不到，尝试在 images/ 下全局搜索
    for ext in [".png", ".jpg", ".jpeg", ".PNG", ".JPG", ".JPEG"]:
        pattern = os.path.join(image_dir, stem + ext)
        if os.path.exists(pattern):
            return pattern

    return None


def print_summary(stats: dict):
    """打印统计信息"""
    print("\n" + "=" * 60)
    print("📊 处理统计")
    print("=" * 60)
    print(f"{'类别':<20} {'图片数':<8} {'标注数':<8} {'成功':<8} {'失败':<8}")
    print("-" * 60)
    total_imgs = total_boxes = total_ok = total_fail = 0
    for cls_id in sorted(stats.keys()):
        s = stats[cls_id]
        print(f"{CLASS_NAMES[cls_id]:<20} {s['images']:<8} {s['bboxes']:<8} {s['ok']:<8} {s['fail']:<8}")
        total_imgs += s['images']
        total_boxes += s['bboxes']
        total_ok += s['ok']
        total_fail += s['fail']
    print("-" * 60)
    print(f"{'合计':<20} {total_imgs:<8} {total_boxes:<8} {total_ok:<8} {total_fail:<8}")


def main():
    print("\n" + "=" * 60)
    print("🔬 SAM BBox → Labelme JSON 预标注工具")
    print("=" * 60)

    # ── 加载 SAM ──
    print("\n📥 加载 SAM 模型...")
    if not os.path.exists(SAM_CHECKPOINT):
        print(f"❌ 模型未找到: {SAM_CHECKPOINT}")
        print("   请先下载: wget -O models/sam_vit_b_01ec64.pth \\")
        print("     https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth")
        sys.exit(1)

    predictor, device = load_sam(SAM_CHECKPOINT)
    print(f"   ✅ SAM vit_b 加载成功\n")

    # ── 扫描标签文件 ──
    label_files = sorted([
        f for f in os.listdir(LABEL_DIR) if f.endswith(".txt")
    ])
    print(f"📂 找到 {len(label_files)} 个 YOLO 标签文件\n")

    # ── 创建输出目录 ──
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ── 逐文件处理 ──
    stats = defaultdict(lambda: {"images": 0, "bboxes": 0, "ok": 0, "fail": 0})
    success_count = 0
    skip_count = 0

    for i, label_file in enumerate(label_files):
        label_path = os.path.join(LABEL_DIR, label_file)

        # 找对应图片
        image_path = find_image_for_label(label_path, IMAGE_DIR)
        if not image_path:
            print(f"⏭ [{i+1}/{len(label_files)}] 跳过 {label_file}: 找不到对应图片")
            skip_count += 1
            continue

        # 解析标签
        labels = parse_yolo_label(label_path)
        if not labels:
            print(f"⏭ [{i+1}/{len(label_files)}] 跳过 {label_file}: 无有效标签")
            skip_count += 1
            continue

        print(f"\n🖼 [{i+1}/{len(label_files)}] {os.path.basename(image_path)} "
              f"({len(labels)} 个 BBox)")

        # 统计
        unique_classes = set()
        for l in labels:
            cls_id = int(l[0])
            stats[cls_id]["bboxes"] += 1
            unique_classes.add(cls_id)

        for cls_id in unique_classes:
            stats[cls_id]["images"] += 1

        # SAM 推理
        labelme_json = process_single_image(image_path, labels, predictor, device)

        if labelme_json is None:
            for cls_id in unique_classes:
                stats[cls_id]["fail"] += len([l for l in labels if int(l[0]) == cls_id])
            skip_count += 1
            continue

        # 保存 Labelme JSON
        json_name = os.path.splitext(os.path.basename(image_path))[0] + ".json"
        json_path = os.path.join(OUTPUT_DIR, json_name)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(labelme_json, f, ensure_ascii=False, indent=2)

        for cls_id in unique_classes:
            stats[cls_id]["ok"] += len([l for l in labels if int(l[0]) == cls_id])
        success_count += 1

        print(f"  💾 → {json_path}")

    # ── 结果 ──
    print_summary(stats)
    print(f"\n✅ 成功: {success_count} 张  |  ⏭ 跳过: {skip_count} 张")
    print(f"📁 输出目录: {OUTPUT_DIR}")
    print(f"\n💡 下一步:")
    print(f"   1. 在终端运行: labelme")
    print(f"   2. 打开目录: {OUTPUT_DIR}")
    print(f"   3. 逐张检查并修正 polygon 边缘")
    print(f"   4. 修正完成后保存 (Ctrl+S)")
    print(f"   5. 运行 python scripts/labelme2yoloseg.py 转换为 YOLO-seg 格式")


if __name__ == "__main__":
    main()
