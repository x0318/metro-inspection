#!/usr/bin/env python3
"""
铁路障碍物数据集 BBox → SAM polygon → Labelme JSON
用户可在 Labelme 中精修，无需从头描边。

用法:
  python scripts/sam_preseg_yiwu.py
"""

import os, sys, json, cv2, torch, shutil, numpy as np
from collections import defaultdict
from segment_anything import sam_model_registry, SamPredictor

BASE = "/home/zt/subway_project"
SAM_CHECKPOINT = f"{BASE}/models/sam_vit_b_01ec64.pth"
SRC = "/home/zt/文档/xwechat_files/wxid_1qqtbfin4t6p22_9395/msg/file/2026-07/铁路障碍物数据集/铁路障碍物数据集-解压后可直接使用/augmented_20260310_185342"
IMG_SRC = f"{BASE}/data/for_annotation/yiwu"
OUT_DIR = f"{BASE}/data/for_annotation/yiwu_labelme"

# 全部映射为 yiwu (class 3)
CLASS_MAP = {0: 3, 1: 3, 2: 3, 3: 3}  # fallen-tree, generic-rock, generic-tree, rock

os.makedirs(OUT_DIR, exist_ok=True)


def mask_to_polygon(mask, epsilon=1.5):
    contours, _ = cv2.findContours(
        (mask.astype(np.uint8) * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    if len(best) < 3:
        return None
    approx = cv2.approxPolyDP(best, epsilon, True)
    pts = approx.squeeze(axis=1)
    if pts.ndim != 2 or len(pts) < 3:
        return None
    return pts.tolist()


def parse_label(label_path):
    """解析 YOLO 标签，返回 [(cls_id, coords), ...]，coords 可能是 BBox(4) 或 polygon(N)"""
    items = []
    if not os.path.exists(label_path):
        return items
    with open(label_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            cls_id = int(parts[0])
            coords = [float(x) for x in parts[1:]]
            items.append((cls_id, coords))
    return items


def coords_to_bbox(coords):
    """YOLO 坐标 → 像素 BBox"""
    if len(coords) == 4:
        # BBox: cx, cy, w, h (归一化)
        xc, yc, bw, bh = coords
        return [xc - bw/2, yc - bh/2, xc + bw/2, yc + bh/2]
    else:
        # Polygon → 计算包围盒
        xs = coords[::2]
        ys = coords[1::2]
        return [min(xs), min(ys), max(xs), max(ys)]


def clean_name(filename):
    """去掉增强后缀，得到原始帧名"""
    name = filename.replace('_aug_0', '').replace('_aug_1', '').replace('_aug_2', '')
    if '.rf.' in name:
        name = name[:name.index('.rf.')] + os.path.splitext(name)[1]
    return name


def build_index():
    """构建 clean_name → label_path 映射"""
    index = {}
    for split in ['train', 'valid', 'test']:
        lbl_dir = f"{SRC}/{split}/labels"
        if not os.path.exists(lbl_dir):
            continue
        for lbl_file in os.listdir(lbl_dir):
            if not lbl_file.endswith('.txt'):
                continue
            clean = clean_name(lbl_file)
            index[clean] = os.path.join(lbl_dir, lbl_file)
    return index


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"📥 加载 SAM vit_b → {device}")
    sam = sam_model_registry["vit_b"](checkpoint=SAM_CHECKPOINT)
    sam.to(device=device)
    sam.eval()
    predictor = SamPredictor(sam)
    print("✅ SAM 就绪")

    # 构建标签索引
    index = build_index()
    print(f"📂 标签索引: {len(index)} 个文件")

    # 处理 yiwu 目录中的图片
    img_files = sorted([
        f for f in os.listdir(IMG_SRC)
        if f.lower().endswith((".jpg", ".jpeg", ".png"))
    ])
    print(f"🖼 共 {len(img_files)} 张图片\n")

    total_masks = 0
    for i, img_file in enumerate(img_files):
        img_path = os.path.join(IMG_SRC, img_file)

        # 找原始标签: clean_name 匹配
        base = os.path.splitext(img_file.replace('railway_', ''))[0] + '.txt'
        src_label = index.get(base)
        if not src_label:
            # 尝试在 index 中模糊匹配
            for k, v in index.items():
                if os.path.splitext(k)[0] == os.path.splitext(base)[0]:
                    src_label = v
                    break

        # 读图片
        img = cv2.imread(img_path)
        if img is None:
            continue
        h, w = img.shape[:2]
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        shapes = []

        if src_label:
            items = parse_label(src_label)
            if items:
                predictor.set_image(img_rgb)

            for cls_id, coords in items:
                if cls_id not in CLASS_MAP:
                    continue

                bbox_norm = coords_to_bbox(coords)
                # 像素坐标 BBox
                x1 = int(bbox_norm[0] * w)
                y1 = int(bbox_norm[1] * h)
                x2 = int(bbox_norm[2] * w)
                y2 = int(bbox_norm[3] * h)
                # 加 10% margin
                mx = int((x2 - x1) * 0.1)
                my = int((y2 - y1) * 0.1)
                box = np.array([[
                    max(0, x1 - mx), max(0, y1 - my),
                    min(w, x2 + mx), min(h, y2 + my)
                ]])

                masks, scores, _ = predictor.predict(box=box, multimask_output=False)

                if masks is None or len(masks) == 0 or scores[0] < 0.3:
                    # SAM 失败，fallback 到 BBox 4 角点
                    poly = [
                        [float(bbox_norm[0] * w), float(bbox_norm[1] * h)],
                        [float(bbox_norm[2] * w), float(bbox_norm[1] * h)],
                        [float(bbox_norm[2] * w), float(bbox_norm[3] * h)],
                        [float(bbox_norm[0] * w), float(bbox_norm[3] * h)],
                    ]
                else:
                    poly = mask_to_polygon(masks[0])
                    if poly is None:
                        poly = [
                            [float(bbox_norm[0] * w), float(bbox_norm[1] * h)],
                            [float(bbox_norm[2] * w), float(bbox_norm[1] * h)],
                            [float(bbox_norm[2] * w), float(bbox_norm[3] * h)],
                            [float(bbox_norm[0] * w), float(bbox_norm[3] * h)],
                        ]

                shapes.append({
                    "label": "yiwu",
                    "points": [[float(x), float(y)] for x, y in poly],
                    "group_id": None,
                    "shape_type": "polygon",
                    "flags": {},
                })
                total_masks += 1

        # 写 Labelme JSON
        stem = os.path.splitext(img_file)[0]
        json_path = os.path.join(OUT_DIR, f"{stem}.json")

        labelme_json = {
            "version": "5.0.1",
            "flags": {},
            "shapes": shapes,
            "imagePath": img_file,
            "imageData": None,
            "imageHeight": h,
            "imageWidth": w,
        }

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(labelme_json, f, ensure_ascii=False, indent=2)

        # 复制图片
        dst_img = os.path.join(OUT_DIR, img_file)
        if not os.path.exists(dst_img):
            shutil.copy2(img_path, dst_img)

        if (i + 1) % 10 == 0 or i == len(img_files) - 1:
            print(f"  [{i+1}/{len(img_files)}] {img_file}: {len(shapes)} instances")

    print(f"\n✅ 完成: {total_masks} 个实例")
    print(f"📁 输出: {OUT_DIR}/")
    print(f"\n💡 用 Labelme 打开精修:")
    print(f"   conda activate subway_label && labelme {OUT_DIR}/")


if __name__ == "__main__":
    main()
