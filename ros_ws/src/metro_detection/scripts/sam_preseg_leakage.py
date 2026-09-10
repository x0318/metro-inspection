#!/usr/bin/env python3
"""
对渗漏水图片用 SAM 自动生成 pre-annotations → Labelme JSON
用户可在 Labelme 中删改/精修，无需从头描边。

用法:
  python scripts/sam_preseg_leakage.py
"""

import os, sys, json, cv2, torch, shutil, base64
import numpy as np
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

BASE = "/home/zt/subway_project"
SAM_CHECKPOINT = f"{BASE}/models/sam_vit_b_01ec64.pth"
IMG_DIR = f"{BASE}/data/for_annotation/shenloushui"
OUT_DIR = f"{BASE}/data/for_annotation/shenloushui_labelme"

os.makedirs(OUT_DIR, exist_ok=True)


def mask_to_polygon(mask, epsilon=2.0):
    """SAM mask → polygon 顶点列表"""
    contours, _ = cv2.findContours(
        (mask.astype(np.uint8) * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
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


def main():
    # 加载 SAM
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"📥 加载 SAM vit_b → {device}")
    sam = sam_model_registry["vit_b"](checkpoint=SAM_CHECKPOINT)
    sam.to(device=device)
    sam.eval()

    # 自动掩码生成器
    mask_generator = SamAutomaticMaskGenerator(
        model=sam,
        points_per_side=32,
        pred_iou_thresh=0.88,       # 高阈值: 只保留置信度高的
        stability_score_thresh=0.92,
        min_mask_region_area=200,    # 最小区域面积 (像素)
        crop_n_layers=1,
        crop_n_points_downscale_factor=2,
    )

    img_files = sorted([
        f for f in os.listdir(IMG_DIR)
        if f.lower().endswith((".jpg", ".jpeg", ".png"))
    ])

    print(f"🖼 共 {len(img_files)} 张图片\n")

    total_masks = 0
    for i, img_file in enumerate(img_files):
        img_path = os.path.join(IMG_DIR, img_file)
        img = cv2.imread(img_path)
        if img is None:
            print(f"  ⚠ 跳过: {img_file}")
            continue
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = img.shape[:2]

        # SAM 自动生成掩码
        masks = mask_generator.generate(img_rgb)

        # 过滤: 面积占比 < 80% (全图掩码通常是噪声), 且 > 0.1%
        shapes = []
        for m in masks:
            area_ratio = m["area"] / (h * w)
            if area_ratio > 0.8:
                continue
            if area_ratio < 0.001:
                continue
            poly = mask_to_polygon(m["segmentation"])
            if poly is None or len(poly) < 3:
                continue
            # SAM 自动模式无类别，统一标为 shenloushui (class 7)
            shapes.append({
                "label": "shenloushui",
                "points": [[float(x), float(y)] for x, y in poly],
                "group_id": None,
                "shape_type": "polygon",
                "flags": {},
            })

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

        # 复制图片到输出目录
        dst_img = os.path.join(OUT_DIR, img_file)
        if not os.path.exists(dst_img):
            shutil.copy2(img_path, dst_img)

        total_masks += len(shapes)
        if (i + 1) % 10 == 0 or i == len(img_files) - 1:
            print(f"  [{i+1}/{len(img_files)}] {img_file}: {len(shapes)} masks")

    print(f"\n✅ 完成: {total_masks} 个候选掩码")
    print(f"📁 输出: {OUT_DIR}/")
    print(f"\n💡 用 Labelme 打开精修:")
    print(f"   conda activate subway_label && labelme {OUT_DIR}/")


if __name__ == "__main__":
    main()
