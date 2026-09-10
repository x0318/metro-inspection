"""
向训练集注入多样化背景图，降低背景→病害误报。

背景来源: 病害训练集+正常状态/正常状态/{轨道,设备,土建} (60张官方正常图)
策略:
  - 小图 (< 1280px): resize → 640×640
  - 大图 (≥ 1280px): 随机裁剪多个 640×640 区块

输出: 将 crops 写入 data/fastener_cropped/images/train/ + 空标签
用法:
  python3 scripts/add_backgrounds.py                     # 默认参数
  python3 scripts/add_backgrounds.py --crops_per_large 20 --dry-run
"""

import cv2
import os, sys, random
from pathlib import Path
from argparse import ArgumentParser

# ─── 配置 ────────────────────────────────────────────
SOURCE_DIR   = Path("/home/zt/subway_project/病害训练集+正常状态/正常状态")
TARGET_DIR   = Path("/home/zt/subway_project/data/fastener_cropped")
CROP_SIZE    = 640          # 与训练 imgsz 一致
CROPS_PER_LARGE = 15        # 大图 (≥1280 任一边) 每张裁剪数
CROPS_PER_SMALL = 3         # 小图每张裁剪数
LARGE_THRESHOLD = 1280      # 判定大图的阈值
PREFIX       = "bg_"        # 输出文件名前缀
SEED         = 42


def extract_crops(img: "np.ndarray", n: int) -> list:
    """从图像中随机提取 n 个 640×640 区块"""
    h, w = img.shape[:2]
    crops = []
    rng = random.Random(SEED + hash(str(img.shape)))  # 确定性随机

    attempts = 0
    while len(crops) < n and attempts < n * 5:
        attempts += 1
        if w <= CROP_SIZE:
            x = 0
        else:
            x = rng.randint(0, w - CROP_SIZE)
        if h <= CROP_SIZE:
            y = 0
        else:
            y = rng.randint(0, h - CROP_SIZE)

        crop = img[y:y + CROP_SIZE, x:x + CROP_SIZE]

        # 如果任一边不足 640，pad 到 640
        if crop.shape[0] < CROP_SIZE or crop.shape[1] < CROP_SIZE:
            pad_h = max(0, CROP_SIZE - crop.shape[0])
            pad_w = max(0, CROP_SIZE - crop.shape[1])
            crop = cv2.copyMakeBorder(crop, 0, pad_h, 0, pad_w,
                                       cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # 跳过纯色 / 过暗的 crop（可能是边框或无效区域）
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        std = gray.std()
        if std < 10:
            continue  # 太均匀，跳过

        crops.append(crop)

    return crops


def add_backgrounds(source_dir: Path, target_dir: Path,
                    crops_per_large: int, crops_per_small: int,
                    dry_run: bool = False):
    """主逻辑"""

    img_train_dir = target_dir / "images" / "train"
    lbl_train_dir = target_dir / "labels" / "train"

    if not img_train_dir.exists():
        print(f"ERROR: {img_train_dir} 不存在，先运行 train_fastener.py --convert-only")
        return

    # 收集源图
    source_images = []
    for sub in sorted(source_dir.iterdir()):
        if not sub.is_dir():
            continue
        for ext in ["*.png", "*.jpg", "*.jpeg", "*.PNG", "*.JPG", "*.JPEG"]:
            source_images.extend(sub.glob(ext))

    print(f"{'='*60}")
    print(f"背景注入工具")
    print(f"{'='*60}")
    print(f"源目录: {source_dir}")
    print(f"目标:   {img_train_dir}")
    print(f"大图裁剪数: {crops_per_large}  (≥{LARGE_THRESHOLD}px)")
    print(f"小图裁剪数: {crops_per_small}  (<{LARGE_THRESHOLD}px)")
    print(f"共找到 {len(source_images)} 张源图")
    print(f"{'[DRY RUN] ' if dry_run else ''}")

    if not source_images:
        print("ERROR: 未找到源图")
        return

    added = 0
    large_count, small_count, skipped = 0, 0, 0

    for i, src_path in enumerate(source_images):
        img = cv2.imread(str(src_path))
        if img is None:
            print(f"  [跳过] 无法读取: {src_path.name}")
            skipped += 1
            continue

        h, w = img.shape[:2]
        is_large = w >= LARGE_THRESHOLD or h >= LARGE_THRESHOLD
        n_crops = crops_per_large if is_large else crops_per_small

        if is_large:
            large_count += 1
        else:
            small_count += 1

        crops = extract_crops(img, n_crops)

        sub_name = src_path.parent.name  # 轨道/设备/土建
        base_stem = src_path.stem

        for j, crop in enumerate(crops):
            out_name = f"{PREFIX}{sub_name}_{base_stem}_{j:02d}.jpg"
            out_img = img_train_dir / out_name
            out_lbl = lbl_train_dir / (out_name.rsplit(".", 1)[0] + ".txt")

            if dry_run:
                continue

            cv2.imwrite(str(out_img), crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
            # 空标签 = 背景
            out_lbl.write_text("", encoding="utf-8")
            added += 1

        if (i + 1) % 20 == 0:
            print(f"  处理中... {i+1}/{len(source_images)} (已添加 {added} crops)")

    # 统计
    print(f"\n{'='*60}")
    print(f"完成{'[DRY RUN]' if dry_run else ''}")
    print(f"{'='*60}")
    print(f"  大图: {large_count} 张 × ~{crops_per_large} crops")
    print(f"  小图: {small_count} 张 × ~{crops_per_small} crops")
    print(f"  跳过: {skipped} 张 (无法读取)")
    print(f"  共注入: {added} 张背景图")

    if not dry_run and added > 0:
        # 统计更新后的训练集
        total_imgs = len(list(img_train_dir.glob("*")))
        bg_count = sum(1 for f in lbl_train_dir.glob("*.txt")
                       if f.stat().st_size == 0)
        bg_ratio = bg_count / total_imgs * 100 if total_imgs > 0 else 0
        print(f"\n  更新后训练集: {total_imgs} 张, "
              f"背景 {bg_count} 张 ({bg_ratio:.1f}%)")

        # 清除旧的 label cache (YOLO 会在训练时自动重建)
        for cache_file in lbl_train_dir.parent.glob("*.cache"):
            cache_file.unlink()
            print(f"  已清除缓存: {cache_file.name}")

    return added


if __name__ == "__main__":
    parser = ArgumentParser(description="向训练集注入多样化背景图")
    parser.add_argument("--source", type=str, default=str(SOURCE_DIR))
    parser.add_argument("--target", type=str, default=str(TARGET_DIR))
    parser.add_argument("--crops_per_large", type=int, default=CROPS_PER_LARGE)
    parser.add_argument("--crops_per_small", type=int, default=CROPS_PER_SMALL)
    parser.add_argument("--dry-run", action="store_true",
                        help="仅统计，不实际写入")
    args = parser.parse_args()

    add_backgrounds(
        source_dir=Path(args.source),
        target_dir=Path(args.target),
        crops_per_large=args.crops_per_large,
        crops_per_small=args.crops_per_small,
        dry_run=args.dry_run,
    )
