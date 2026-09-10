#!/usr/bin/env python3
"""
YOLOv8-seg 5类病害训练 — 重点优化检出率 + 分类正确度

用法:
  python3 scripts/train_seg.py                          # 默认 yolov8l-seg
  python3 scripts/train_seg.py --model yolov8m-seg.pt   # 换模型
"""

import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="YOLOv8-seg 5类病害训练 (分类优先)")
    parser.add_argument("--data", default="/media/zt/A934-128D/5class_yolo/data.yaml")
    parser.add_argument("--model", default="yolov8l-seg.pt",
                        help="yolov8n/s/m/l/x-seg.pt")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--project", default="runs/5class_seg")
    parser.add_argument("--name", default="train3")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    model = YOLO(args.model)

    results = model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        workers=args.workers,
        project=args.project,
        name=args.name,
        resume=args.resume,

        # ── 损失权重: 分类 > 检测 > 分割 ──
        cls=1.2,           # 提高分类损失 (默认0.5)
        box=7.5,           # bbox 回归
        dfl=1.5,           # DFL
        # seg 用默认

        # ── 数据增强 (加大, 弥补小类别) ──
        hsv_h=0.03,        # 色调
        hsv_s=0.5,         # 饱和度
        hsv_v=0.3,         # 明度
        degrees=20,        # 旋转 ±20°
        translate=0.15,    # 平移
        scale=0.4,         # 缩放 ±40%
        shear=5,
        flipud=0.5,        # 上下翻转 (隧道向上/向下)
        fliplr=0.5,        # 左右翻转
        mosaic=0.8,        # Mosaic 增强 (提高)
        mixup=0.15,        # MixUp
        copy_paste=0.15,   # Copy-Paste (小类别受益)
        close_mosaic=15,   # 最后15轮关闭mosaic

        # ── 优化器 ──
        optimizer="AdamW",
        lr0=1e-3,
        lrf=1e-2,
        momentum=0.9,
        weight_decay=5e-4,
        warmup_epochs=3,
        cos_lr=True,

        # ── 其他 ──
        patience=50,        # 早停放宽
        label_smoothing=0.1,
        dropout=0.1,        # 正则化防过拟合
        save=True,
        save_period=-1,       # 不存中间 epoch checkpoint, 只留 best/last (防磁盘爆)
        exist_ok=True,
        seed=42,
        pretrained=True,
    )

    # 验证
    print("\n📊 验证最佳模型...")
    metrics = model.val()

    print(f"\n=== 检测+分类 (Box) ===")
    print(f"   mAP@50:    {metrics.box.map50:.4f}")
    print(f"   mAP@50-95:  {metrics.box.map:.4f}")
    print(f"   P: {metrics.box.p:.4f}  R: {metrics.box.r:.4f}")
    print(f"\n=== 分割 (Mask) ===")
    print(f"   mAP@50:    {metrics.seg.map50:.4f}")
    print(f"   mAP@50-95:  {metrics.seg.map:.4f}")

    print(f"\n✅ 模型: {args.project}/{args.name}/weights/best.pt")


if __name__ == "__main__":
    main()
