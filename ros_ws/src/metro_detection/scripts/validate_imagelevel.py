#!/usr/bin/env python3
"""
对训练数据集的 val 集做「图像级检出率」验证 (与官方赛制"离线检出率"同口径)。

用法:
  python3 validate_imagelevel.py --data /media/zt/A934-128D/5class_yolo \
      --weights runs/.../best.pt
  python3 validate_imagelevel.py --data /media/zt/A934-128D/subway_data/fastener_v6 \
      --weights /media/zt/A934-128D/subway_runs/segment/fastener_v6_wiou_tuned/weights/best.pt --gray

口径:
  图像级检出率 = 含某类病害的图中, 正确检出该类的图占比
  实例级召回   = 命中目标数 / 真值目标数 (参考)

匹配: 同类别 BBox IoU ≥ 0.5 (由 seg 多边形取外接框)。
"""
import argparse
import os
import yaml
import numpy as np
import cv2
from PIL import Image
from collections import defaultdict
from ultralytics import YOLO
import torch

NAME2DOMAIN = {
    "liefeng": "土建", "shenloushui": "土建", "guanpianposun": "土建",
    "yiwu": "设备", "guanxiansongtuo": "设备",
    "koujianwaixie": "轨道", "koujianduanlie": "轨道", "koujianqueshi": "轨道",
}
DOMAIN_TARGET = {"土建": 0.95, "轨道": 0.95, "设备": 0.97}
CN = {"liefeng": "裂缝", "shenloushui": "渗漏水", "guanpianposun": "管片破损",
      "yiwu": "异物入侵", "guanxiansongtuo": "管线松脱",
      "koujianwaixie": "扣件松动歪斜", "koujianduanlie": "扣件断裂", "koujianqueshi": "扣件缺失"}


def iou(a, b):
    x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
    x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
    inter = max(0., x2 - x1) * max(0., y2 - y1)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / (ua + 1e-9)


def load_names(data_dir):
    y = yaml.safe_load(open(os.path.join(data_dir, "data.yaml")))
    names = y["names"]
    if isinstance(names, dict):
        return {int(k): v for k, v in names.items()}
    return {i: v for i, v in enumerate(names)}


def read_gt(lbl_path):
    """读 YOLO-seg label, 返回 [(class_id, [x1,y1,x2,y2] 归一化外接框)]。"""
    gts = []
    for line in open(lbl_path):
        line = line.strip()
        if not line:
            continue
        p = line.split()
        c = int(p[0])
        coords = list(map(float, p[1:]))
        xs = coords[0::2]; ys = coords[1::2]
        if not xs:
            continue
        gts.append((c, [min(xs), min(ys), max(xs), max(ys)]))
    return gts


def predict_boxes(model, img_path, gray, device):
    if gray:
        im = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        r = model.predict(im, imgsz=640, conf=0.001, device=device, verbose=False)[0]
    else:
        r = model.predict(img_path, imgsz=640, conf=0.001, device=device, verbose=False)[0]
    boxes = r.boxes
    pd = []
    if boxes is not None and len(boxes) > 0:
        for xyxy, cl, cf in zip(boxes.xyxy.cpu().numpy(),
                                boxes.cls.cpu().numpy().astype(int),
                                boxes.conf.cpu().numpy()):
            pd.append((int(cl), xyxy.tolist(), float(cf)))
    return pd


def match(gts, preds, conf_thr, img_w, img_h):
    """gts 归一化, preds 像素坐标 → 归一化后匹配。返回命中数。"""
    # 预测框归一化
    preds_n = []
    for c, b, cf in preds:
        if cf < conf_thr:
            continue
        x1, y1, x2, y2 = b
        preds_n.append((c, [x1/img_w, y1/img_h, x2/img_w, y2/img_h]))
    used = [False] * len(preds_n)
    hit = 0
    for c, gb in gts:
        best_i, best_iou = -1, 0.0
        for i, (pc, pb) in enumerate(preds_n):
            if used[i] or pc != c:
                continue
            v = iou(gb, pb)
            if v > best_iou:
                best_iou, best_i = v, i
        if best_i >= 0 and best_iou >= 0.5:
            used[best_i] = True
            hit += 1
    return hit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--gray", action="store_true", help="灰度推理 (扣件模型)")
    ap.add_argument("--conf", type=float, default=0.001, help="匹配置信度阈值")
    args = ap.parse_args()

    names = load_names(args.data)
    device = 0 if torch.cuda.is_available() else "cpu"
    model = YOLO(args.weights)

    val_dir = os.path.join(args.data, "images", "val")
    lbl_dir = os.path.join(args.data, "labels", "val")
    imgs = sorted(os.listdir(val_dir))

    # 防御性剔除与训练集重叠的泄漏图 (train/val 划分本应互斥)
    train_dir = os.path.join(args.data, "images", "train")
    if os.path.isdir(train_dir):
        train_names = set(os.listdir(train_dir))
        imgs_clean = [f for f in imgs if f not in train_names]
        leaked = len(imgs) - len(imgs_clean)
        if leaked:
            print(f"[提示] 已排除 {leaked} 张与训练集重叠的泄漏图")
        imgs = imgs_clean

    # 类别名 → 类别 id 反转
    name2id = {v: k for k, v in names.items()}
    # 类别 id → 领域
    id2domain = {name2id[n]: d for n, d in NAME2DOMAIN.items() if n in name2id}

    stats = defaultdict(lambda: {"inst_hit": 0, "inst_gt": 0, "img_hit": 0, "img_gt": 0})

    for fname in imgs:
        img_path = os.path.join(val_dir, fname)
        stem = os.path.splitext(fname)[0]
        lbl_path = os.path.join(lbl_dir, stem + ".txt")
        if not os.path.exists(lbl_path):
            continue
        with Image.open(img_path) as im:
            img_w, img_h = im.size
        gts = read_gt(lbl_path)
        if not gts:
            continue
        pd = predict_boxes(model, img_path, args.gray, device)
        hit = match(gts, pd, args.conf, img_w, img_h)

        # 图像级: 该图每个类别是否检出
        gt_classes = set(c for c, _ in gts)
        hit_classes = set()
        # 逐类命中数
        cls_hit = defaultdict(int)
        # 重新逐类匹配 (用全局 match 的结果不够, 需按类别统计命中实例)
        # 简化: 用 match 返回的总命中数分配不了类别, 这里逐类重算
        used_global = [False] * 0
        # 逐类独立匹配
        preds_n = []
        for c, b, cf in pd:
            if cf < args.conf:
                continue
            x1, y1, x2, y2 = b
            preds_n.append((c, [x1/img_w, y1/img_h, x2/img_w, y2/img_h]))
        for c in gt_classes:
            gts_c = [gb for gc, gb in gts if gc == c]
            used = [False] * len(preds_n)
            ch = 0
            for gb in gts_c:
                bi, biou = -1, 0.0
                for i, (pc, pb) in enumerate(preds_n):
                    if used[i] or pc != c:
                        continue
                    v = iou(gb, pb)
                    if v > biou:
                        biou, bi = v, i
                if bi >= 0 and biou >= 0.5:
                    used[bi] = True
                    ch += 1
            stats[c]["inst_gt"] += len(gts_c)
            stats[c]["inst_hit"] += ch
            if ch >= 1:
                hit_classes.add(c)

        for c in gt_classes:
            stats[c]["img_gt"] += 1
        for c in hit_classes:
            stats[c]["img_hit"] += 1

    # 输出
    print(f"\n{'='*76}")
    print(f"图像级检出率  (conf={args.conf}, gray={args.gray})")
    print(f"数据: {args.data}  (val {len(imgs)} 张)")
    print(f"{'='*76}")
    print(f"  {'类别':<12}{'检出图/总图':>14}{'图像检出率':>11}{'实例命中/总数':>14}{'实例召回':>10}")
    for c in sorted(stats):
        if stats[c]["img_gt"] == 0:
            continue
        n = names[c]
        ir = stats[c]["img_hit"] / stats[c]["img_gt"]
        rr = stats[c]["inst_hit"] / stats[c]["inst_gt"]
        print(f"  {CN.get(n, n):<12}{stats[c]['img_hit']:>5}/{stats[c]['img_gt']:<7}{ir:>11.3f}"
              f"{stats[c]['inst_hit']:>6}/{stats[c]['inst_gt']:<6}{rr:>10.3f}")

    print(f"\n  {'─'*60}")
    for d in ["土建", "轨道", "设备"]:
        cs = [c for c in id2domain if id2domain[c] == d]
        if not cs:
            continue
        ih = sum(stats[c]["img_hit"] for c in cs)
        ig = sum(stats[c]["img_gt"] for c in cs)
        ph = sum(stats[c]["inst_hit"] for c in cs)
        pg = sum(stats[c]["inst_gt"] for c in cs)
        if ig == 0:
            continue
        ir = ih / ig; rr = ph / pg
        mark = "✓" if ir >= DOMAIN_TARGET[d] else "✗"
        print(f"  领域[{d}] 图像检出 {ih}/{ig} = {ir:.3f}  (实例召回 {rr:.3f})  目标≥{DOMAIN_TARGET[d]:.2f} {mark}")


if __name__ == "__main__":
    main()
