#!/usr/bin/env python3
"""
合并数据源 → 5类 YOLOv8-seg 训练数据集
类别: 裂缝 渗漏水 异物入侵 管片破损掉块 管线支架松脱 (无扣件)
"""

import json, os, sys, shutil, random
from collections import defaultdict

USB = "/media/zt/A934-128D"
OUTPUT_DIR = os.path.join(USB, "5class_train")
VAL_RATIO = 0.2
SEED = 42

# 5类: 土建+设备 (不含轨道扣件)
CLASS_MAP = {
    "liefeng": 0,
    "shenloushui": 1, "water_leakage": 1,
    "yiwu": 2,
    "guanpianposun": 3,
    "guanxiansongtuo": 4,
}
NAMES = {0:"liefeng", 1:"shenloushui", 2:"yiwu", 3:"guanpianposun", 4:"guanxiansongtuo"}
CN = {0:"裂缝", 1:"渗漏水", 2:"异物入侵", 3:"管片破损掉块", 4:"管线支架松脱"}

# 数据源 (优先级从高到低) - 只保留5类相关
SOURCES = [
    ("病害分类数据/liefeng", "liefeng"),
    ("病害分类数据/shenloushui", "shenloushui"),
    ("病害分类数据/yiwu", "yiwu"),
    ("病害分类数据/guanpianposun", "guanpianposun"),
    ("病害分类数据/guanxiansongtuo", "guanxiansongtuo"),
    ("优质数据集/liefeng", "liefeng"),
    ("优质数据集/shenloushui", "shenloushui"),
    ("优质数据集/guanpianposun", "guanpianposun"),
    ("RFDD_classified/crack_liefeng", "liefeng"),
]

# 正常背景
NORMAL_SOURCES = [
    ("扣件模型数据/normal",),
    ("优质数据集/normal",),
]

random.seed(SEED)

def find_img(json_path, img_dir):
    stem = os.path.splitext(os.path.basename(json_path))[0]
    for ext in [".jpg",".jpeg",".png",".JPG",".JPEG",".PNG"]:
        c = os.path.join(img_dir, stem + ext)
        if os.path.exists(c) and os.path.getsize(c) > 0:
            return c
    try:
        d = json.load(open(json_path))
        ip = d.get("imagePath","")
        if ip:
            if not os.path.isabs(ip): ip = os.path.join(img_dir, ip)
            if os.path.exists(ip) and os.path.getsize(ip) > 0:
                return ip
    except: pass
    return None

def parse_json(jf):
    d = json.load(open(jf))
    w, h = d.get("imageWidth",0), d.get("imageHeight",0)
    shapes = []
    for s in d.get("shapes",[]):
        lbl = s.get("label","").strip()
        tp = s.get("shape_type","polygon")
        pts = s.get("points",[])
        if tp not in ("polygon","linestrip"): continue
        if lbl not in CLASS_MAP: continue
        if len(pts) < 3: continue
        shapes.append({"label":lbl, "points":pts})
    return w, h, shapes

def norm_pts(pts, w, h):
    r = []
    for x,y in pts:
        r.extend([max(0.0,min(1.0,x/max(w,1))), max(0.0,min(1.0,y/max(h,1)))])
    return r

def process_source(src_dir, seen, records):
    d = os.path.join(USB, src_dir)
    if not os.path.isdir(d): return 0,0
    added = skipped = errs = 0
    for jf in sorted(f for f in os.listdir(d) if f.endswith(".json")):
        jp = os.path.join(d, jf)
        stem = os.path.splitext(jf)[0]
        if stem in seen: skipped += 1; continue
        img = find_img(jp, d)
        if not img: errs += 1; continue
        try: w,h,shapes = parse_json(jp)
        except: errs += 1; continue
        if w<=0 or h<=0: errs += 1; continue
        seen.add(stem)
        records.append({"stem":stem,"img":img,"w":w,"h":h,"shapes":shapes,"source":src_dir})
        added += 1
    if added: print(f"  ✓ {src_dir}: +{added} (跳过{skipped}, 错误{errs})")
    return added, skipped

def process_normal(src_dir, seen, records):
    d = os.path.join(USB, src_dir)
    if not os.path.isdir(d): return 0,0
    added = skipped = 0
    for jf in sorted(f for f in os.listdir(d) if f.endswith(".json")):
        stem = os.path.splitext(jf)[0]
        if stem in seen: skipped += 1; continue
        jp = os.path.join(d, jf)
        img = find_img(jp, d)
        if not img: continue
        seen.add(stem)
        records.append({"stem":stem,"img":img})
        added += 1
    if added: print(f"  ✓ {src_dir} (normal): +{added} (跳过{skipped})")
    return added, skipped

def main():
    print("=" * 60)
    print("  5类病害 (土建+设备) YOLOv8-seg 数据集")
    print("=" * 60)

    seen = set()
    records = []
    normal_records = []

    print("\n📂 有标注数据源...")
    for src_dir, _ in SOURCES:
        process_source(src_dir, seen, records)

    print("\n📂 正常背景...")
    ns = set()
    for (src_dir,) in NORMAL_SOURCES:
        process_normal(src_dir, ns, normal_records)

    print(f"\n📊 去重后: {len(records)} 标注图 + {len(normal_records)} 正常图")

    if len(records) == 0:
        print("❌ 无有效标注!"); sys.exit(1)

    # 类别统计
    cc = defaultdict(int)
    for r in records:
        for s in r["shapes"]:
            cc[CLASS_MAP[s["label"]]] += 1

    print(f"\n📊 实例数:")
    for i in range(5):
        print(f"   {i} {NAMES[i]:<18} {CN[i]:<10} {cc[i]:>6}")

    # 分层划分
    sample_cls = []
    for r in records:
        sc = set()
        for s in r["shapes"]:
            sc.add(CLASS_MAP[s["label"]])
        sample_cls.append(sc)

    val_idx = set()
    cv = {i:0 for i in range(5)}
    idxs = list(range(len(records)))
    random.shuffle(idxs)
    for i in idxs:
        for c in sample_cls[i]:
            if cv[c] < 1: val_idx.add(i); cv[c] += 1

    remaining = [i for i in range(len(records)) if i not in val_idx]
    random.shuffle(remaining)
    nv = max(0, int(len(records)*VAL_RATIO) - len(val_idx))
    val_idx.update(remaining[:nv])
    train_idx = [i for i in range(len(records)) if i not in val_idx]

    print(f"\n📊 划分: train={len(train_idx)}, val={len(val_idx)}")

    # 输出
    for sub in ["images/train","images/val","labels/train","labels/val"]:
        os.makedirs(os.path.join(OUTPUT_DIR, sub), exist_ok=True)

    stats = {"train":defaultdict(int),"val":defaultdict(int)}
    img_err = 0

    for split, idx_set in [("train",train_idx),("val",sorted(val_idx))]:
        ld = os.path.join(OUTPUT_DIR, f"labels/{split}")
        idir = os.path.join(OUTPUT_DIR, f"images/{split}")
        for i in idx_set:
            r = records[i]
            lines = []
            for s in r["shapes"]:
                cid = CLASS_MAP[s["label"]]
                pts = norm_pts(s["points"], r["w"], r["h"])
                lines.append(f"{cid} " + " ".join(f"{v:.6f}" for v in pts))
                stats[split][cid] += 1
            with open(os.path.join(ld, f"{r['stem']}.txt"),"w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))
            ext = os.path.splitext(r["img"])[1] or ".jpg"
            dst = os.path.join(idir, f"{r['stem']}{ext}")
            if not os.path.exists(dst):
                try: shutil.copy2(r["img"], dst)
                except: img_err += 1

    # 正常样本
    nidx = list(range(len(normal_records)))
    random.shuffle(nidx)
    nv = max(1, int(len(nidx)*VAL_RATIO))
    n_val = set(nidx[:nv]); n_train = set(nidx[nv:])
    for split, idx_set in [("train",n_train),("val",n_val)]:
        ld = os.path.join(OUTPUT_DIR, f"labels/{split}")
        idir = os.path.join(OUTPUT_DIR, f"images/{split}")
        for i in idx_set:
            r = normal_records[i]
            open(os.path.join(ld, f"{r['stem']}.txt"),"w").close()
            ext = os.path.splitext(r["img"])[1] or ".jpg"
            dst = os.path.join(idir, f"{r['stem']}{ext}")
            if not os.path.exists(dst):
                try: shutil.copy2(r["img"], dst)
                except: pass

    print(f"   正常: train={len(n_train)}, val={len(n_val)}")

    # data.yaml
    yaml = f"""# 地铁5类病害 (土建+设备)
path: {OUTPUT_DIR}
train: images/train
val: images/val
nc: 5
names:
  0: liefeng          # 裂缝
  1: shenloushui      # 渗漏水
  2: yiwu             # 异物入侵
  3: guanpianposun    # 管片破损掉块
  4: guanxiansongtuo  # 管线支架松脱
"""
    with open(os.path.join(OUTPUT_DIR,"data.yaml"),"w") as f: f.write(yaml)

    print("\n" + "=" * 60)
    print("✅ 完成!")
    print(f"{'ID':<4} {'类别':<18} {'中文':<12} {'训练':<8} {'验证':<8} {'合计':<8}")
    print("-" * 60)
    for i in range(5):
        t=stats["train"].get(i,0); v=stats["val"].get(i,0)
        print(f"{i:<4} {NAMES[i]:<18} {CN[i]:<12} {t:<8} {v:<8} {t+v:<8}")
    print(f"\n训练: {len(train_idx)} 病害 + {len(n_train)} 正常 = {len(train_idx)+len(n_train)}")
    print(f"验证: {len(val_idx)} 病害 + {len(n_val)} 正常 = {len(val_idx)+len(n_val)}")
    print(f"总计: {len(records)+len(normal_records)} 张")

    # 验证
    for split in ["train","val"]:
        imgs = set(os.path.splitext(f)[0] for f in os.listdir(os.path.join(OUTPUT_DIR,f"images/{split}")) if f.lower().endswith((".jpg",".jpeg",".png")))
        lbls = set(os.path.splitext(f)[0] for f in os.listdir(os.path.join(OUTPUT_DIR,f"labels/{split}")) if f.endswith(".txt"))
        ml = imgs-lbls; mi = lbls-imgs
        print(f"  {split}: {'✅' if not ml and not mi else '⚠'+str(len(ml))+'/'+str(len(mi))} {len(imgs)}↔{len(lbls)}")
    if img_err: print(f"\n⚠ 图片失败: {img_err}")

if __name__ == "__main__":
    main()
