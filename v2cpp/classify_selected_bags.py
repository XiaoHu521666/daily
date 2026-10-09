#!/usr/bin/env python3
"""按筛选表第一列，将同名数据包目录移到源目录下的汇总文件夹。"""

import argparse
import csv
import os
import shutil
from pathlib import Path


# 后续换表或目录，只需修改下面三个路径。
# Windows 下填写 r"\\服务器\共享目录\..."；WSL 下填写对应的 /mnt/... 路径。
CSV_PATH = Path(__file__).parent / "新版防撞统计 - 筛选表.csv"
SOURCE_DIR = Path(
    r"\\10.0.0.110\Data_Storage\研发\HIT\大车防撞\655_2026_09_11"
    if os.name == "nt"
    else "/mnt/data_storage/研发/HIT/大车防撞/655_2026_09_11"
)
TARGET_DIR = SOURCE_DIR / "筛选表数据"


def read_names(csv_path):
    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        next(reader, None)  # 跳过表头
        names = dict.fromkeys(row[0].strip() for row in reader if row and row[0].strip())
    for name in names:
        if name in (".", "..") or "/" in name or "\\" in name:
            raise ValueError(f"第一列不是单个数据包名称：{name!r}")
    return list(names)


def classify(csv_path, root, target, dry_run=False):
    root = root.absolute()
    target = target.absolute()
    if root == target:
        raise ValueError("源目录和目标目录不能相同")
    names = read_names(csv_path)
    if any(target == root / name or root / name in target.parents for name in names):
        raise ValueError("目标目录不能位于待移动的数据包目录内")
    print(f"CSV 数据包数：{len(names)}\n源目录：{root}\n目标目录：{target}", flush=True)
    # 只扫描源目录第一层，不递归扫描数据包内容。
    with os.scandir(root) as entries:
        source_dirs = {entry.name for entry in entries if entry.is_dir(follow_symlinks=False)}
    target_names = set()
    if target.exists():
        with os.scandir(target) as entries:
            target_names = {entry.name for entry in entries}

    moved = missing = existing = conflicts = failed = 0
    for name in names:
        if name in target_names:
            if name in source_dirs:
                conflicts += 1
                print(f"[冲突，跳过] 源目录和目标目录均存在：{name}", flush=True)
            else:
                existing += 1
                print(f"[目标已存在，跳过] {name}", flush=True)
            continue
        if name not in source_dirs:
            missing += 1
            print(f"[未找到] {name}", flush=True)
            continue
        if dry_run:
            print(f"[预览] {root / name} -> {target / name}", flush=True)
        else:
            try:
                target.mkdir(parents=True, exist_ok=True)
                destination = target / name
                if os.path.lexists(destination):
                    conflicts += 1
                    print(f"[冲突，跳过] {destination}", flush=True)
                    continue
                # 同一文件系统优先重命名；跨盘时复制成功后删除源目录。
                shutil.move(str(root / name), str(destination))
            except OSError as error:
                failed += 1
                print(f"[移动失败] {name}：{error}", flush=True)
                continue
            print(f"[已移动] {name}", flush=True)
        moved += 1
    print(
        f"\n{'预计移动' if dry_run else '已移动'}：{moved}；目标已存在：{existing}；"
        f"未找到：{missing}；冲突：{conflicts}；失败：{failed}",
        flush=True,
    )
    return 1 if missing or conflicts or failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=CSV_PATH, help="筛选表 CSV 路径")
    parser.add_argument("--source", "--root", type=Path, default=SOURCE_DIR, help="数据包所在目录")
    parser.add_argument("--target", type=Path, default=TARGET_DIR, help="转移目标目录，不存在时自动创建")
    parser.add_argument("--dry-run", action="store_true", help="仅预览，不新建目录、不移动数据")
    args = parser.parse_args()
    try:
        return classify(args.csv, args.source, args.target, args.dry_run)
    except (OSError, ValueError) as error:
        parser.exit(1, f"错误：{error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
