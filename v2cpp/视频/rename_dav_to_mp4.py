#!/usr/bin/env python3
"""整理所有 *_extracted/C1、C2：将 DAV 改为 MP4，同名时删除 DAV。"""

from __future__ import annotations

import argparse
from pathlib import Path


DEFAULT_TARGETS = [Path("/mnt/data_storage/研发/HIT/新版堆扫")]


def camera_dirs(root: Path) -> list[Path]:
    result = []
    print(f"开始查找 extracted 目录: {root}", flush=True)
    for folder in sorted(root.iterdir()):
        if not folder.is_dir() or "extracted" not in folder.name.lower():
            continue
        print(f"发现 extracted 目录: {folder}", flush=True)
        for camera_name in ("C1", "C2"):
            camera = folder / camera_name
            if camera.is_dir():
                print(f"  找到 {camera_name} 目录: {camera}", flush=True)
                result.append(camera)
            else:
                print(f"  未找到 {camera_name} 目录，跳过", flush=True)
    return result


def process_root(root: Path, dry_run: bool) -> tuple[int, int, int]:
    renamed = 0
    removed = 0
    checked = 0

    folders = camera_dirs(root)
    print(f"共找到 {len(folders)} 个 C1/C2 目录，开始逐个检查。", flush=True)
    for folder_index, folder in enumerate(folders, start=1):
        print(f"\n[{folder_index}/{len(folders)}] 检查目录: {folder}", flush=True)
        dav_list = sorted(
            path
            for path in folder.iterdir()
            if path.is_file() and path.suffix.lower() == ".dav"
        )
        print(f"  发现 {len(dav_list)} 个 DAV 文件", flush=True)

        for file_index, dav in enumerate(dav_list, start=1):
            checked += 1
            print(f"  [{file_index}/{len(dav_list)}] 检查: {dav.name}", flush=True)
            mp4 = dav.with_suffix(".MP4")
            existing_mp4 = next(
                (candidate for candidate in (mp4, dav.with_suffix(".mp4")) if candidate.exists()),
                None,
            )

            if existing_mp4 is None:
                action = "[预览] 改名" if dry_run else "改名"
                print(f"    {action}: {dav.name} -> {mp4.name}", flush=True)
                if not dry_run:
                    dav.rename(mp4)
                renamed += 1
                continue

            action = "[预览] 删除" if dry_run else "删除"
            print(
                f"    找到同名 MP4，{action} DAV，保留: {existing_mp4.name}",
                flush=True,
            )
            if not dry_run:
                dav.unlink()
            removed += 1

    return checked, renamed, removed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "targets",
        nargs="*",
        type=Path,
        default=DEFAULT_TARGETS,
        help="需要递归整理的目标目录；默认处理新版堆扫目录",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只预览，不改名或删除；不加此参数时实际执行",
    )
    args = parser.parse_args()

    totals = [0, 0, 0]
    for root in args.targets:
        root = root.expanduser().resolve()
        if not root.is_dir():
            print(f"目标目录不存在，跳过: {root}")
            continue
        print(f"\n处理目标目录: {root}", flush=True)
        result = process_root(root, args.dry_run)
        totals = [left + right for left, right in zip(totals, result)]

    mode = "预览" if args.dry_run else "执行"
    print(
        f"\n{mode}完成：检查 DAV {totals[0]} 个，改名 {totals[1]} 个，"
        f"删除同名 DAV {totals[2]} 个。",
        flush=True,
    )


if __name__ == "__main__":
    main()
