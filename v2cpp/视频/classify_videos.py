#!/usr/bin/env python3
"""按文件夹名称，把源文件夹内容复制到目标根目录中的同名文件夹。"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from pathlib import Path


SOURCE_ROOTS = [
    # Path("/mnt/data_storage/研发/HIT/2026_video_download"),
    Path("/mnt/data_storage/研发/HIT/防吊起/rosbag/651_8.20-26/651_video"),
]

TARGET_ROOTS = [
    Path("/mnt/data_storage/研发/HIT/防吊起/rosbag/651_8.20-26"),
    # Path("/mnt/data_storage/研发/HIT/堆场"),
]

def direct_folders(root: Path) -> dict[str, Path]:
    folders = {}
    last_report = time.monotonic()
    with os.scandir(root) as entries:
        for entry in entries:
            if entry.is_dir(follow_symlinks=False):
                folders[entry.name] = Path(entry.path)
            now = time.monotonic()
            if now - last_report >= 2:
                print(
                    f"  已读取源目录 {len(folders)} 个文件夹: {root}",
                    flush=True,
                )
                last_report = now
    return folders


def folder_contents(folder: Path) -> tuple[list[Path], list[Path]]:
    files = []
    directories = []
    for current, dirnames, filenames in os.walk(folder):
        current_path = Path(current)
        directories.extend(current_path / dirname for dirname in dirnames)
        files.extend(current_path / filename for filename in filenames)
    return sorted(files), sorted(directories)


def existing_same_name_mp4(target_file: Path) -> Path | None:
    if target_file.suffix.lower() != ".dav" or not target_file.parent.is_dir():
        return None
    return next(
        (
            candidate
            for candidate in target_file.parent.iterdir()
            if candidate.is_file()
            and candidate.stem == target_file.stem
            and candidate.suffix.lower() == ".mp4"
        ),
        None,
    )


def target_dirs(
    roots: list[Path], excluded_roots: list[Path], source_names: set[str]
) -> list[tuple[Path, Path]]:
    result = []
    excluded_paths = {path.absolute() for path in excluded_roots}
    for root in roots:
        if not root.is_dir():
            print(f"目标目录不存在，跳过: {root}", flush=True)
            continue
        print(f"开始递归扫描目标目录: {root}", flush=True)
        scanned = 0
        skipped_extracted = 0
        skipped_copied = 0
        skipped_source = 0
        last_report = time.monotonic()
        for current, dirnames, _ in os.walk(root):
            current_path = Path(current)
            retained_dirnames = [
                dirname
                for dirname in dirnames
                if "extracted" not in dirname.lower()
                and dirname != current_path.name
                and (current_path / dirname).absolute() not in excluded_paths
            ]
            skipped_extracted += sum(
                "extracted" in dirname.lower() for dirname in dirnames
            )
            skipped_copied += sum(
                "extracted" not in dirname.lower() and dirname == current_path.name
                for dirname in dirnames
            )
            skipped_source += sum(
                (current_path / dirname).absolute() in excluded_paths
                for dirname in dirnames
            )
            dirnames[:] = retained_dirnames
            result.extend((root, current_path / dirname) for dirname in dirnames)
            scanned += 1
            if current_path == root and source_names <= set(dirnames):
                dirnames.clear()
                print(
                    "  所有源文件夹均已在目标根一级匹配，停止向下递归",
                    flush=True,
                )
            now = time.monotonic()
            if now - last_report >= 2:
                print(
                    f"  已扫描 {scanned} 个目录，当前: {current_path}",
                    flush=True,
                )
                last_report = now
        print(
            f"目标目录扫描完成: {root}，共访问 {scanned} 个目录，"
            f"跳过 {skipped_extracted} 个 extracted 目录树，"
            f"跳过 {skipped_copied} 个内层同名目录树，"
            f"跳过 {skipped_source} 个源目录树",
            flush=True,
        )
    return result


def resolve_target(
    source_name: str, target_dir_list: list[tuple[Path, Path]]
) -> tuple[Path | None, str | None]:
    candidates = [
        (target_root, path)
        for target_root, path in target_dir_list
        if path.name == source_name
    ]
    direct_candidates = [
        (target_root, path)
        for target_root, path in candidates
        if path.parent == target_root
    ]
    if direct_candidates:
        candidates = direct_candidates
    elif candidates:
        minimum_depth = min(
            len(path.relative_to(target_root).parts)
            for target_root, path in candidates
        )
        candidates = [
            (target_root, path)
            for target_root, path in candidates
            if len(path.relative_to(target_root).parts) == minimum_depth
        ]

    if len(candidates) == 1:
        return candidates[0][1], None
    if not candidates:
        print(f"未找到同名目标目录，跳过: {source_name}")
        reason = "未找到同名目标目录"
    else:
        print(f"同名目标目录不唯一，跳过: {source_name}")
        for _, path in candidates:
            print(f"  候选: {path}")
        reason = "同名目标目录不唯一"
    return None, reason


def folder_already_copied(
    source_folder: Path,
    destination_folder: Path,
    files: list[Path],
    directories: list[Path],
) -> bool:
    if not destination_folder.is_dir():
        return False
    if any(
        not (destination_folder / path.relative_to(source_folder)).is_dir()
        for path in directories
    ):
        return False
    return all(
        (
            (target_file := destination_folder / path.relative_to(source_folder)).is_file()
            and target_file.stat().st_size == path.stat().st_size
        )
        or existing_same_name_mp4(target_file) is not None
        for path in files
    )


def classify(copy_mode: str) -> None:
    source_folders = []
    for root in SOURCE_ROOTS:
        if not root.is_dir():
            print(f"源目录不存在，跳过: {root}")
            continue
        source_folders.extend(direct_folders(root).values())

    target_dir_list = target_dirs(
        TARGET_ROOTS,
        SOURCE_ROOTS,
        {folder.name for folder in source_folders},
    )

    copied = 0
    skipped_existing = 0
    skipped_same_name_mp4 = 0
    skipped_unmatched = 0
    skipped_folders: list[tuple[Path, str]] = []
    already_copied_folders: list[tuple[Path, Path]] = []
    total_bytes = 0

    for source_folder in sorted(source_folders, key=lambda path: (path.name, str(path))):
        print(f"\n开始处理源文件夹: {source_folder}", flush=True)
        target_folder, skip_reason = resolve_target(
            source_folder.name, target_dir_list
        )
        if target_folder is not None:
            print(f"匹配到目标文件夹: {target_folder}", flush=True)
        print(f"正在统计源文件夹内容: {source_folder}", flush=True)
        files, directories = folder_contents(source_folder)
        print(f"源文件夹共 {len(files)} 个文件", flush=True)
        if target_folder is None:
            skipped_unmatched += len(files)
            skipped_folders.append((source_folder, skip_reason or "未知原因"))
            continue

        destination_folder = target_folder / source_folder.name
        if folder_already_copied(
            source_folder, destination_folder, files, directories
        ):
            already_copied_folders.append((source_folder, destination_folder))
            print(f"整个文件夹已传过，跳过: {destination_folder}")
            continue

        print(
            f"整文件夹"
            f"{'复制' if copy_mode == 'copy' else '移动' if copy_mode == 'move' else '空跑'}: "
            f"{source_folder} -> {target_folder}",
            flush=True,
        )
        if copy_mode == "dry-run":
            if not destination_folder.exists():
                print(f"[空跑] 创建目录: {destination_folder}")
        else:
            destination_folder.mkdir(parents=True, exist_ok=True)

        for source_dir in directories:
            target_dir = destination_folder / source_dir.relative_to(source_folder)
            if target_dir.exists():
                continue
            if copy_mode == "dry-run":
                print(f"[空跑] 创建目录: {target_dir}")
            else:
                target_dir.mkdir(parents=True, exist_ok=True)

        for source_file in files:
            relative_path = source_file.relative_to(source_folder)
            target_file = destination_folder / relative_path
            source_size = source_file.stat().st_size

            same_name_mp4 = existing_same_name_mp4(target_file)
            if same_name_mp4 is not None:
                skipped_same_name_mp4 += 1
                print(f"目标已有同名 MP4，跳过 DAV: {source_file} -> {same_name_mp4}")
                continue

            if target_file.exists():
                if target_file.stat().st_size == source_size:
                    skipped_existing += 1
                    print(f"目标已存在且大小一致，跳过: {target_file}")
                    continue
                print(f"目标大小不一致，重新复制: {target_file}")

            if copy_mode == "dry-run":
                print(f"[空跑] {source_file} -> {target_file}")
            else:
                target_file.parent.mkdir(parents=True, exist_ok=True)
                print(
                    f"正在{'复制' if copy_mode == 'copy' else '移动'} "
                    f"{source_size / 1024 / 1024:.1f} MiB: "
                    f"{source_file} -> {target_file}"
                )
                if copy_mode == "copy":
                    shutil.copyfile(source_file, target_file)
                else:
                    shutil.move(source_file, target_file)
                print(f"{copy_mode}: {source_file} -> {target_file}")
            copied += 1
            total_bytes += source_size

    print(
        f"\n完成：{copy_mode} {copied} 个文件，约 {total_bytes / 1024 / 1024:.1f} MiB；"
        f"已存在跳过 {skipped_existing} 个；同名 MP4 跳过 DAV {skipped_same_name_mp4} 个；"
        f"已传过跳过 {len(already_copied_folders)} 个文件夹；"
        f"未匹配或无唯一目标跳过 {skipped_unmatched} 个文件。"
    )
    if already_copied_folders:
        print(f"\n已传过而跳过的源文件夹（共 {len(already_copied_folders)} 个）：")
        for source_folder, destination_folder in already_copied_folders:
            print(f"  - {source_folder}\n    已存在: {destination_folder}")
    else:
        print("\n已传过而跳过的源文件夹：无")

    if skipped_folders:
        print(f"\n未处理的源文件夹（共 {len(skipped_folders)} 个）：")
        for source_folder, reason in skipped_folders:
            print(f"  - {source_folder}\n    原因: {reason}")
    else:
        print("\n未处理的源文件夹：无")


def main() -> None:
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("dry-run", "copy", "move"),
        default="copy",
        help="dry-run 只显示，copy 复制并保留源文件，move 移动源文件（默认 copy）",
    )
    args = parser.parse_args()
    classify(args.mode)


if __name__ == "__main__":
    main()
