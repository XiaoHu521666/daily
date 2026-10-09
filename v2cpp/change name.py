#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
import re
import sys
import threading
import time
from pathlib import Path

# 默认目录（如果你在 Windows 跑，改成 r"\\10.0.0.110\Data_Storage\研发\HIT\大车防撞\655_2026_09_11"）
DEFAULT_DIR = "/mnt/data_storage/研发/HIT/大车防撞/655_09_11_video"

TOKEN = re.compile(r'(?i)(?<![A-Za-z0-9])(left|right)(?![A-Za-z0-9])')
MULTI_SEP = re.compile(r'[_-]{2,}')

def clean_name(name: str) -> str:
    stem, ext = os.path.splitext(name)
    new_stem = TOKEN.sub('', stem)
    new_stem = MULTI_SEP.sub('_', new_stem)
    new_stem = new_stem.strip('_-')
    if not new_stem:
        return name
    return new_stem + ext


def scan_items(root: Path, recursive: bool):
    """扫描目录，并持续输出进度；避免对网络盘重复执行 is_file/is_dir。"""
    items = []
    scanned = 0
    started = time.monotonic()
    last_report = started

    print(f'[扫描] 开始读取: {root}', flush=True)
    if recursive:
        for current, dirs, files in os.walk(root):
            base = Path(current)
            for name in dirs + files:
                items.append(base / name)
                scanned += 1
            now = time.monotonic()
            if now - last_report >= 1:
                print(
                    f'[扫描] 已读取 {scanned} 项，当前目录: {base}',
                    flush=True,
                )
                last_report = now
    else:
        with os.scandir(root) as entries:
            for entry in entries:
                items.append(root / entry.name)
                scanned += 1
                now = time.monotonic()
                if scanned % 100 == 0 or now - last_report >= 1:
                    print(f'[扫描] 已读取 {scanned} 项', flush=True)
                    last_report = now

    elapsed = time.monotonic() - started
    print(f'[扫描] 完成，共读取 {scanned} 项，耗时 {elapsed:.1f} 秒。', flush=True)
    return items


def scan_with_heartbeat(root: Path, recursive: bool):
    stop = threading.Event()

    def report_waiting():
        while not stop.wait(5):
            print('[扫描] 网络目录仍在读取，请稍候……', flush=True)

    reporter = threading.Thread(target=report_waiting, daemon=True)
    reporter.start()
    try:
        return scan_items(root, recursive)
    finally:
        stop.set()
        reporter.join()

def main():
    ap = argparse.ArgumentParser(description='去掉文件名或文件夹名中的 left/right')
    ap.add_argument('-d', '--dir', default=DEFAULT_DIR, help='目标目录')
    ap.add_argument('-r', '--recursive', action='store_true', help='递归子目录')
    ap.add_argument('--apply', action='store_true', help='实际执行改名（默认只预览）')
    args = ap.parse_args()

    root = Path(args.dir)
    if not root.is_dir():
        sys.exit(f'[错误] 目录不存在或不是文件夹: {root}')

    try:
        items = scan_with_heartbeat(root, args.recursive)
    except OSError as e:
        sys.exit(f'[错误] 扫描目录失败: {e}')
    
    # 排序逻辑
    if args.recursive:
        # 递归时，先处理深层路径，避免父文件夹改名后子路径找不到
        items.sort(key=lambda p: len(p.parts), reverse=True)
    else:
        items.sort()

    plan = []
    for p in items:
        new = clean_name(p.name)
        if new != p.name:
            plan.append((p, p.with_name(new)))

    if not plan:
        print('没有需要改名的文件或文件夹。')
        return

    print(f'目录: {root}')
    print(f'共 {len(plan)} 个项待改名：\n')
    for src, dst in plan:
        mark = ''
        if dst.exists() and dst != src:
            mark = '   <-- 目标已存在，执行时会跳过'
        print(f'{src.name}', flush=True)
        print(f'    -> {dst.name}{mark}', flush=True)
    print()

    if not args.apply:
        print('以上为预览。确认无误后加 --apply 真正执行：')
        print(f'    python3 {Path(__file__).name} -d "{root}" --apply')
        return

    done, skipped = 0, []
    total = len(plan)
    for index, (src, dst) in enumerate(plan, 1):
        if dst.exists() and dst != src:
            reason = '目标名称已存在'
            skipped.append((src, dst, reason))
            print(f'[{index}/{total}] 跳过: {src.name} -> {dst.name}（{reason}）', flush=True)
            continue
        try:
            src.rename(dst)
            done += 1
            print(f'[{index}/{total}] 成功: {src.name} -> {dst.name}', flush=True)
        except OSError as e:
            reason = str(e)
            skipped.append((src, dst, reason))
            print(
                f'[{index}/{total}] 失败: {src.name} -> {dst.name}（{reason}）',
                file=sys.stderr,
                flush=True,
            )

    print(f'完成：成功改名 {done} 个，跳过 {len(skipped)} 个。')
    if skipped:
        print('未改名项目：')
        for src, dst, reason in skipped:
            print(f'  {src.name} -> {dst.name}：{reason}')
        sys.exit(1)

if __name__ == '__main__':
    main()
