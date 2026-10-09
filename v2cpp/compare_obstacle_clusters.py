"""对已补偿的红绿 PCD 独立复核，红点为输入；不会再次应用外参。"""
import csv
import importlib
from datetime import datetime
from pathlib import Path
from time import perf_counter

import numpy as np
import open3d as o3d

from lidar_service import LidarService, points_in_roi


def compare(input_folder, output_folder, config_name='config_655', limit=0):
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    cfg = importlib.import_module(config_name).config
    service = LidarService(cfg)
    files = sorted(input_path.glob('*.pcd'))
    if not files:
        raise FileNotFoundError(f'输入目录中没有 PCD 文件：{input_path.resolve()}')
    if limit:
        files = files[:limit]
    output_path.mkdir(parents=True, exist_ok=False)
    print(f'输出目录：{output_path.resolve()}；共 {len(files)} 帧', flush=True)
    device_ids = {'L18': 10, 'L19': 11, 'L20': 36, 'L21': 37}
    with (output_path / 'comparison.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['file', 'old_distance', 'new_distance', 'new_voxels', 'elapsed_ms'])
        for i, path in enumerate(files, 1):
            cloud = o3d.io.read_point_cloud(str(path))
            points = np.asarray(cloud.points)
            if cloud.has_colors():
                colors = np.asarray(cloud.colors)
                points = points[(colors[:, 0] > .5) & (colors[:, 1] < .5)]
            device = device_ids[path.name.split('_')[0]]
            tensor = service._to_tensor(points)
            roi = tensor[points_in_roi(tensor, service.lidar_exclusion_regions[device])]
            centroids, raw = service._voxel_downsample(roi, cfg.LIDAR_VOXEL_SIZE, cfg.LIDAR_VOXEL_MIN_POINTS)
            old = raw[:, 0].min().item() if len(centroids) > cfg.LIDAR_OBSTACLE_MIN_VOXELS else 9999.0
            o3d.utility.random.seed(7)
            start = perf_counter()
            new, kept, _ = service._lidar_process2(tensor, device)
            elapsed = (perf_counter() - start) * 1000
            background = service._visualization_cloud(tensor, device)
            if kept is not None:
                kept.paint_uniform_color([0, 1, 0])
                background += kept
            if not o3d.io.write_point_cloud(str(output_path / path.name), background):
                raise OSError(f'写入失败：{path.name}')
            writer.writerow([path.name, old, new, len(kept.points) if kept is not None else 0, elapsed])
            print(f'[{i}/{len(files)}] {path.name}: {old:.3f} → {new:.3f}', flush=True)


if __name__ == '__main__':
    # 在这里改路径，然后点击编辑器“运行 Python 文件”，无需终端参数。
    project_dir = Path(__file__).resolve().parent

    # 输入：已经过外参补偿的 PCD 目录，文件名以 L18/L19/L20/L21 开头。
    # 支持原脚本保存的红色背景+绿色检测结果；不要填写原始雷达 PCD 或 DB3 目录。
    input_folder = project_dir / '222rtg_655_gantry_collision_2026_09_11_07_49_52_861335553'

    # 输出：每次自动新建时间戳目录，避免覆盖上次结果；可修改前面的目录。
    output_folder = project_dir / 'cluster_comparison' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    config_name = 'config_655'  # 必须与输入点云所属设备及补偿配置一致。
    limit = 0  # 0：全部；例如 20：只检查前 20 个文件。

    compare(input_folder, output_folder, config_name, limit)
