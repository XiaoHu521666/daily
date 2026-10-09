import argparse
from pathlib import Path

import open3d as o3d

from BaseConfig import ALGL_CONFIG


LIDAR_DEVICE_MAP = {
    10: "L18",
    11: "L19",
    36: "L20",
    37: "L21",
}


def export_compensated_point_clouds(input_folder: str, output_folder: str) -> None:
    """按每台雷达自己的外参矩阵补偿逐帧点云。"""
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    total = 0

    for device_id, device_name in LIDAR_DEVICE_MAP.items():
        source_files = sorted((input_path / device_name).glob("*.pcd"))
        if not source_files:
            continue

        target_dir = output_path / device_name
        target_dir.mkdir(parents=True, exist_ok=True)
        transform = o3d.core.Tensor(
            ALGL_CONFIG.TRANSFORM_MATRICES[device_id][0],
            dtype=o3d.core.Dtype.Float64,
        )

        for source_file in source_files:
            point_cloud = o3d.t.io.read_point_cloud(str(source_file))
            point_cloud.transform(transform)
            target_file = target_dir / source_file.name
            if not o3d.t.io.write_point_cloud(str(target_file), point_cloud):
                raise RuntimeError(f"点云写入失败: {target_file}")
            total += 1

        print(f"{device_name}: 已输出 {len(source_files)} 帧到 {target_dir}")

    if total == 0:
        raise FileNotFoundError(f"数据集中没有找到 L18/L19/L20/L21 点云: {input_path}")
    print(f"补偿完成，共输出 {total} 帧: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="导出大车防撞外参补偿后的逐帧点云")
    parser.add_argument("input_folder", help="解包后的数据集目录")
    parser.add_argument("--output", help="输出目录；默认保存在当前目录的 output 下")
    args = parser.parse_args()

    output_folder = args.output or str(
        Path.cwd() / "output" / f"{Path(args.input_folder).name}_compensated"
    )
    export_compensated_point_clouds(args.input_folder, output_folder)
# /home/wxh/code/dc531/.venv/bin/python3 /home/wxh/code/hit_gantry_collision/export_compensated_point_clouds.py "/mnt/data_storage/研发/HIT/大车防撞/x655_2026_09_09/rtg_655_gantry_collision_right_2026_09_09_16_24_00_488996827_655_extracted"