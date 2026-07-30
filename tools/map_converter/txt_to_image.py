#!/usr/bin/env python3
"""
地图转换器 - 将 txt 格式的栅格地图转换为 PNG/SVG 图片

使用方法:
    python txt_to_image.py <input_txt_file> [output_file] [--format png|svg] [--point-size SIZE]

示例:
    python txt_to_image.py /path/to/map_gridmap.txt ./output/map.png --format png
    python txt_to_image.py /path/to/map_gridmap.txt ./output/map.svg --format svg --point-size 0.8
"""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple, Optional

# 颜色配置
COLORS = {
    "graphite": "#374151",      # 石墨灰
    "orange": "#f97316",       # 橙色
    "graphite_orange_mix": "#c25e1f",  # 石墨橙混合
    "background": "#1f2937",     # 深色背景
    "grid": "#4b5563",          # 网格线
}


def parse_grid_map(filepath: str) -> Tuple[float, float, float, int, int, List[Tuple[float, float]]]:
    """
    解析 txt 地图文件

    返回:
        (resolution, origin_x, origin_y, grid_width, grid_height, points)
    """
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    if len(lines) < 3:
        raise ValueError(f"地图文件格式错误: {filepath}")

    # 第一行：分辨率
    resolution = float(lines[0])

    # 第二行：原始世界坐标起始点
    origin_x, origin_y = map(float, lines[1].split())

    # 第三行：栅格宽高
    grid_width, grid_height = map(int, lines[2].split())

    # 修正坐标原点 (根据 MATLAB 代码逻辑)
    adjusted_origin_x = origin_x + (grid_width / 2) * resolution
    adjusted_origin_y = origin_y + (grid_height / 2) * resolution

    points: List[Tuple[float, float]] = []

    # 从第四行开始是详细的二维占据栅格点
    for line in lines[3:]:
        parts = line.split()
        if len(parts) < 2:
            continue

        grid_x, grid_y = map(float, parts[:2])

        # 翻转 y 轴，并替换坐标系 (MATLAB 逻辑)
        old_grid_y = grid_width - 1 - grid_y

        # 计算具体的世界坐标
        wx = adjusted_origin_x + (grid_width / 2 - old_grid_y) * resolution
        wy = adjusted_origin_y + (grid_height / 2 - grid_x) * resolution

        points.append((wx, wy))

    return resolution, origin_x, origin_y, grid_width, grid_height, points


def get_bounding_box(points: List[Tuple[float, float]], padding: float = 0.1) -> Tuple[float, float, float, float]:
    """计算点集的边界框"""
    if not points:
        return 0, 0, 100, 100

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # 添加 padding
    width = max_x - min_x
    height = max_y - min_y
    pad_x = width * padding
    pad_y = height * padding

    return min_x - pad_x, min_y - pad_y, max_x + pad_x, max_y + pad_y


def convert_to_png(input_path: str, output_path: str, point_size: float = 1.0, dpi: int = 150):
    """将 txt 地图转换为 PNG 图片"""
    try:
        import matplotlib
        matplotlib.use('Agg')  # 非交互式后端
        import matplotlib.pyplot as plt
    except ImportError:
        print("错误: 需要安装 matplotlib。请运行: pip install matplotlib")
        sys.exit(1)

    # 解析地图
    print(f"正在解析地图文件: {input_path}")
    resolution, origin_x, origin_y, grid_width, grid_height, points = parse_grid_map(input_path)

    print(f"分辨率: {resolution}")
    print(f"原点: ({origin_x}, {origin_y})")
    print(f"栅格尺寸: {grid_width} x {grid_height}")
    print(f"点数: {len(points)}")

    if not points:
        print("警告: 地图中没有点数据")
        return

    # 创建图形
    fig, ax = plt.subplots(figsize=(12, 10), dpi=dpi)

    # 提取坐标
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    # 绘制点 - 使用石墨橙色
    ax.plot(xs, ys, 'o', color=COLORS["graphite_orange_mix"], markersize=point_size, alpha=0.8)

    # 设置背景色
    ax.set_facecolor(COLORS["background"])
    fig.patch.set_facecolor(COLORS["background"])

    # 设置标题和标签颜色
    ax.set_title("Grid Map", color='white', fontsize=14, fontweight='bold')
    ax.set_xlabel("X (meters)", color='white', fontsize=11)
    ax.set_ylabel("Y (meters)", color='white', fontsize=11)

    # 设置刻度颜色
    ax.tick_params(colors='white')

    # 添加网格
    ax.grid(True, linestyle='--', alpha=0.3, color=COLORS["grid"])

    # 设置等比例
    ax.set_aspect('equal', adjustable='box')

    # 保存图片
    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight', facecolor=COLORS["background"])
    plt.close()

    print(f"地图已保存到: {output_path}")


def convert_to_svg(input_path: str, output_path: str, point_size: float = 1.0):
    """将 txt 地图转换为 SVG 矢量图"""
    try:
        import xml.etree.ElementTree as ET
    except ImportError:
        raise ImportError("xml.etree.ElementTree 模块不可用")

    # 解析地图
    print(f"正在解析地图文件: {input_path}")
    resolution, origin_x, origin_y, grid_width, grid_height, points = parse_grid_map(input_path)

    print(f"分辨率: {resolution}")
    print(f"原点: ({origin_x}, {origin_y})")
    print(f"栅格尺寸: {grid_width} x {grid_height}")
    print(f"点数: {len(points)}")

    if not points:
        print("警告: 地图中没有点数据")
        return

    # 计算边界框
    bbox = get_bounding_box(points, padding=0.05)
    min_x, min_y, max_x, max_y = bbox

    width = max_x - min_x
    height = max_y - min_y

    # 创建 SVG
    ET.register_namespace('', 'http://www.w3.org/2000/svg')
    svg = ET.Element('svg')
    svg.set('xmlns', 'http://www.w3.org/2000/svg')
    svg.set('viewBox', f'{min_x} {min_y} {width} {height}')
    svg.set('width', '100%')
    svg.set('height', '100%')

    # 添加样式
    style = ET.SubElement(svg, 'style')
    style.text = f"""
        .background {{ fill: {COLORS['background']}; }}
        .grid {{ stroke: {COLORS['grid']}; stroke-width: {resolution * 0.1}; opacity: 0.3; }}
        .point {{ fill: {COLORS['graphite_orange_mix']}; }}
        .title {{ fill: {COLORS['text']}; font-family: Arial, sans-serif; font-size: 14px; font-weight: bold; text-anchor: middle; }}
        .label {{ fill: {COLORS['text']}; font-family: Arial, sans-serif; font-size: 11px; text-anchor: middle; }}
    """

    # 设置背景
    bg = ET.SubElement(svg, 'rect')
    bg.set('class', 'background')
    bg.set('x', str(min_x))
    bg.set('y', str(min_y))
    bg.set('width', str(width))
    bg.set('height', str(height))

    # 绘制网格
    grid_group = ET.SubElement(svg, 'g')
    grid_group.set('class', 'grid')

    # 垂直网格线
    x = (min_x // (resolution * 10)) * (resolution * 10)
    while x < min_x + width:
        line = ET.SubElement(grid_group, 'line')
        line.set('x1', str(x))
        line.set('y1', str(min_y))
        line.set('x2', str(x))
        line.set('y2', str(min_y + height))
        x += resolution * 10

    # 水平网格线
    y = (min_y // (resolution * 10)) * (resolution * 10)
    while y < min_y + height:
        line = ET.SubElement(grid_group, 'line')
        line.set('x1', str(min_x))
        line.set('y1', str(y))
        line.set('x2', str(min_x + width))
        line.set('y2', str(y))
        y += resolution * 10

    # 绘制点
    points_group = ET.SubElement(svg, 'g')
    points_group.set('class', 'point')

    r = resolution * point_size * 0.5

    for px, py in points:
        circle = ET.SubElement(points_group, 'circle')
        circle.set('cx', str(px))
        circle.set('cy', str(py))
        circle.set('r', str(r))

    # 添加标题
    title = ET.SubElement(svg, 'text')
    title.set('class', 'title')
    title.set('x', str(min_x + width / 2))
    title.set('y', str(min_y + height * 0.05))
    title.text = 'Grid Map'

    # 保存 SVG
    tree = ET.ElementTree(svg)
    ET.indent(tree, space='  ')
    tree.write(output_path, encoding='utf-8', xml_declaration=True)

    print(f"SVG 地图已保存到: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='将 txt 格式的栅格地图转换为 PNG 或 SVG 图片'
    )
    parser.add_argument('input', help='输入的 txt 地图文件路径')
    parser.add_argument('output', nargs='?', help='输出文件路径 (可选，默认使用输入文件名)')
    parser.add_argument('--format', '-f', choices=['png', 'svg'], default='png',
                        help='输出格式 (默认: png)')
    parser.add_argument('--point-size', '-s', type=float, default=1.0,
                        help='点的大小倍数 (默认: 1.0)')
    parser.add_argument('--dpi', '-d', type=int, default=150,
                        help='PNG 输出分辨率 (默认: 150)')

    args = parser.parse_args()

    # 确定输出路径
    if args.output:
        output_path = args.output
    else:
        input_path = Path(args.input)
        output_path = input_path.with_suffix(f'.{args.format}')

    # 执行转换
    if args.format == 'png':
        convert_to_png(args.input, str(output_path), args.point_size, args.dpi)
    else:
        convert_to_svg(args.input, str(output_path), args.point_size)


if __name__ == '__main__':
    main()
