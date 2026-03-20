"""
地图转换服务 - 将 txt 栅格地图转换为 PNG 图片

提供地图解析和转换功能，支持自动缓存。
"""

import os
import hashlib
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
import io
import base64

# 颜色配置
COLORS = {
    "graphite": "#374151",
    "orange": "#f97316",
    "graphite_orange_mix": "#c25e1f",  # 石墨橙
    "background": "#1f2937",
    "grid": "#4b5563",
    "text": "#f3f4f6",
}


class MapData:
    """地图数据类"""

    def __init__(
        self,
        resolution: float,
        origin_x: float,
        origin_y: float,
        grid_width: int,
        grid_height: int,
        points: List[Tuple[float, float]],
        source_file: str = "",
    ):
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.points = points
        self.source_file = source_file

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """返回地图边界 (min_x, min_y, max_x, max_y)"""
        if not self.points:
            return (0, 0, 100, 100)
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        return (min(xs), min(ys), max(xs), max(ys))

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        bounds = self.bounds
        return {
            "resolution": self.resolution,
            "origin": {"x": self.origin_x, "y": self.origin_y},
            "grid_size": {"width": self.grid_width, "height": self.grid_height},
            "bounds": {
                "min_x": bounds[0],
                "min_y": bounds[1],
                "max_x": bounds[2],
                "max_y": bounds[3],
            },
            "point_count": len(self.points),
            "source_file": self.source_file,
        }


class MapConverter:
    """地图转换器"""

    def __init__(self, cache_dir: Optional[str] = None):
        self.cache_dir = Path(cache_dir) if cache_dir else Path(__file__).parent / "../../data/map/cache"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

    def parse_grid_map(self, filepath: str) -> MapData:
        """解析 txt 地图文件"""
        with open(filepath, "r") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        if len(lines) < 3:
            raise ValueError(f"地图文件格式错误: {filepath}")

        # 第一行：分辨率
        resolution = float(lines[0])

        # 第二行：原始世界坐标起始点
        origin_x, origin_y = map(float, lines[1].split())

        # 第三行：栅格宽高
        grid_width, grid_height = map(int, lines[2].split())

        # 修正坐标原点
        adjusted_origin_x = origin_x + (grid_width / 2) * resolution
        adjusted_origin_y = origin_y + (grid_height / 2) * resolution

        points: List[Tuple[float, float]] = []

        # 从第四行开始是详细的二维占据栅格点
        for line in lines[3:]:
            parts = line.split()
            if len(parts) < 2:
                continue

            grid_x, grid_y = map(float, parts[:2])

            # 翻转 y 轴，并替换坐标系
            old_grid_y = grid_width - 1 - grid_y

            # 计算具体的世界坐标
            wx = adjusted_origin_x + (grid_width / 2 - old_grid_y) * resolution
            wy = adjusted_origin_y + (grid_height / 2 - grid_x) * resolution

            points.append((wx, wy))

        return MapData(
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            grid_width=grid_width,
            grid_height=grid_height,
            points=points,
            source_file=filepath,
        )

    def to_png(
        self,
        map_data: MapData,
        output_path: Optional[str] = None,
        point_size: float = 1.0,
        dpi: int = 150,
        width: Optional[int] = None,
        height: Optional[int] = None,
    ) -> str:
        """将地图数据转换为 PNG 图片"""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            raise ImportError("需要安装 matplotlib。请运行: pip install matplotlib")

        if not map_data.points:
            raise ValueError("地图中没有点数据")

        # 计算边界
        bounds = map_data.bounds
        min_x, min_y, max_x, max_y = bounds
        data_width = max_x - min_x
        data_height = max_y - min_y

        # 确定输出尺寸
        if width and height:
            figsize = (width / dpi, height / dpi)
        else:
            # 根据数据比例自动计算
            aspect_ratio = data_width / data_height if data_height > 0 else 1
            base_size = 12
            if aspect_ratio > 1:
                figsize = (base_size, base_size / aspect_ratio)
            else:
                figsize = (base_size * aspect_ratio, base_size)

        # 创建图形
        fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

        # 提取坐标
        xs = [p[0] for p in map_data.points]
        ys = [p[1] for p in map_data.points]

        # 绘制点 - 使用石墨橙色
        ax.plot(
            xs, ys,
            'o',
            color=COLORS["graphite_orange_mix"],
            markersize=point_size,
            alpha=0.9,
            markeredgewidth=0
        )

        # 设置背景色
        ax.set_facecolor(COLORS["background"])
        fig.patch.set_facecolor(COLORS["background"])

        # 设置标题和标签颜色
        ax.set_title("Grid Map", color=COLORS["text"], fontsize=14, fontweight='bold')
        ax.set_xlabel("X (meters)", color=COLORS["text"], fontsize=11)
        ax.set_ylabel("Y (meters)", color=COLORS["text"], fontsize=11)

        # 设置刻度颜色
        ax.tick_params(colors=COLORS["text"])

        # 添加网格
        ax.grid(True, linestyle='--', alpha=0.3, color=COLORS["grid"])

        # 设置等比例
        ax.set_aspect('equal', adjustable='box')

        # 设置坐标轴范围（添加一些边距）
        margin = max(data_width, data_height) * 0.05
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)

        # 保存图片
        if output_path:
            plt.tight_layout()
            plt.savefig(
                output_path,
                dpi=dpi,
                bbox_inches='tight',
                facecolor=COLORS["background"],
                edgecolor='none'
            )
            plt.close()
            return output_path
        else:
            # 返回 base64 编码的图片数据
            buf = io.BytesIO()
            plt.savefig(
                buf,
                format='png',
                dpi=dpi,
                bbox_inches='tight',
                facecolor=COLORS["background"],
                edgecolor='none'
            )
            plt.close()
            buf.seek(0)
            return base64.b64encode(buf.getvalue()).decode('utf-8')

    def to_svg(
        self,
        map_data: MapData,
        output_path: str,
        point_size: float = 1.0,
    ) -> str:
        """将地图数据转换为 SVG 矢量图"""
        try:
            import xml.etree.ElementTree as ET
        except ImportError:
            raise ImportError("xml.etree.ElementTree 模块不可用")

        if not map_data.points:
            raise ValueError("地图中没有点数据")

        # 计算边界框
        bounds = map_data.bounds
        min_x, min_y, max_x, max_y = bounds
        width = max_x - min_x
        height = max_y - min_y

        # 添加 padding
        padding_ratio = 0.05
        pad_x = width * padding_ratio
        pad_y = height * padding_ratio

        view_min_x = min_x - pad_x
        view_min_y = min_y - pad_y
        view_width = width + 2 * pad_x
        view_height = height + 2 * pad_y

        # 创建 SVG
        ET.register_namespace('', 'http://www.w3.org/2000/svg')
        svg = ET.Element('svg')
        svg.set('xmlns', 'http://www.w3.org/2000/svg')
        svg.set('viewBox', f'{view_min_x} {view_min_y} {view_width} {view_height}')
        svg.set('width', '100%')
        svg.set('height', '100%')

        # 添加样式
        style = ET.SubElement(svg, 'style')
        style.text = f"""
            .background {{ fill: {COLORS['background']}; }}
            .grid {{ stroke: {COLORS['grid']}; stroke-width: {map_data.resolution * 0.1}; opacity: 0.3; }}
            .point {{ fill: {COLORS['graphite_orange_mix']}; }}
            .title {{ fill: {COLORS['text']}; font-family: Arial, sans-serif; font-size: 14px; font-weight: bold; text-anchor: middle; }}
            .label {{ fill: {COLORS['text']}; font-family: Arial, sans-serif; font-size: 11px; text-anchor: middle; }}
        """

        # 设置背景
        bg = ET.SubElement(svg, 'rect')
        bg.set('class', 'background')
        bg.set('x', str(view_min_x))
        bg.set('y', str(view_min_y))
        bg.set('width', str(view_width))
        bg.set('height', str(view_height))

        # 绘制网格
        grid_group = ET.SubElement(svg, 'g')
        grid_group.set('class', 'grid')

        # 垂直网格线
        x = (view_min_x // (map_data.resolution * 10)) * (map_data.resolution * 10)
        while x < view_min_x + view_width:
            line = ET.SubElement(grid_group, 'line')
            line.set('x1', str(x))
            line.set('y1', str(view_min_y))
            line.set('x2', str(x))
            line.set('y2', str(view_min_y + view_height))
            x += map_data.resolution * 10

        # 水平网格线
        y = (view_min_y // (map_data.resolution * 10)) * (map_data.resolution * 10)
        while y < view_min_y + view_height:
            line = ET.SubElement(grid_group, 'line')
            line.set('x1', str(view_min_x))
            line.set('y1', str(y))
            line.set('x2', str(view_min_x + view_width))
            line.set('y2', str(y))
            y += map_data.resolution * 10

        # 绘制点
        points_group = ET.SubElement(svg, 'g')
        points_group.set('class', 'point')

        r = map_data.resolution * point_size * 0.5

        for px, py in map_data.points:
            circle = ET.SubElement(points_group, 'circle')
            circle.set('cx', str(px))
            circle.set('cy', str(py))
            circle.set('r', str(r))

        # 添加标题
        title = ET.SubElement(svg, 'text')
        title.set('class', 'title')
        title.set('x', str(view_min_x + view_width / 2))
        title.set('y', str(view_min_y + view_height * 0.05))
        title.text = 'Grid Map'

        # 保存 SVG
        tree = ET.ElementTree(svg)
        ET.indent(tree, space='  ')
        tree.write(output_path, encoding='utf-8', xml_declaration=True)

        print(f"SVG 地图已保存到: {output_path}")
        return output_path


def convert_map(
    input_path: str,
    output_path: Optional[str] = None,
    format: str = "png",
    point_size: float = 1.0,
    dpi: int = 150,
) -> str:
    """
    便捷函数：转换地图文件

    参数:
        input_path: 输入的 txt 地图文件路径
        output_path: 输出文件路径 (可选)
        format: 输出格式 (png 或 svg)
        point_size: 点的大小倍数
        dpi: PNG 分辨率

    返回:
        输出文件路径
    """
    converter = MapConverter()
    map_data = converter.parse_grid_map(input_path)

    if output_path is None:
        input_path_obj = Path(input_path)
        output_path = str(input_path_obj.with_suffix(f'.{format}'))

    if format.lower() == "png":
        return converter.to_png(map_data, output_path, point_size, dpi)
    elif format.lower() == "svg":
        return converter.to_svg(map_data, output_path, point_size)
    else:
        raise ValueError(f"不支持的格式: {format}")


if __name__ == "__main__":
    # 测试代码
    import sys

    if len(sys.argv) > 1:
        input_file = sys.argv[1]
        convert_map(input_file)
    else:
        # 默认测试
        test_file = "/home/jetson/sweetbomb/data/map/map_gridmap.txt"
        if Path(test_file).exists():
            convert_map(test_file)
        else:
            print(f"测试文件不存在: {test_file}")
