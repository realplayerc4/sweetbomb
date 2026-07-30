# 机器人地图目录

此目录用于存放机器人的栅格地图数据，支持自动将 txt 格式转换为 PNG/SVG 图片。

## 文件说明

| 文件 | 说明 |
|------|------|
| `map_gridmap.txt` | 原始栅格地图数据（txt格式，MATLAB生成） |
| `map_gridmap.png` | 转换后的PNG图片（150 DPI） |
| `map_gridmap_clean.png` | 高清PNG图片（300 DPI） |
| `cache/` | PNG缓存目录 |

## 地图格式

txt文件格式：
```
第一行：分辨率 (meters/pixel)
第二行：原点坐标 x y
第三行：栅格宽高 width height
第四行起：占据栅格坐标 x y (每行一个点)
```

示例：
```
0.025
-49.825 -21.85
3401 1089
127 3399
128 3399
129 3399
...
```

## 转换脚本

使用 `tools/map_converter/txt_to_image.py` 将txt转换为PNG/SVG：

```bash
# 转换为PNG
python tools/map_converter/txt_to_image.py data/map/map_gridmap.txt output/map.png --point-size 0.8 --dpi 300

# 转换为SVG
python tools/map_converter/txt_to_image.py data/map/map_gridmap.txt output/map.svg --format svg
```

## 后端API

地图API端点（FastAPI）：

- `GET /api/map/` - 获取所有地图列表
- `GET /api/map/{name}.png` - 获取地图PNG图片（自动转换和缓存）
- `GET /api/map/{name}/info` - 获取地图详细信息
- `POST /api/map/cache/clear` - 清理缓存

请求参数：
- `refresh` (bool) - 强制重新生成缓存
- `point_size` (float) - 点大小倍数 (0.1-5.0)
- `dpi` (int) - 图片分辨率 (72-300)

## 前端使用

在HTML中直接显示地图：

```html
<!-- 直接显示PNG -->
<img src="/api/map/map_gridmap.png" alt="Grid Map" style="max-width: 100%; height: auto;">

<!-- 支持放大缩小 -->
<div style="overflow: auto;">
  <img src="/api/map/map_gridmap.png?dpi=300" alt="Grid Map">
</div>
```

## 颜色配置

地图使用石墨橙色主题：
- 背景色: `#1f2937` (深灰蓝)
- 点颜色: `#c25e1f` (石墨橙)
- 网格线: `#4b5563` (灰色)
- 文字: `#f3f4f6` (白色)
