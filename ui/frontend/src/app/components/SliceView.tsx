import { useEffect, useRef, useState } from 'react';
import { Layers } from 'lucide-react';

interface SliceViewProps {
    isActive: boolean;
    pointCloudData: Float32Array | null;
    // 新增：后端分析结果（可选，如果提供则直接使用，否则自行计算）
    pointCloudAnalysis?: {
        volume: {
            current: number;
            target: number;
            reached: boolean;
        };
        distances: {
            nearest_material: number | null;
            nearest_x: number | null;
            nearest_y: number | null;
        };
        target_depth: {
            x: number | null;
        };
        pile_height?: number;  // 堆体高度 (m)
        has_material: boolean;
        timestamp: number;
    } | null;
}

interface SliceSettings {
    teethHeight: number;   // Z1: 铲齿放平时的高度 (米)
    cameraToTeeth: number;  // 相机到铲齿前沿距离 (米)
    bucketDepth: number;    // 铲斗深度 (米)
    bucketVolume: number;   // 铲斗目标体积 (升，默认30L)
}


interface AdvanceInfo {
    nearestX: number;        // 最近物料的 X 坐标
    nearestY: number;        // 最近物料的 Y 坐标
    materialDistance: number; // 铲齿到物料的距离
    advanceDistance: number;  // 需要前进的总距离
}

// 30L 体积目标计算结果
interface VolumeTargetInfo {
    targetVolume: number;      // 目标体积 0.03 m³ (30L)
    bucketWidth: number;       // 铲斗宽度 0.6m (600mm)
    targetDepthX: number;      // 达到 30L 时的 X 深度 (从相机起算)
    actualVolume: number;      // 实际可达到的体积
    isReachable: boolean;      // 是否能在当前视图中达到 30L
    fillDepth: number;         // 需要填充的深度 (从铲齿起算)
}

// ----- 固定参数 -----
const CELL_SIZE = 0.02;          // 格子大小 0.02m
const DEFAULT_VIEW_DEPTH = 2.0;  // 默认观测深度 2.0m (X轴显示范围加长到2m)
const VIEW_MIN_Y = -1.5;           // Y(宽度) 固定显示范围起点 -1.5m
const VIEW_MAX_Y = 1.5;            // Y(宽度) 固定显示范围终点 1.5m
const BUCKET_HEIGHT = 0.3;         // 铲斗高度 300mm (Z2 - Z1)
const FOCUS_HALF_WIDTH = 0.3;      // 焦点区域半宽 600mm/2 = 300mm

// 向后兼容：导出旧常量名用于外部引用 (实际使用 viewMinX/viewMaxX 动态值)
export const VIEW_MIN_X = 1.0;
export const VIEW_MAX_X = 2.0;

const DEFAULT_SETTINGS: SliceSettings = {
    teethHeight: -0.1,   // 默认铲齿高度
    cameraToTeeth: 0.8,  // 默认相机到铲齿 0.8m
    bucketDepth: 0.3,    // 默认铲斗深度 0.3m
    bucketVolume: 30,    // 默认铲斗目标体积 30L
};

const STORAGE_KEY = 'slice-view-settings-v3';

function loadSettings(): SliceSettings {
    try {
        const saved = localStorage.getItem(STORAGE_KEY);
        if (saved) return { ...DEFAULT_SETTINGS, ...JSON.parse(saved) };
    } catch (e) {
        console.warn('Failed to load slice settings:', e);
    }
    return DEFAULT_SETTINGS;
}

function saveSettings(settings: SliceSettings) {
    try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
    } catch (e) {
        console.warn('Failed to save slice settings:', e);
    }
}

// 5 级橙黄渐变色（低 → 高）— RGB 元组用于 ImageData 高性能渲染
const HEATMAP_COLORS_RGB: [number, number, number][] = [
    [204, 85, 0],    // 深橙 #CC5500
    [230, 115, 0],   // 橙 #E67300
    [253, 128, 46],  // 品牌橙 #FD802E
    [255, 169, 64],  // 琥珀 #FFA940
    [255, 215, 0],   // 金黄 #FFD700
];
const GRAY_RGB: [number, number, number] = [58, 58, 60]; // #3a3a3c

// CSS 颜色字符串（仅用于图例等非性能敏感区域）
// const HEATMAP_COLORS = ['#CC5500', '#E67300', '#FD802E', '#FFA940', '#FFD700'];
// const GRAY_COLOR = '#3a3a3c';

// 根据高度获取 RGB 颜色（用于 ImageData 高性能路径）
function getColorRGBForHeight(height: number, minH: number, maxH: number, inFocus: boolean): [number, number, number] {
    if (!inFocus) return GRAY_RGB;

    const range = maxH - minH;
    if (range <= 0) return HEATMAP_COLORS_RGB[2];

    const normalized = (height - minH) / range;
    const clamped = Math.max(0, Math.min(1, normalized));
    const idx = Math.min(4, Math.floor(clamped * 5));
    return HEATMAP_COLORS_RGB[idx];
}

// 生成模拟点云数据（开发调试用，有真实数据时可删除）
function generateMockPointCloud(): Float32Array {
    const points: number[] = [];
    // 在 X=1~3m, Y=-1.0~1.0m 范围内生成模拟地形
    for (let x = 1.0; x <= 3.0; x += 0.05) {
        for (let y = -1.0; y <= 1.0; y += 0.05) {
            // 模拟起伏地形：中心高、边缘低
            const distFromCenter = Math.sqrt((x - 2.0) ** 2 + y ** 2);
            const baseHeight = -0.05 + 0.2 * Math.exp(-distFromCenter * 1.5);
            // 添加随机噪声
            const noise = (Math.random() - 0.5) * 0.04;
            const z = baseHeight + noise;
            points.push(x, y, z);
        }
    }
    return new Float32Array(points);
}

// 缓存模拟数据避免每帧重新生成
let _mockData: Float32Array | null = null;
function getMockPointCloud(): Float32Array {
    if (!_mockData) _mockData = generateMockPointCloud();
    return _mockData;
}

export function SliceView({ isActive, pointCloudData, pointCloudAnalysis }: SliceViewProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const [settings, setSettings] = useState<SliceSettings>(loadSettings);
    const [isHovered, setIsHovered] = useState(false);
    const [pointCount, setPointCount] = useState(0);
    const [advanceInfo, setAdvanceInfo] = useState<AdvanceInfo | null>(null);
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    const [_volumeInfo, _setVolumeInfo] = useState<VolumeTargetInfo | null>(null);

    // Z1 = 铲齿高度, Z2 = Z1 + 铲斗高度 300mm
    const z1 = settings.teethHeight;
    const z2 = z1 + BUCKET_HEIGHT;

    // 动态计算 X 轴视图范围
    const viewMinX = settings.cameraToTeeth;
    const viewMaxX = settings.cameraToTeeth + DEFAULT_VIEW_DEPTH;

    // 无真实数据时使用模拟数据（开发调试）。如果接入了真机，activeData 就是真实点云。
    const activeData = (pointCloudData && pointCloudData.length > 0) ? pointCloudData : getMockPointCloud();
    const dataAvailable = activeData.length > 0;

    // 设置变化时持久化
    useEffect(() => {
        const timer = setTimeout(() => saveSettings(settings), 500);
        return () => clearTimeout(timer);
    }, [settings]);

    // 绘制 BEV 切片热力图
    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas || !dataAvailable) {
            setPointCount(0);
            if (canvas) {
                const ctx = canvas.getContext('2d');
                if (ctx) {
                    ctx.fillStyle = '#1c1c1e';
                    ctx.fillRect(0, 0, canvas.width, canvas.height);
                }
            }
            return;
        }

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        ctx.fillStyle = '#1c1c1e';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        const minHeight = z1;
        const maxHeight = z2;
        const fixedMinX = viewMinX;
        const fixedMaxX = viewMaxX;

        // Y 轴使用固定范围
        const minY = VIEW_MIN_Y;
        const maxY = VIEW_MAX_Y;

        // 第一遍：过滤点并统计实际 Z 范围，同时找出最近的物料点
        let actualMinZ = Infinity, actualMaxZ = -Infinity;
        let filteredCount = 0;
        let nearestX = Infinity;
        let nearestY = 0;

        // 最近物料点的工作范围定义：
        // X方向：铲齿前方完整2m范围（观测范围）
        // Y方向：只在铲斗宽度600mm范围内（-0.3m ~ +0.3m）
        const BUCKET_HALF_WIDTH = 0.3; // 铲斗半宽 300mm

        for (let i = 0; i < activeData.length; i += 3) {
            const x = activeData[i];
            const y = activeData[i + 1];
            const z = activeData[i + 2];

            // 主过滤：在完整视野范围内统计
            if (x >= fixedMinX && x <= fixedMaxX && y >= minY && y <= maxY && z >= minHeight && z <= maxHeight) {
                actualMinZ = Math.min(actualMinZ, z);
                actualMaxZ = Math.max(actualMaxZ, z);
                filteredCount++;

                // 只在铲斗宽度范围内（Y方向±0.3m）追踪最近物料点
                if (y >= -BUCKET_HALF_WIDTH && y <= BUCKET_HALF_WIDTH) {
                    if (x < nearestX) {
                        nearestX = x;
                        nearestY = y;
                    }
                }
            }
        }

        setPointCount(filteredCount);

        // 计算前进距离
        if (nearestX !== Infinity) {
            const materialDistance = nearestX - settings.cameraToTeeth;
            const advanceDistance = materialDistance + settings.bucketDepth;
            setAdvanceInfo({
                nearestX,
                nearestY,
                materialDistance,
                advanceDistance,
            });
        } else {
            setAdvanceInfo(null);
        }

        if (filteredCount === 0) {
            return;
        }

        // 固定网格参数
        const xRange = fixedMaxX - fixedMinX;
        const yRange = VIEW_MAX_Y - VIEW_MIN_Y;

        const gridCols = Math.max(1, Math.ceil(yRange / CELL_SIZE));
        const gridRows = Math.ceil(xRange / CELL_SIZE);  // 10 行 (2m / 0.2m)
        const heightGrid = new Float32Array(gridRows * gridCols).fill(-Infinity);

        // 第二遍：填充高度网格
        for (let i = 0; i < activeData.length; i += 3) {
            const x = activeData[i];
            const y = activeData[i + 1];
            const z = activeData[i + 2];

            if (x >= fixedMinX && x <= fixedMaxX && y >= minY && y <= maxY && z >= minHeight && z <= maxHeight) {
                const col = Math.min(Math.floor((VIEW_MAX_Y - y) / CELL_SIZE), gridCols - 1);
                const row = Math.min(Math.floor((x - fixedMinX) / CELL_SIZE), gridRows - 1);
                const idx = row * gridCols + col;

                if (z > heightGrid[idx]) {
                    heightGrid[idx] = z;
                }
            }
        }

        // 固定视口缩放
        const padding = 40;
        const availableWidth = canvas.width - 2 * padding;
        const availableHeight = canvas.height - 2 * padding;
        const dataWidth = Math.max(0.1, yRange);
        const dataHeight = Math.max(0.1, xRange);

        const dataAspectRatio = dataWidth / dataHeight;
        const canvasAspectRatio = availableWidth / availableHeight;

        let scale: number, offsetX: number, offsetY: number;
        if (dataAspectRatio > canvasAspectRatio) {
            scale = availableWidth / dataWidth;
            offsetX = padding;
            offsetY = padding + (availableHeight - dataHeight * scale) / 2;
        } else {
            scale = availableHeight / dataHeight;
            offsetX = padding + (availableWidth - dataWidth * scale) / 2;
            offsetY = padding;
        }

        const cellSizePixels = CELL_SIZE * scale;

        // 焦点区域：Y 中心 ±400mm (共 800mm)
        const yCenter = (minY + maxY) / 2;
        const focusMinY = yCenter - FOCUS_HALF_WIDTH;
        const focusMaxY = yCenter + FOCUS_HALF_WIDTH;

        // 使用 ImageData 像素缓冲高性能渲染热力图（替代 20,000+ 次 fillRect）
        const heatmapWidthPx = Math.ceil(gridCols * cellSizePixels);
        const heatmapHeightPx = Math.ceil(gridRows * cellSizePixels);

        if (heatmapWidthPx > 0 && heatmapHeightPx > 0) {
            const imgData = ctx.createImageData(heatmapWidthPx, heatmapHeightPx);
            const pixels = imgData.data;

            for (let row = 0; row < gridRows; row++) {
                for (let col = 0; col < gridCols; col++) {
                    const idx = row * gridCols + col;
                    const z = heightGrid[idx];

                    if (z > -Infinity) {
                        const cellCenterY = VIEW_MAX_Y - (col + 0.5) * CELL_SIZE;
                        const inFocus = cellCenterY >= focusMinY && cellCenterY <= focusMaxY;
                        const [r, g, b] = getColorRGBForHeight(z, minHeight, maxHeight, inFocus);

                        // 计算此格子在 ImageData 中的像素区域
                        const pxLeft = Math.floor(col * cellSizePixels);
                        const pxTop = Math.floor((gridRows - 1 - row) * cellSizePixels);
                        const pxRight = Math.min(Math.ceil((col + 1) * cellSizePixels), heatmapWidthPx);
                        const pxBottom = Math.min(Math.ceil((gridRows - row) * cellSizePixels), heatmapHeightPx);

                        for (let py = pxTop; py < pxBottom; py++) {
                            for (let px = pxLeft; px < pxRight; px++) {
                                const pixelIdx = (py * heatmapWidthPx + px) * 4;
                                pixels[pixelIdx] = r;
                                pixels[pixelIdx + 1] = g;
                                pixels[pixelIdx + 2] = b;
                                pixels[pixelIdx + 3] = 255;
                            }
                        }
                    }
                }
            }

            ctx.putImageData(imgData, Math.round(offsetX), Math.round(offsetY));
        }

        // 仅在格子像素足够大时绘制网格线（避免密集时 300+ 次无意义 stroke）
        if (cellSizePixels >= 4) {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
            ctx.lineWidth = 0.5;
            for (let row = 0; row <= gridRows; row++) {
                const cy = offsetY + (gridRows - row) * cellSizePixels;
                ctx.beginPath();
                ctx.moveTo(offsetX, cy);
                ctx.lineTo(offsetX + gridCols * cellSizePixels, cy);
                ctx.stroke();
            }
            for (let col = 0; col <= gridCols; col++) {
                const cx = offsetX + col * cellSizePixels;
                ctx.beginPath();
                ctx.moveTo(cx, offsetY);
                ctx.lineTo(cx, offsetY + gridRows * cellSizePixels);
                ctx.stroke();
            }
        }

        // X 轴刻度标注（左侧）— 格子很小，按 0.5m 步进标注
        ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
        ctx.font = '9px monospace';
        ctx.textAlign = 'right';
        // 动态步进：根据显示范围选择合适的刻度间隔
        const xRangeMeters = fixedMaxX - fixedMinX;
        const xStep = xRangeMeters <= 1.0 ? 0.2 : (xRangeMeters <= 2.0 ? 0.5 : 1.0);
        for (let xVal = fixedMinX; xVal <= fixedMaxX + 0.001; xVal += xStep) {
            const row = Math.round((xVal - fixedMinX) / CELL_SIZE);
            const cy = offsetY + (gridRows - row) * cellSizePixels;
            ctx.fillText(`${xVal.toFixed(1)}m`, offsetX - 4, cy + 3);
        }

        // Y 轴刻度标注（底部）— 按 0.5m 步进标注
        ctx.textAlign = 'center';
        for (let yVal = VIEW_MIN_Y; yVal <= VIEW_MAX_Y + 0.001; yVal += 0.5) {
            const col = Math.round((VIEW_MAX_Y - yVal) / CELL_SIZE);
            const cx = offsetX + col * cellSizePixels;
            ctx.fillText(`${yVal.toFixed(1)}`, cx, offsetY + gridRows * cellSizePixels + 14);
        }

        // 绘制焦点区域边界线
        const focusLeftPx = offsetX + Math.max(0, (maxY - focusMaxY)) * scale;
        const focusRightPx = offsetX + Math.min(yRange, (maxY - focusMinY)) * scale;
        ctx.strokeStyle = 'rgba(253, 128, 46, 0.3)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        // 左边界
        ctx.beginPath();
        ctx.moveTo(focusLeftPx, offsetY);
        ctx.lineTo(focusLeftPx, offsetY + gridRows * cellSizePixels);
        ctx.stroke();
        // 右边界
        ctx.beginPath();
        ctx.moveTo(focusRightPx, offsetY);
        ctx.lineTo(focusRightPx, offsetY + gridRows * cellSizePixels);
        ctx.stroke();
        ctx.setLineDash([]);

        // 焦点区域标签
        ctx.fillStyle = 'rgba(253, 128, 46, 0.5)';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('600mm焦点区域', (focusLeftPx + focusRightPx) / 2, offsetY - 4);

        // 指南针
        const compassCX = canvas.width - 40;
        const compassCY = 40;
        ctx.strokeStyle = 'rgba(253, 128, 46, 0.5)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(compassCX, compassCY, 25, 0, Math.PI * 2);
        ctx.stroke();

        ctx.fillStyle = '#FD802E';
        ctx.beginPath();
        ctx.moveTo(compassCX, compassCY - 18);
        ctx.lineTo(compassCX - 6, compassCY - 6);
        ctx.lineTo(compassCX + 6, compassCY - 6);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = 'rgba(253, 128, 46, 0.7)';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('前', compassCX, compassCY - 22);

        ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
        ctx.font = '8px monospace';
        ctx.fillText('X深度', compassCX, compassCY + 20);

    }, [activeData, z1, z2]);

    const handleResetView = () => {
        setSettings(DEFAULT_SETTINGS);
    };

    return (
        <div
            className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md"
            onMouseEnter={() => setIsHovered(true)}
            onMouseLeave={() => setIsHovered(false)}
        >
            {/* 顶部状态胶囊 */}
            <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
                {isActive ? (
                    <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
                ) : (
                    <div className="w-3 h-3 bg-slate-500 rounded-full" />
                )}
                <Layers className="w-3.5 h-3.5 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> BEV SLICE </span>
                <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
                    | {pointCount.toLocaleString()} PTS
                </span>
                <span className="text-[10px] text-slate-500 border-l border-slate-600 pl-2 ml-1 font-mono">
                    X:{viewMinX.toFixed(1)}~{viewMaxX.toFixed(1)}m Y:{VIEW_MIN_Y}~{VIEW_MAX_Y}m
                </span>
            </div>

            {/* Canvas */}
            <canvas
                ref={canvasRef}
                width={600}
                height={330}
                className="w-full h-full"
            />

            {/* 控制面板（移至右下角并减小宽度） */}
            <div className="absolute bottom-[10px] right-[10px] z-[100] bg-black/80 backdrop-blur-md p-2 rounded-lg border border-slate-700 max-w-[130px]">
                {/* 相机到铲齿距离 */}
                <div className="text-[9px] text-slate-400 font-mono mb-2">安装参数</div>
                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[8px] text-[#FD802E] font-mono w-9">相机→齿</span>
                    <input
                        type="number"
                        step="0.05"
                        value={settings.cameraToTeeth.toFixed(2)}
                        onChange={(e) => setSettings(prev => ({ ...prev, cameraToTeeth: parseFloat(e.target.value) || 0 }))}
                        className="w-[32px] h-4 bg-transparent border border-[#FD802E] rounded text-[9px] text-[#FD802E] text-center font-mono focus:outline-none"
                    />
                    <span className="text-[8px] text-slate-500 font-mono">m</span>
                </div>

                {/* 铲斗深度 */}
                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[8px] text-[#FD802E] font-mono w-9">铲斗深度</span>
                    <input
                        type="number"
                        step="0.05"
                        value={settings.bucketDepth.toFixed(2)}
                        onChange={(e) => setSettings(prev => ({ ...prev, bucketDepth: parseFloat(e.target.value) || 0 }))}
                        className="w-[32px] h-4 bg-transparent border border-[#FD802E] rounded text-[9px] text-[#FD802E] text-center font-mono focus:outline-none"
                    />
                    <span className="text-[8px] text-slate-500 font-mono">m</span>
                </div>

                {/* 铲斗目标体积 */}
                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[8px] text-green-400 font-mono w-9">目标体积</span>
                    <input
                        type="number"
                        step="5"
                        value={Math.round(settings.bucketVolume)}
                        onChange={(e) => setSettings(prev => ({ ...prev, bucketVolume: parseFloat(e.target.value) || 30 }))}
                        className="w-[32px] h-4 bg-transparent border border-[#FD802E] rounded text-[9px] text-[#FD802E] text-center font-mono focus:outline-none"
                    />
                    <span className="text-[8px] text-slate-500 font-mono">L</span>
                </div>

                {/* 堆体高度显示 */}
                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[8px] text-[#FD802E] font-mono w-9">堆体高度</span>
                    <span className="text-[9px] text-[#FD802E] font-mono w-[32px] text-right">
                        {(() => {
                            // 优先使用后端传来的分析结果
                            if (pointCloudAnalysis?.pile_height !== undefined) {
                                return pointCloudAnalysis.pile_height.toFixed(2);
                            }
                            return '--';
                        })()}
                    </span>
                    <span className="text-[8px] text-slate-500 font-mono">m</span>
                </div>

                {/* Z1 铲齿高度设置 */}
                <div className="border-t border-slate-600 pt-2 mb-2">
                    <div className="text-[9px] text-slate-400 font-mono mb-1">切片高度</div>
                    <div className="flex flex-col gap-1">
                        <div className="flex items-center gap-1">
                            <span className="text-[8px] text-[#FD802E] font-mono w-6">Z1</span>
                            <input
                                type="number"
                                step="0.01"
                                value={settings.teethHeight.toFixed(2)}
                                onChange={(e) => setSettings(prev => ({ ...prev, teethHeight: parseFloat(e.target.value) || 0 }))}
                                className="w-[32px] h-4 bg-transparent border border-[#FD802E] rounded text-[9px] text-[#FD802E] text-center font-mono focus:outline-none"
                            />
                        </div>
                        <span className="text-[7px] text-slate-500 font-mono">→ Z2: {z2.toFixed(2)}</span>
                    </div>
                </div>



                {/* 前进距离计算结果 */}
                {advanceInfo && (
                    <div className="border-t border-[#FD802E]/30 pt-2 mb-2">
                        <div className="text-[9px] text-[#FD802E] font-mono mb-1.5">作业建议</div>
                        <div className="flex items-center justify-between mb-1">
                            <span className="text-[8px] text-slate-500 font-mono">铲齿→物料</span>
                            <span className="text-[9px] text-cyan-400 font-mono">{advanceInfo.materialDistance.toFixed(2)}m</span>
                        </div>
                        <div className="flex items-center justify-between">
                            <span className="text-[8px] text-slate-500 font-mono">建议前进</span>
                            <span className="text-[12px] text-[#FD802E] font-mono font-bold pr-1">{advanceInfo.advanceDistance.toFixed(2)}m</span>
                        </div>
                    </div>
                )}



                {/* 重置按钮 */}
                <button
                    onClick={handleResetView}
                    className="w-full px-2 py-1 mt-1 bg-slate-700 hover:bg-slate-600 rounded text-[9px] text-slate-300 font-mono transition-colors"
                >
                    重置
                </button>
            </div>

            {/* {isHovered && (
                <div className="absolute top-[50px] right-[10px] z-10 bg-black/80 backdrop-blur-md px-3 py-2 rounded-lg text-[9px] text-slate-400 font-mono">
                    <div className="flex items-center gap-2">
                        <span className="text-[#FD802E]">●</span>
                        <span>俯视热力图</span>
                    </div>
                    <div className="flex items-center gap-2 mt-1">
                        <span className="text-cyan-400">↑</span>
                        <span>X 深度 {viewMinX.toFixed(1)}~{viewMaxX.toFixed(1)}m</span>
                    </div>
                    <div className="flex items-center gap-2 mt-1">
                        <span className="text-green-400">→</span>
                        <span>Y 宽度 {VIEW_MIN_Y}~{VIEW_MAX_Y}m</span>
                    </div>
                    <div className="flex items-center gap-2 mt-1">
                        <span className="text-slate-500">▦</span>
                        <span>格子 {(CELL_SIZE * 100).toFixed(0)}cm</span>
                    </div>
                </div>
            )} */}

            {/* 未连接后端时，仍让底层 canvas 显示出来，给一个半透明遮罩说明状态 */}
            {!isActive && (
                <div className="absolute inset-0 flex items-center justify-center bg-black/30 pointer-events-none z-20">
                    <div className="text-center bg-black/60 p-4 rounded-xl backdrop-blur-md border border-slate-700 pointer-events-auto">
                        <div className="w-12 h-12 rounded-full bg-slate-700/50 flex items-center justify-center mx-auto mb-2">
                            <Layers className="w-6 h-6 text-slate-400" />
                        </div>
                        <p className="text-slate-300 text-sm font-bold">无设备连接</p>
                        <p className="text-[#FD802E] text-xs mt-1">目前展示 Mock 模拟数据测试效果</p>
                    </div>
                </div>
            )}
        </div>
    );
}
