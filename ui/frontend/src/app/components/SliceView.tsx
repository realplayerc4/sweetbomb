import { useEffect, useRef, useState } from 'react';
import { Layers } from 'lucide-react';

interface SliceViewProps {
    isActive: boolean;
    pointCloudData: Float32Array | null;
}

interface SliceSettings {
    teethHeight: number;  // Z1: 铲齿放平时的高度 (米)
}

// 点云选取范围统计信息
interface SliceStats {
    widthY: number;      // Y 轴跨度 (宽度, 米)
    depthX: number;      // X 轴跨度 (深度, 米)
    z1: number;          // 铲齿高度 (Z1)
    z2: number;          // 铲齿 + 斗高 (Z2 = Z1 + 0.3m)
    actualMinZ: number;  // 切片内实际最低 Z 值
    actualMaxZ: number;  // 切片内实际最高 Z 值
}

// ----- 固定参数 -----
const CELL_SIZE = 0.2;        // 格子大小 0.2m
const VIEW_MIN_X = 1.0;       // X(深度) 固定显示范围起点
const VIEW_MAX_X = 3.0;       // X(深度) 固定显示范围终点
const BUCKET_HEIGHT = 0.3;    // 铲斗高度 300mm (Z2 - Z1)
const FOCUS_HALF_WIDTH = 0.4; // 焦点区域半宽 800mm/2 = 400mm

const DEFAULT_SETTINGS: SliceSettings = {
    teethHeight: -0.1,  // 默认铲齿高度
};

const STORAGE_KEY = 'slice-view-settings-v2';

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

// 5 级橙黄渐变色（低 → 高）
const HEATMAP_COLORS = [
    '#CC5500',  // 深橙
    '#E67300',  // 橙
    '#FD802E',  // 品牌橙
    '#FFA940',  // 琥珀
    '#FFD700',  // 金黄
];
const GRAY_COLOR = '#3a3a3c';  // 非焦点区域灰色

// 根据高度获取 5 级颜色索引
function getColorForHeight(height: number, minH: number, maxH: number, inFocus: boolean): string {
    if (!inFocus) return GRAY_COLOR;

    const range = maxH - minH;
    if (range <= 0) return HEATMAP_COLORS[2];

    const normalized = (height - minH) / range;
    const clamped = Math.max(0, Math.min(1, normalized));
    const idx = Math.min(4, Math.floor(clamped * 5));
    return HEATMAP_COLORS[idx];
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

export function SliceView({ isActive, pointCloudData }: SliceViewProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const [settings, setSettings] = useState<SliceSettings>(loadSettings);
    const [isHovered, setIsHovered] = useState(false);
    const [pointCount, setPointCount] = useState(0);
    const [stats, setStats] = useState<SliceStats | null>(null);

    // Z1 = 铲齿高度, Z2 = Z1 + 铲斗高度 300mm
    const z1 = settings.teethHeight;
    const z2 = z1 + BUCKET_HEIGHT;

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
            setStats(null);
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
        const fixedMinX = VIEW_MIN_X;
        const fixedMaxX = VIEW_MAX_X;

        // 第一遍：过滤点并统计 Y 范围和实际 Z 范围
        let minY = Infinity, maxY = -Infinity;
        let actualMinZ = Infinity, actualMaxZ = -Infinity;
        let filteredCount = 0;

        for (let i = 0; i < activeData.length; i += 3) {
            const x = activeData[i];
            const y = activeData[i + 1];
            const z = activeData[i + 2];

            if (x >= fixedMinX && x <= fixedMaxX && z >= minHeight && z <= maxHeight) {
                minY = Math.min(minY, y);
                maxY = Math.max(maxY, y);
                actualMinZ = Math.min(actualMinZ, z);
                actualMaxZ = Math.max(actualMaxZ, z);
                filteredCount++;
            }
        }

        setPointCount(filteredCount);

        if (filteredCount === 0 || !isFinite(minY)) {
            setStats(null);
            return;
        }

        // 固定网格参数
        const xRange = fixedMaxX - fixedMinX;
        const yRange = maxY - minY;

        const gridCols = Math.max(1, Math.ceil(yRange / CELL_SIZE));
        const gridRows = Math.ceil(xRange / CELL_SIZE);  // 10 行 (2m / 0.2m)
        const heightGrid = new Float32Array(gridRows * gridCols).fill(-Infinity);

        // 第二遍：填充高度网格
        for (let i = 0; i < activeData.length; i += 3) {
            const x = activeData[i];
            const y = activeData[i + 1];
            const z = activeData[i + 2];

            if (x >= fixedMinX && x <= fixedMaxX && z >= minHeight && z <= maxHeight) {
                const col = Math.min(Math.floor((y - minY) / CELL_SIZE), gridCols - 1);
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

        // 绘制网格线
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

        // 绘制热力图网格
        for (let row = 0; row < gridRows; row++) {
            for (let col = 0; col < gridCols; col++) {
                const idx = row * gridCols + col;
                const z = heightGrid[idx];

                if (z > -Infinity) {
                    const cellCenterY = minY + (col + 0.5) * CELL_SIZE;
                    const inFocus = cellCenterY >= focusMinY && cellCenterY <= focusMaxY;

                    const canvasX = offsetX + col * cellSizePixels;
                    const canvasY = offsetY + (gridRows - 1 - row) * cellSizePixels;

                    ctx.fillStyle = getColorForHeight(z, minHeight, maxHeight, inFocus);
                    ctx.fillRect(canvasX, canvasY, cellSizePixels + 0.5, cellSizePixels + 0.5);
                }
            }
        }

        // X 轴刻度标注（左侧）
        ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
        ctx.font = '9px monospace';
        ctx.textAlign = 'right';
        for (let row = 0; row <= gridRows; row++) {
            const xVal = fixedMinX + row * CELL_SIZE;
            const cy = offsetY + (gridRows - row) * cellSizePixels;
            if (Math.abs(xVal % 1.0) < 0.01 || row === 0 || row === gridRows) {
                ctx.fillText(`${xVal.toFixed(1)}m`, offsetX - 4, cy + 3);
            }
        }

        // Y 轴刻度标注（底部）
        ctx.textAlign = 'center';
        const yStep = Math.max(1, Math.floor(gridCols / 6));
        for (let col = 0; col <= gridCols; col += yStep) {
            const yVal = minY + col * CELL_SIZE;
            const cx = offsetX + col * cellSizePixels;
            ctx.fillText(`${yVal.toFixed(1)}`, cx, offsetY + gridRows * cellSizePixels + 14);
        }

        // 绘制焦点区域边界线
        const focusLeftPx = offsetX + Math.max(0, (focusMinY - minY)) * scale;
        const focusRightPx = offsetX + Math.min(yRange, (focusMaxY - minY)) * scale;
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
        ctx.fillText('800mm焦点区域', (focusLeftPx + focusRightPx) / 2, offsetY - 4);

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

        // 更新统计
        setStats({
            widthY: yRange,
            depthX: xRange,
            z1,
            z2,
            actualMinZ,
            actualMaxZ,
        });

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
                    X:{VIEW_MIN_X}~{VIEW_MAX_X}m
                </span>
            </div>

            {/* Canvas */}
            <canvas
                ref={canvasRef}
                width={600}
                height={330}
                className="w-full h-full"
            />

            {/* 左下角控制面板 */}
            <div className="absolute bottom-[10px] left-[10px] z-10 bg-black/80 backdrop-blur-md p-3 rounded-lg border border-slate-700 max-w-[210px]">
                {/* Z1 铲齿高度设置 */}
                <div className="text-[9px] text-slate-400 font-mono mb-2">铲齿高度 Z1 (米)</div>
                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[9px] text-[#FD802E] font-mono w-6">Z1</span>
                    <input
                        type="number"
                        step="0.01"
                        value={settings.teethHeight.toFixed(2)}
                        onChange={(e) => setSettings(prev => ({ ...prev, teethHeight: parseFloat(e.target.value) || 0 }))}
                        className="w-16 h-5 bg-slate-800 border border-slate-600 rounded text-[9px] text-white px-1 font-mono"
                    />
                    <span className="text-[8px] text-slate-500 font-mono">→ Z2: {z2.toFixed(2)}</span>
                </div>

                {/* 固定参数 */}
                <div className="border-t border-slate-600 pt-2 mb-2">
                    <div className="text-[9px] text-slate-400 font-mono mb-1">固定参数</div>
                    <div className="space-y-0.5 text-[8px] font-mono">
                        <div className="flex justify-between">
                            <span className="text-slate-500">深度</span>
                            <span className="text-white">{VIEW_MIN_X}~{VIEW_MAX_X}m</span>
                        </div>
                        <div className="flex justify-between">
                            <span className="text-slate-500">格子</span>
                            <span className="text-white">{(CELL_SIZE * 100).toFixed(0)}cm</span>
                        </div>
                        <div className="flex justify-between">
                            <span className="text-slate-500">焦点宽度</span>
                            <span className="text-white">{FOCUS_HALF_WIDTH * 2 * 1000}mm</span>
                        </div>
                    </div>
                </div>

                {/* 选取范围统计 */}
                {stats && (
                    <div className="border-t border-slate-600 pt-2 mb-2">
                        <div className="text-[9px] text-slate-400 font-mono mb-1.5">选取范围</div>
                        <div className="grid grid-cols-2 gap-x-3 gap-y-1">
                            <div className="flex items-center justify-between">
                                <span className="text-[8px] text-slate-500 font-mono">宽度Y</span>
                                <span className="text-[9px] text-green-400 font-mono">{stats.widthY.toFixed(2)}m</span>
                            </div>
                            <div className="flex items-center justify-between">
                                <span className="text-[8px] text-slate-500 font-mono">深度X</span>
                                <span className="text-[9px] text-cyan-400 font-mono">{stats.depthX.toFixed(1)}m</span>
                            </div>
                            <div className="flex items-center justify-between">
                                <span className="text-[8px] text-slate-500 font-mono">Z1低</span>
                                <span className="text-[9px] text-[#CC5500] font-mono">{stats.z1.toFixed(2)}m</span>
                            </div>
                            <div className="flex items-center justify-between">
                                <span className="text-[8px] text-slate-500 font-mono">Z2高</span>
                                <span className="text-[9px] text-[#FFD700] font-mono">{stats.z2.toFixed(2)}m</span>
                            </div>
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

            {/* 右下角：颜色图例 */}
            <div className="absolute bottom-[10px] right-[10px] z-10 bg-black/80 backdrop-blur-md p-2 rounded-lg border border-slate-700">
                <div className="text-[9px] text-slate-400 font-mono mb-1">高度 (Z1→Z2)</div>
                <div className="flex items-center gap-0.5">
                    <span className="text-[8px] text-[#CC5500] font-mono">低</span>
                    {HEATMAP_COLORS.map((color, i) => (
                        <div key={i} className="w-3 h-3 rounded-sm" style={{ backgroundColor: color }} />
                    ))}
                    <span className="text-[8px] text-[#FFD700] font-mono">高</span>
                </div>
                <div className="flex items-center gap-1 mt-1">
                    <div className="w-3 h-3 rounded-sm" style={{ backgroundColor: GRAY_COLOR }} />
                    <span className="text-[8px] text-slate-500 font-mono">焦点外</span>
                </div>
            </div>

            {/* 悬停提示 */}
            {isHovered && (
                <div className="absolute top-[50px] right-[10px] z-10 bg-black/80 backdrop-blur-md px-3 py-2 rounded-lg text-[9px] text-slate-400 font-mono">
                    <div className="flex items-center gap-2">
                        <span className="text-[#FD802E]">●</span>
                        <span>俯视热力图</span>
                    </div>
                    <div className="flex items-center gap-2 mt-1">
                        <span className="text-cyan-400">↑</span>
                        <span>X 深度 {VIEW_MIN_X}~{VIEW_MAX_X}m</span>
                    </div>
                    <div className="flex items-center gap-2 mt-1">
                        <span className="text-green-400">→</span>
                        <span>Y 宽度 焦点800mm</span>
                    </div>
                    <div className="flex items-center gap-2 mt-1">
                        <span className="text-slate-500">▦</span>
                        <span>格子 {(CELL_SIZE * 100).toFixed(0)}cm</span>
                    </div>
                </div>
            )}

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
