import { useEffect, useRef, useState } from 'react';
import { Locate } from 'lucide-react';

interface SliceViewProps {
    isActive: boolean;
    pointCloudData: Float32Array | null;
    systemStats?: {
        cpu_load?: number;
        battery?: number;
        temperature?: number;
        signal?: number;
        hostname?: string;
    } | null;
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

import { API_BASE } from '../config';

interface SliceSettings {
    teethHeight: number;   // Z1: 铲齿放平时的高度 (米)
    cameraToTeeth: number;  // 相机到铲齿前沿距离 (米)
    bucketDepth: number;    // 铲斗深度 (米)
    bucketVolume: number;   // 铲斗目标体积 (升)
    lr: number;             // 取料半径 (米) - 防止超挖
}


interface AdvanceInfo {
    nearestX: number;        // 最近物料的 X 坐标
    nearestY: number;        // 最近物料的 Y 坐标
    materialDistance: number; // 铲齿到物料的距离
    advanceDistance: number;  // 需要前进的总距离
}

// ----- 固定参数 -----
const CELL_SIZE = 0.02;          // 格子大小 0.02m
const DEFAULT_VIEW_DEPTH = 2.0;  // 默认观测深度 2.0m
const VIEW_MIN_Y = -1.5;
const VIEW_MAX_Y = 1.5;
const BUCKET_HEIGHT = 0.3;
const FOCUS_HALF_WIDTH = 0.3;

const DEFAULT_SETTINGS: SliceSettings = {
    teethHeight: -0.85,
    cameraToTeeth: 1000,
    bucketDepth: 0.3,
    bucketVolume: 30,
    lr: 3.0,
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

const HEATMAP_COLORS_RGB: [number, number, number][] = [
    [204, 85, 0],
    [230, 115, 0],
    [253, 128, 46],
    [255, 169, 64],
    [255, 215, 0],
];
const GRAY_RGB: [number, number, number] = [58, 58, 60];

function getColorRGBForHeight(height: number, minH: number, maxH: number, inFocus: boolean): [number, number, number] {
    if (!inFocus) return GRAY_RGB;
    const range = maxH - minH;
    if (range <= 0) return HEATMAP_COLORS_RGB[2];
    const normalized = (height - minH) / range;
    const clamped = Math.max(0, Math.min(1, normalized));
    const idx = Math.min(4, Math.floor(clamped * 5));
    return HEATMAP_COLORS_RGB[idx];
}

function generateMockPointCloud(): Float32Array {
    const points: number[] = [];
    for (let x = 1.0; x <= 3.0; x += 0.05) {
        for (let y = -1.0; y <= 1.0; y += 0.05) {
            const distFromCenter = Math.sqrt((x - 2.0) ** 2 + y ** 2);
            const baseHeight = -0.05 + 0.2 * Math.exp(-distFromCenter * 1.5);
            const noise = (Math.random() - 0.5) * 0.04;
            const z = baseHeight + noise;
            points.push(x, y, z);
        }
    }
    return new Float32Array(points);
}

let _mockData: Float32Array | null = null;
function getMockPointCloud(): Float32Array {
    if (!_mockData) _mockData = generateMockPointCloud();
    return _mockData;
}

export function SliceView({
    isActive,
    pointCloudData,
    systemStats: _systemStats,
    pointCloudAnalysis
}: SliceViewProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const [settings, setSettings] = useState<SliceSettings>(loadSettings);
    const [pointCount, setPointCount] = useState(0);
    const [advanceInfo, setAdvanceInfo] = useState<AdvanceInfo | null>(null);

    const z1 = settings.teethHeight;
    const z2 = z1 + BUCKET_HEIGHT;
    const viewMinX = settings.cameraToTeeth;
    const viewMaxX = settings.cameraToTeeth + DEFAULT_VIEW_DEPTH;

    const activeData = (pointCloudData && pointCloudData.length > 0) ? pointCloudData : getMockPointCloud();
    const dataAvailable = activeData.length > 0;

    useEffect(() => {
        const timer = setTimeout(() => saveSettings(settings), 500);
        return () => clearTimeout(timer);
    }, [settings]);

    // 同步设置到后端
    useEffect(() => {
        const timer = setTimeout(async () => {
            try {
                const res = await fetch(`${API_BASE}/devices/038122250462/point_cloud/settings`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(settings),
                });
                if (res.ok) {
                    console.log('[SliceView] Settings synced to backend:', settings);
                }
            } catch (e) {
                console.warn('[SliceView] Failed to sync settings:', e);
            }
        }, 600);
        return () => clearTimeout(timer);
    }, [settings]);

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
        const minY = VIEW_MIN_Y;
        const maxY = VIEW_MAX_Y;

        let filteredCount = 0;

        for (let i = 0; i < activeData.length; i += 3) {
            const x = activeData[i];
            const y = activeData[i + 1];
            const z = activeData[i + 2];
            if (x >= fixedMinX && x <= fixedMaxX && y >= minY && y <= maxY && z >= minHeight && z <= maxHeight) {
                filteredCount++;
            }
        }

        setPointCount(filteredCount);

        // 使用后端分析结果，前端不再计算
        if (pointCloudAnalysis?.distances?.nearest_material !== undefined && pointCloudAnalysis.distances.nearest_material !== null) {
            const materialDistance = pointCloudAnalysis.distances.nearest_material;
            const advanceDistance = materialDistance + settings.bucketDepth;
            const nearestX = pointCloudAnalysis.distances.nearest_x ?? (materialDistance + settings.cameraToTeeth);
            const nearestY = pointCloudAnalysis.distances.nearest_y ?? 0;
            setAdvanceInfo({ nearestX, nearestY, materialDistance, advanceDistance });
        } else {
            setAdvanceInfo(null);
        }

        if (filteredCount === 0) return;

        const xRange = fixedMaxX - fixedMinX;
        const yRange = VIEW_MAX_Y - VIEW_MIN_Y;
        const gridCols = Math.max(1, Math.ceil(yRange / CELL_SIZE));
        const gridRows = Math.ceil(xRange / CELL_SIZE);
        const heightGrid = new Float32Array(gridRows * gridCols).fill(-Infinity);

        for (let i = 0; i < activeData.length; i += 3) {
            const x = activeData[i];
            const y = activeData[i + 1];
            const z = activeData[i + 2];
            if (x >= fixedMinX && x <= fixedMaxX && y >= minY && y <= maxY && z >= minHeight && z <= maxHeight) {
                const col = Math.min(Math.floor((VIEW_MAX_Y - y) / CELL_SIZE), gridCols - 1);
                const row = Math.min(Math.floor((x - fixedMinX) / CELL_SIZE), gridRows - 1);
                const idx = row * gridCols + col;
                if (z > heightGrid[idx]) heightGrid[idx] = z;
            }
        }

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
        const yCenter = (minY + maxY) / 2;
        const focusMinY = yCenter - FOCUS_HALF_WIDTH;
        const focusMaxY = yCenter + FOCUS_HALF_WIDTH;

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

        if (cellSizePixels >= 4) {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
            ctx.lineWidth = 0.5;
            for (let row = 0; row <= gridRows; row++) {
                const cy = offsetY + (gridRows - row) * cellSizePixels;
                ctx.beginPath(); ctx.moveTo(offsetX, cy); ctx.lineTo(offsetX + gridCols * cellSizePixels, cy); ctx.stroke();
            }
            for (let col = 0; col <= gridCols; col++) {
                const cx = offsetX + col * cellSizePixels;
                ctx.beginPath(); ctx.moveTo(cx, offsetY); ctx.lineTo(cx, offsetY + gridRows * cellSizePixels); ctx.stroke();
            }
        }

        ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
        ctx.font = '9px monospace';
        ctx.textAlign = 'right';
        const xRangeMeters = fixedMaxX - fixedMinX;
        const xStep = xRangeMeters <= 1.0 ? 0.2 : (xRangeMeters <= 2.0 ? 0.5 : 1.0);
        for (let xVal = fixedMinX; xVal <= fixedMaxX + 0.001; xVal += xStep) {
            const row = Math.round((xVal - fixedMinX) / CELL_SIZE);
            const cy = offsetY + (gridRows - row) * cellSizePixels;
            ctx.fillText(`${xVal.toFixed(1)}m`, offsetX - 4, cy + 3);
        }

        ctx.textAlign = 'center';
        for (let yVal = VIEW_MIN_Y; yVal <= VIEW_MAX_Y + 0.001; yVal += 0.5) {
            const col = Math.round((VIEW_MAX_Y - yVal) / CELL_SIZE);
            const cx = offsetX + col * cellSizePixels;
            ctx.fillText(`${yVal.toFixed(1)}`, cx, offsetY + gridRows * cellSizePixels + 14);
        }

        const focusLeftPx = offsetX + Math.max(0, (maxY - focusMaxY)) * scale;
        const focusRightPx = offsetX + Math.min(yRange, (maxY - focusMinY)) * scale;
        ctx.strokeStyle = 'rgba(253, 128, 46, 0.3)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(focusLeftPx, offsetY); ctx.lineTo(focusLeftPx, offsetY + gridRows * cellSizePixels); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(focusRightPx, offsetY); ctx.lineTo(focusRightPx, offsetY + gridRows * cellSizePixels); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = 'rgba(253, 128, 46, 0.5)';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('600mm焦点区域', (focusLeftPx + focusRightPx) / 2, offsetY - 4);
    }, [activeData, z1, z2, viewMinX, viewMaxX, settings.cameraToTeeth, settings.bucketDepth, dataAvailable]);

    const handleResetView = () => { setSettings(DEFAULT_SETTINGS); };

    return (
        <div className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group border border-[#FD802E]/20 shadow-[0_0_25px_rgba(253,128,46,0.1)]">
            <style>{`
                input[type='number']::-webkit-inner-spin-button,
                input[type='number']::-webkit-outer-spin-button {
                    -webkit-appearance: none;
                    margin: 0;
                }
                input[type='number'] {
                    -moz-appearance: textfield;
                }
            `}</style>

            {/* 顶部状态胶囊 */}
            <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
                {isActive ? (
                    <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
                ) : (
                    <div className="w-3 h-3 bg-slate-500 rounded-full" />
                )}
                <Locate className="w-3.5 h-3.5 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> BEV SLICE </span>
                <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
                    | {pointCount.toLocaleString()} PTS
                </span>
            </div>

            <canvas ref={canvasRef} width={600} height={330} className="w-full h-full" />

            <div className="absolute bottom-[10px] right-[10px] z-[100] bg-black/80 backdrop-blur-md p-2 rounded-lg border border-[#FD802E]/30 max-w-[153px]">
                <div className="text-[10px] text-[#FD802E] font-mono mb-2 uppercase tracking-tighter font-bold">parametre</div>

                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[10px] text-[#FD802E] font-mono w-14">相机→齿</span>
                    <input
                        type="number"
                        step="0.001"
                        value={settings.cameraToTeeth.toFixed(3)}
                        onChange={(e) => setSettings(prev => ({ ...prev, cameraToTeeth: parseFloat(e.target.value) || 0 }))}
                        className="w-[48px] h-4 bg-transparent border border-[#FD802E]/20 rounded text-[10px] text-[#FD802E] text-center font-mono focus:outline-none focus:border-[#FD802E]"
                    />
                    <span className="text-[10px] text-[#FD802E] font-mono">m</span>
                </div>

                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[10px] text-[#FD802E] font-mono w-14">铲斗深度</span>
                    <input
                        type="number"
                        step="0.001"
                        value={settings.bucketDepth.toFixed(3)}
                        onChange={(e) => setSettings(prev => ({ ...prev, bucketDepth: parseFloat(e.target.value) || 0 }))}
                        className="w-[48px] h-4 bg-transparent border border-[#FD802E]/20 rounded text-[10px] text-[#FD802E] text-center font-mono focus:outline-none focus:border-[#FD802E]"
                    />
                    <span className="text-[10px] text-[#FD802E] font-mono">m</span>
                </div>

                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[10px] text-[#FD802E] font-mono w-14">目标体积</span>
                    <input
                        type="number"
                        step="0.001"
                        value={settings.bucketVolume.toFixed(3)}
                        onChange={(e) => setSettings(prev => ({ ...prev, bucketVolume: parseFloat(e.target.value) || 30 }))}
                        className="w-[48px] h-4 bg-transparent border border-[#FD802E]/20 rounded text-[10px] text-[#FD802E] text-center font-mono focus:outline-none focus:border-[#FD802E]"
                    />
                    <span className="text-[10px] text-[#FD802E] font-mono">L</span>
                </div>

                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[10px] text-[#FD802E] font-mono w-14">相机高度</span>
                    <input
                        type="number"
                        step="0.001"
                        value={settings.teethHeight.toFixed(3)}
                        onChange={(e) => setSettings(prev => ({ ...prev, teethHeight: parseFloat(e.target.value) || 0 }))}
                        className="w-[48px] h-4 bg-transparent border border-[#FD802E]/20 rounded text-[10px] text-[#FD802E] text-center font-mono focus:outline-none focus:border-[#FD802E]"
                    />
                    <span className="text-[10px] text-[#FD802E] font-mono">m</span>
                </div>

                <div className="flex items-center gap-2 mb-2">
                    <span className="text-[10px] text-[#FD802E] font-mono w-14">堆体高度</span>
                    <span className="text-[10px] text-[#FD802E] font-mono w-[48px] text-right font-bold pr-1">
                        {pointCloudAnalysis?.pile_height !== undefined ? pointCloudAnalysis.pile_height.toFixed(3) : '--'}
                    </span>
                    <span className="text-[10px] text-[#FD802E] font-mono">m</span>
                </div>

                {/* 作业建议 */}
                {advanceInfo && (
                    <div className="border-t border-[#FD802E]/30 pt-2 mb-2">
                        <div className="text-[10px] text-[#FD802E] font-mono mb-1.5 font-bold">作业建议</div>
                        <div className="flex items-center justify-between mb-1">
                            <span className="text-[10px] text-[#FD802E] font-mono font-bold">铲齿→物料</span>
                            <span className="text-[10px] text-[#FD802E] font-mono font-bold">{advanceInfo.materialDistance.toFixed(3)}m</span>
                        </div>
                        <div className="flex items-center justify-between">
                            <span className="text-[10px] text-[#FD802E] font-mono font-bold">建议前进</span>
                            <span className="text-[10px] text-[#FD802E] font-mono font-bold">{advanceInfo.advanceDistance.toFixed(3)}m</span>
                        </div>
                    </div>
                )}

                <button
                    onClick={handleResetView}
                    className="w-full px-2 py-1 mt-1 bg-[#FD802E]/50 hover:bg-[#FD802E]/70 rounded text-[10px] text-[#FD802E] font-mono font-bold transition-colors"
                >
                    重置
                </button>
            </div>

            {!isActive && (
                <div className="absolute inset-0 flex items-center justify-center bg-black/30 pointer-events-none z-20">
                    <div className="text-center bg-black/60 p-4 rounded-xl backdrop-blur-md border border-slate-700 pointer-events-auto">
                        <div className="w-12 h-12 rounded-full bg-slate-700/50 flex items-center justify-center mx-auto mb-2" />
                        <p className="text-slate-300 text-sm font-bold">无设备连接</p>
                        <p className="text-[#FD802E] text-xs mt-1">目前展示 Mock 模拟数据测试效果</p>
                    </div>
                </div>
            )}
        </div>
    );
}
