import { useEffect, useRef, useState } from 'react';
import { Layers } from 'lucide-react';

interface SliceViewProps {
  isActive: boolean;
  pointCloudData: Float32Array | null;
}

interface SliceSettings {
  minHeight: number;  // Minimum Z height in meters
  maxHeight: number;  // Maximum Z height in meters
  sliceResolution: number;  // Grid resolution (cells per meter)
}

const DEFAULT_SETTINGS: SliceSettings = {
  minHeight: -0.5,   // Ground level and below
  maxHeight: 1.0,    // Up to 1 meter high
  sliceResolution: 20,  // 20 cells per meter = 5cm per cell
};

const STORAGE_KEY = 'slice-view-settings';

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

// Color mapping for height: blue (low) -> green -> yellow -> red (high)
function getColorForHeight(height: number, minH: number, maxH: number): string {
  const normalized = (height - minH) / (maxH - minH);
  const clamped = Math.max(0, Math.min(1, normalized));

  // Blue to Green (0.0 - 0.5)
  if (clamped < 0.5) {
    const t = clamped * 2;
    const r = 0;
    const g = Math.round(255 * t);
    const b = Math.round(255 * (1 - t));
    return `rgb(${r}, ${g}, ${b})`;
  }
  // Green to Red (0.5 - 1.0)
  else {
    const t = (clamped - 0.5) * 2;
    const r = Math.round(255 * t);
    const g = Math.round(255 * (1 - t));
    const b = 0;
    return `rgb(${r}, ${g}, ${b})`;
  }
}

export function SliceView({ isActive, pointCloudData }: SliceViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [settings, setSettings] = useState<SliceSettings>(loadSettings);
  const [isHovered, setIsHovered] = useState(false);
  const [pointCount, setPointCount] = useState(0);

  // Cache bounds to prevent jitter - only update when significantly different
  const boundsRef = useRef<{ minX: number; maxX: number; minY: number; maxY: number } | null>(null);

  // Save settings when changed
  useEffect(() => {
    const timer = setTimeout(() => saveSettings(settings), 500);
    return () => clearTimeout(timer);
  }, [settings]);

  // Draw BEV slice
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !pointCloudData || pointCloudData.length === 0) {
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

    // Clear canvas
    ctx.fillStyle = '#1c1c1e';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Filter points by Z height (slice) and accumulate max heights per cell
    const { minHeight, maxHeight, sliceResolution } = settings;

    // Determine XY bounds from point cloud
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let filteredCount = 0;

    for (let i = 0; i < pointCloudData.length; i += 3) {
      const x = pointCloudData[i];
      const y = pointCloudData[i + 1];
      const z = pointCloudData[i + 2];

      if (z >= minHeight && z <= maxHeight) {
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
        filteredCount++;
      }
    }

    setPointCount(filteredCount);

    if (filteredCount === 0 || !isFinite(minX)) return;

    // Use cached bounds if available and similar (prevents jitter)
    const bounds = boundsRef.current;
    const boundsChanged = !bounds ||
      Math.abs(bounds.minX - minX) > 0.5 ||
      Math.abs(bounds.maxX - maxX) > 0.5 ||
      Math.abs(bounds.minY - minY) > 0.5 ||
      Math.abs(bounds.maxY - maxY) > 0.5;

    if (!boundsChanged && bounds) {
      // Use cached bounds
      minX = bounds.minX;
      maxX = bounds.maxX;
      minY = bounds.minY;
      maxY = bounds.maxY;
    } else {
      // Update cached bounds
      boundsRef.current = { minX, maxX, minY, maxY };
    }

    // Create grid for height map
    const cellSize = 1.0 / sliceResolution;
    const gridCols = Math.ceil((maxX - minX) / cellSize);
    const gridRows = Math.ceil((maxY - minY) / cellSize);
    const heightGrid: Float32Array[] = [];

    // Second pass: accumulate max heights per cell
    for (let i = 0; i < pointCloudData.length; i += 3) {
      const x = pointCloudData[i];
      const y = pointCloudData[i + 1];
      const z = pointCloudData[i + 2];

      if (z >= minHeight && z <= maxHeight) {
        const col = Math.floor((x - minX) / cellSize);
        const row = Math.floor((y - minY) / cellSize);
        const idx = row * gridCols + col;

        if (!heightGrid[idx]) {
          heightGrid[idx] = new Float32Array([z]);
        } else {
          // Keep maximum height for this cell
          if (z > heightGrid[idx][0]) {
            heightGrid[idx][0] = z;
          }
        }
      }
    }

    // Scale to fit canvas while maintaining aspect ratio
    const padding = 40;
    const availableWidth = canvas.width - 2 * padding;
    const availableHeight = canvas.height - 2 * padding;
    const dataAspectRatio = (maxX - minX) / (maxY - minY);
    const canvasAspectRatio = availableWidth / availableHeight;

    let scale, offsetX, offsetY;
    if (dataAspectRatio > canvasAspectRatio) {
      scale = availableWidth / (maxX - minX);
      offsetX = padding;
      offsetY = padding + (availableHeight - (maxY - minY) * scale) / 2;
    } else {
      scale = availableHeight / (maxY - minY);
      offsetX = padding + (availableWidth - (maxX - minX) * scale) / 2;
      offsetY = padding;
    }

    // Draw height grid cells
    for (let row = 0; row < gridRows; row++) {
      for (let col = 0; col < gridCols; col++) {
        const idx = row * gridCols + col;
        const cellData = heightGrid[idx];

        if (cellData && cellData[0] !== undefined) {
          const z = cellData[0];
          const x = minX + col * cellSize + cellSize / 2;
          const y = minY + row * cellSize + cellSize / 2;

          // Map to canvas coordinates (Y is inverted in canvas)
          const canvasX = offsetX + (x - minX) * scale;
          const canvasY = canvas.height - (offsetY + (y - minY) * scale);
          const cellSizePixels = cellSize * scale;

          ctx.fillStyle = getColorForHeight(z, minHeight, maxHeight);
          ctx.fillRect(
            canvasX - cellSizePixels / 2,
            canvasY - cellSizePixels / 2,
            cellSizePixels + 1,  // +1 to fill gaps
            cellSizePixels + 1
          );
        }
      }
    }

    // Draw origin marker
    const originX = offsetX + (0 - minX) * scale;
    const originY = canvas.height - (offsetY + (0 - minY) * scale);

    ctx.strokeStyle = '#FD802E';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(originX, originY, 8, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#FD802E';
    ctx.font = 'bold 10px monospace';
    ctx.fillText('ORIGIN', originX + 12, originY + 4);

    // Draw compass
    const compassX = canvas.width - 40;
    const compassY = 40;
    ctx.strokeStyle = 'rgba(253, 128, 46, 0.5)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(compassX, compassY, 25, 0, Math.PI * 2);
    ctx.stroke();

    // North arrow
    ctx.fillStyle = '#FD802E';
    ctx.beginPath();
    ctx.moveTo(compassX, compassY - 18);
    ctx.lineTo(compassX - 6, compassY - 6);
    ctx.lineTo(compassX + 6, compassY - 6);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = 'rgba(253, 128, 46, 0.7)';
    ctx.font = 'bold 9px monospace';
    ctx.fillText('N', compassX - 3, compassY - 22);

  }, [pointCloudData, settings]);

  const handleSettingChange = (key: keyof SliceSettings, value: number) => {
    setSettings(prev => ({ ...prev, [key]: value }));
  };

  const handleResetView = () => {
    setSettings(DEFAULT_SETTINGS);
  };

  return (
    <div
      className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md"
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      {/* Top Status Capsule */}
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        {isActive ? (
          <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
        ) : (
          <div className="w-3 h-3 bg-slate-500 rounded-full" />
        )}
        <Layers className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> BEV SLICE </span>
        <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
          | {pointCount.toLocaleString()} POINTS
        </span>
      </div>

      {/* Canvas */}
      <canvas
        ref={canvasRef}
        width={600}
        height={330}
        className="w-full h-full"
      />

      {/* Height Range Controls */}
      <div className="absolute bottom-[10px] left-[10px] z-10 bg-black/80 backdrop-blur-md p-3 rounded-lg border border-slate-700">
        <div className="text-[9px] text-slate-400 font-mono mb-2">Z 轴切片范围 (米)</div>

        <div className="space-y-2 mb-3">
          <div className="flex items-center gap-2">
            <span className="text-[9px] text-blue-400 font-mono w-8">MIN</span>
            <input
              type="number"
              step="0.1"
              value={settings.minHeight.toFixed(2)}
              onChange={(e) => handleSettingChange('minHeight', parseFloat(e.target.value) || 0)}
              className="w-16 h-5 bg-slate-800 border border-slate-600 rounded text-[9px] text-white px-1 font-mono"
            />
          </div>
          <div className="flex items-center gap-2">
            <span className="text-[9px] text-red-400 font-mono w-8">MAX</span>
            <input
              type="number"
              step="0.1"
              value={settings.maxHeight.toFixed(2)}
              onChange={(e) => handleSettingChange('maxHeight', parseFloat(e.target.value) || 0)}
              className="w-16 h-5 bg-slate-800 border border-slate-600 rounded text-[9px] text-white px-1 font-mono"
            />
          </div>
        </div>

        {/* Resolution Control */}
        <div className="border-t border-slate-600 pt-2 mb-2">
          <div className="flex items-center justify-between mb-1">
            <span className="text-[9px] text-slate-400 font-mono">分辨率</span>
            <span className="text-[9px] text-[#FD802E] font-mono">{settings.sliceResolution} /m</span>
          </div>
          <input
            type="range"
            min="5"
            max="50"
            step="5"
            value={settings.sliceResolution}
            onChange={(e) => handleSettingChange('sliceResolution', parseInt(e.target.value))}
            className="w-full h-1.5 bg-slate-700 rounded-lg appearance-none cursor-pointer accent-[#FD802E]"
          />
        </div>

        {/* Reset Button */}
        <button
          onClick={handleResetView}
          className="w-full px-2 py-1 bg-slate-700 hover:bg-slate-600 rounded text-[9px] text-slate-300 font-mono transition-colors"
        >
          重置视图
        </button>
      </div>

      {/* Height Legend */}
      <div className="absolute bottom-[10px] right-[10px] z-10 bg-black/80 backdrop-blur-md p-2 rounded-lg border border-slate-700">
        <div className="text-[9px] text-slate-400 font-mono mb-1">高度</div>
        <div className="flex items-center gap-1">
          <span className="text-[8px] text-blue-400 font-mono">低</span>
          <div className="w-16 h-2 rounded" style={{
            background: 'linear-gradient(to right, rgb(0,0,255), rgb(0,255,0), rgb(255,255,0), rgb(255,0,0))'
          }} />
          <span className="text-[8px] text-red-400 font-mono">高</span>
        </div>
      </div>

      {/* Instructions */}
      {isHovered && (
        <div className="absolute top-[50px] right-[10px] z-10 bg-black/80 backdrop-blur-md px-3 py-2 rounded-lg text-[9px] text-slate-400 font-mono">
          <div className="flex items-center gap-2">
            <span className="text-[#FD802E]">●</span>
            <span>俯视热力图</span>
          </div>
          <div className="flex items-center gap-2 mt-1">
            <span className="text-blue-400">蓝</span>
            <span>→</span>
            <span className="text-red-400">红</span>
            <span>: 低 → 高</span>
          </div>
        </div>
      )}

      {!isActive && (
        <div className="absolute inset-0 flex items-center justify-center bg-black/50">
          <div className="text-center">
            <div className="w-16 h-16 rounded-full bg-slate-700/50 flex items-center justify-center mx-auto mb-3">
              <Layers className="w-8 h-8 text-slate-500" />
            </div>
            <p className="text-slate-500 text-sm">等待流启动...</p>
            <p className="text-slate-600 text-xs mt-1">BEV 切片视图</p>
          </div>
        </div>
      )}
    </div>
  );
}
