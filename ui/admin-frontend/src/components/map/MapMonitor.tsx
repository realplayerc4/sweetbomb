import { useState, useRef, useCallback, useEffect, useMemo } from 'react';
import { ZoomIn, ZoomOut, RotateCcw, Maximize, Loader2 } from 'lucide-react';
import { useMapList, useMapImage, useMapInfo } from '@/hooks/useMap';
import { usePathMap } from '@/hooks/usePathMap';
import { useRobotStore } from '@/stores/useRobotStore';

interface Transform {
  scale: number;
  translateX: number;
  translateY: number;
}

const MIN_SCALE = 0.5;
const MAX_SCALE = 5;
const ZOOM_STEP = 0.2;
const DEFAULT_THETA_DEG = -178.0472; // 顺时针旋转178.0472度

// 车辆实际尺寸（米）
const VEHICLE_WIDTH_M = 0.8; // 800mm
const VEHICLE_LENGTH_M = 1.4; // 1400mm

export function MapMonitor() {
  // 使用第一个可用的地图
  const { maps } = useMapList();
  const currentMap = maps[0]?.name || 'map_gridmap';

  // theta输入值（角度）
  const [thetaInput, setThetaInput] = useState<string>(String(DEFAULT_THETA_DEG));

  // 传递theta参数给useMapImage
  const currentThetaDeg = parseFloat(thetaInput) || 0;
  const { imageUrl, isLoading, error, loadImage } = useMapImage(currentMap, currentThetaDeg);
  const { info: mapInfo } = useMapInfo(currentMap, currentThetaDeg);
  const { pickStations, dropStations, chargeStations, nodes, loadPathMap } = usePathMap();

  // 机器人状态
  const robotStatus = useRobotStore((s) => s.status);
  const robotPosition = useRobotStore((s) => s.position);
  const robotOrientation = useRobotStore((s) => s.orientation);
  const isConnected = useRobotStore((s) => s.isConnected);

  // 变换状态
  const [transform, setTransform] = useState<Transform>({
    scale: 1,
    translateX: 0,
    translateY: 0,
  });

  // 拖拽状态
  const [isDragging, setIsDragging] = useState(false);
  const dragStartRef = useRef({ x: 0, y: 0 });
  const containerRef = useRef<HTMLDivElement>(null);
  const imageRef = useRef<HTMLImageElement>(null);

  // 图片加载状态
  const [imageLoaded, setImageLoaded] = useState(false);

  // 像素偏移量（手动校准）
  const [pixelOffsetX, setPixelOffsetX] = useState(() => {
    const saved = localStorage.getItem('mapOffsetX');
    return saved ? parseInt(saved) : 0;
  });
  const [pixelOffsetY, setPixelOffsetY] = useState(() => {
    const saved = localStorage.getItem('mapOffsetY');
    return saved ? parseInt(saved) : 0;
  });

  // 保存偏移量到localStorage
  useEffect(() => {
    localStorage.setItem('mapOffsetX', String(pixelOffsetX));
  }, [pixelOffsetX]);

  useEffect(() => {
    localStorage.setItem('mapOffsetY', String(pixelOffsetY));
  }, [pixelOffsetY]);

  // 构建机器人状态对象（兼容 SW frontend 格式）
  const robotData = useMemo(() => {
    if (!isConnected) return null;
    return {
      x: robotPosition[0], // mm
      y: robotPosition[1], // mm
      a: robotOrientation[0], // 角度
    };
  }, [isConnected, robotPosition, robotOrientation]);

  // 计算车辆在图片上的像素位置和尺寸
  const vehicleData = useMemo(() => {
    if (!robotData || !mapInfo || !imageLoaded || !imageRef.current) return null;

    const img = imageRef.current;
    const offset = mapInfo.offset;
    const dataArea = mapInfo.data_area_px;
    const ar = mapInfo.axes_range || mapInfo.png_bounds;

    // 机器人坐标 mm -> m
    const xM = robotData.x / 1000;
    const yM = robotData.y / 1000;

    // 归一化位置
    const normX = (xM - ar.min_x) / (ar.max_x - ar.min_x);
    const normY = (ar.max_y - yM) / (ar.max_y - ar.min_y);

    // PNG像素位置
    const px = offset.x_left + normX * dataArea.width + pixelOffsetX;
    const py = offset.y_top + normY * dataArea.height + pixelOffsetY;

    // 车辆像素尺寸
    const pixelPerMeterX = dataArea.width / (ar.max_x - ar.min_x);
    const pixelPerMeterY = dataArea.height / (ar.max_y - ar.min_y);
    const vehicleWidthPx = VEHICLE_WIDTH_M * pixelPerMeterX;
    const vehicleLengthPx = VEHICLE_LENGTH_M * pixelPerMeterY;

    // 车辆朝向角度
    const rotatedAngle = 90 - robotData.a;

    return {
      px,
      py,
      vehicleWidthPx,
      vehicleLengthPx,
      imgW: img.naturalWidth,
      imgH: img.naturalHeight,
      rotatedAngle,
    };
  }, [robotData, mapInfo, imageLoaded, pixelOffsetX, pixelOffsetY]);

  // 计算站点标记的像素位置
  const stationMarkers = useMemo(() => {
    if (!mapInfo || !imageLoaded || !imageRef.current)
      return { markers: [], sugarPile: null };
    if (pickStations.length === 0 && dropStations.length === 0 && chargeStations.length === 0)
      return { markers: [], sugarPile: null };

    const offset = mapInfo.offset;
    const dataArea = mapInfo.data_area_px;
    const ar = mapInfo.axes_range || mapInfo.png_bounds;

    // 坐标转换函数：mm -> 像素
    const mmToPx = (xMm: number, yMm: number) => {
      const xM = xMm / 1000;
      const yM = yMm / 1000;
      const normX = (xM - ar.min_x) / (ar.max_x - ar.min_x);
      const normY = (ar.max_y - yM) / (ar.max_y - ar.min_y);
      const px = offset.x_left + normX * dataArea.width + pixelOffsetX;
      const py = offset.y_top + normY * dataArea.height + pixelOffsetY;
      return { px, py };
    };

    const pixelPerMeter = dataArea.width / (ar.max_x - ar.min_x);

    const markers: Array<{ px: number; py: number; label: string; type: 'pick' | 'drop' | 'charge' }> =
      [];

    // 取货站
    pickStations.forEach((ps) => {
      const connectNode = nodes.find((n) => n.id === ps.connect_node);
      if (connectNode) {
        const { px, py } = mmToPx(connectNode.x, connectNode.y);
        markers.push({ px, py, label: `${ps.connect_node}`, type: 'pick' });
      }
      ps.positions.forEach((pos) => {
        const { px, py } = mmToPx(pos.x, pos.y);
        markers.push({ px, py, label: `${pos.station_id}`, type: 'pick' });
      });
    });

    // 放货站
    dropStations.forEach((ds) => {
      if (ds.x !== null && ds.y !== null) {
        const { px, py } = mmToPx(ds.x, ds.y);
        markers.push({ px, py, label: `${ds.connect_node}`, type: 'drop' });
      }
    });

    // 充电站
    chargeStations.forEach((cs) => {
      const node = nodes.find((n) => n.id === cs.node);
      if (node) {
        const { px, py } = mmToPx(node.x, node.y);
        markers.push({ px, py, label: `${cs.node}`, type: 'charge' });
      }
    });

    // 糖堆中心点 + R 半径圆
    let sugarPile: { cx: number; cy: number; radiusPx: number } | null = null;
    if (pickStations.length > 0) {
      const ps = pickStations[0];
      const { px, py } = mmToPx(ps.ox, ps.oy);
      const radiusPx = (ps.R / 1000) * pixelPerMeter;
      sugarPile = { cx: px, cy: py, radiusPx };
    }

    return { markers, sugarPile };
  }, [mapInfo, imageLoaded, pickStations, dropStations, chargeStations, nodes, pixelOffsetX, pixelOffsetY]);

  // 图片加载完成回调
  const handleImageLoad = useCallback(() => {
    setImageLoaded(true);
  }, []);

  // 自动加载 pathMap 站点数据
  useEffect(() => {
    loadPathMap();
  }, [loadPathMap]);

  // 重置图片加载状态
  useEffect(() => {
    setImageLoaded(false);
  }, [imageUrl]);

  // 计算合适的初始缩放
  const fitToContainer = useCallback(() => {
    if (!containerRef.current || !imageRef.current) return;

    const container = containerRef.current.getBoundingClientRect();
    const img = imageRef.current;

    const scaleX = (container.width - 40) / img.naturalWidth;
    const scaleY = (container.height - 40) / img.naturalHeight;
    const scale = Math.min(scaleX, scaleY, 1);

    setTransform((prev) => ({
      ...prev,
      scale,
      translateX: 0,
      translateY: 0,
    }));
  }, []);

  // 图片加载完成后自动适应容器
  useEffect(() => {
    if (imageUrl && !isLoading && imageLoaded) {
      const timer = setTimeout(() => {
        fitToContainer();
      }, 100);
      return () => clearTimeout(timer);
    }
  }, [imageUrl, isLoading, imageLoaded, fitToContainer]);

  // 处理theta输入变化
  const handleThetaChange = useCallback((value: string) => {
    setThetaInput(value);
  }, []);

  // 缩放控制
  const handleZoomIn = () => {
    setTransform((prev) => ({
      ...prev,
      scale: Math.min(prev.scale + ZOOM_STEP, MAX_SCALE),
    }));
  };

  const handleZoomOut = () => {
    setTransform((prev) => ({
      ...prev,
      scale: Math.max(prev.scale - ZOOM_STEP, MIN_SCALE),
    }));
  };

  const handleReset = () => {
    fitToContainer();
    setThetaInput(String(DEFAULT_THETA_DEG));
  };

  const handleFit = () => {
    if (!containerRef.current || !imageRef.current) return;

    const container = containerRef.current.getBoundingClientRect();
    const img = imageRef.current;

    const scaleX = (container.width - 40) / img.naturalWidth;
    const scaleY = (container.height - 40) / img.naturalHeight;
    const scale = Math.min(scaleX, scaleY);

    setTransform((prev) => ({
      ...prev,
      scale,
      translateX: 0,
      translateY: 0,
    }));
  };

  // 鼠标滚轮缩放
  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? -ZOOM_STEP : ZOOM_STEP;
    setTransform((prev) => ({
      ...prev,
      scale: Math.max(MIN_SCALE, Math.min(MAX_SCALE, prev.scale + delta)),
    }));
  };

  // 拖拽功能
  const handleMouseDown = (e: React.MouseEvent) => {
    if (e.button !== 0) return;
    setIsDragging(true);
    dragStartRef.current = {
      x: e.clientX - transform.translateX,
      y: e.clientY - transform.translateY,
    };
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!isDragging) return;
    setTransform((prev) => ({
      ...prev,
      translateX: e.clientX - dragStartRef.current.x,
      translateY: e.clientY - dragStartRef.current.y,
    }));
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleMouseLeave = () => {
    setIsDragging(false);
  };

  return (
    <div className="relative w-full h-full bg-[#0f172a] rounded-lg overflow-hidden border border-cyan-500/20">
      {/* Status Header */}
      <div className="absolute top-2 left-1/2 -translate-x-1/2 z-10 flex items-center gap-2 bg-[#0f172a]/90 backdrop-blur-md px-3 py-1.5 rounded-full border border-cyan-500/40">
        <div
          className={`w-2 h-2 rounded-full ${isConnected ? 'bg-cyan-400 animate-pulse' : 'bg-gray-500'}`}
        />
        <span className="text-[10px] text-cyan-400 font-bold tracking-widest uppercase font-mono">
          {currentMap.toUpperCase()}
        </span>
        {robotData && (
          <>
            <span className="text-[10px] text-cyan-400/80 border-l border-cyan-500/30 pl-2 font-mono">
              X {robotData.x.toFixed(0)}
            </span>
            <span className="text-[10px] text-cyan-400/80 border-l border-cyan-500/30 pl-2 font-mono">
              Y {robotData.y.toFixed(0)}
            </span>
            <span className="text-[10px] text-cyan-400/80 border-l border-cyan-500/30 pl-2 font-mono">
              A {robotData.a.toFixed(1)}°
            </span>
          </>
        )}
      </div>

      {/* Legend */}
      <div className="absolute top-2 right-2 z-10 flex items-center gap-3 bg-[#0f172a]/90 backdrop-blur-md px-3 py-1.5 rounded-lg border border-cyan-500/40">
        <div className="flex items-center gap-1.5">
          <div className="w-4 h-3 border-2 border-cyan-400 bg-cyan-400/30 rounded-sm" />
          <span className="text-[10px] text-slate-300 font-mono">机器人</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-3 h-3 rounded-full bg-purple-500" />
          <span className="text-[10px] text-slate-300 font-mono">站点</span>
        </div>
      </div>

      {/* Toolbar */}
      <div className="absolute top-1/2 right-2 -translate-y-1/2 z-10 flex flex-col gap-1">
        <button
          onClick={handleZoomIn}
          className="w-7 h-7 flex items-center justify-center bg-[#0f172a]/90 backdrop-blur-md rounded border border-cyan-500/40 text-cyan-400 hover:bg-cyan-500/20 transition-all"
          title="放大"
        >
          <ZoomIn className="w-3.5 h-3.5" />
        </button>
        <button
          onClick={handleZoomOut}
          className="w-7 h-7 flex items-center justify-center bg-[#0f172a]/90 backdrop-blur-md rounded border border-cyan-500/40 text-cyan-400 hover:bg-cyan-500/20 transition-all"
          title="缩小"
        >
          <ZoomOut className="w-3.5 h-3.5" />
        </button>
        <button
          onClick={handleReset}
          className="w-7 h-7 flex items-center justify-center bg-[#0f172a]/90 backdrop-blur-md rounded border border-cyan-500/40 text-cyan-400 hover:bg-cyan-500/20 transition-all"
          title="重置"
        >
          <RotateCcw className="w-3.5 h-3.5" />
        </button>
        <button
          onClick={handleFit}
          className="w-7 h-7 flex items-center justify-center bg-[#0f172a]/90 backdrop-blur-md rounded border border-cyan-500/40 text-cyan-400 hover:bg-cyan-500/20 transition-all"
          title="适应窗口"
        >
          <Maximize className="w-3.5 h-3.5" />
        </button>
      </div>

      {/* Zoom Level + Offset Controls */}
      <div className="absolute bottom-2 left-2 z-10 flex flex-col gap-1">
        <div className="bg-[#0f172a]/90 backdrop-blur-md px-2 py-1 rounded border border-cyan-500/40">
          <span className="text-[10px] text-cyan-400 font-mono">
            {(transform.scale * 100).toFixed(0)}%
          </span>
        </div>
        <div className="flex items-center gap-1 bg-[#0f172a]/90 backdrop-blur-md px-2 py-1 rounded border border-cyan-500/40">
          <span className="text-[9px] text-slate-400 font-mono">偏移</span>
          <div className="flex items-center gap-0.5">
            <span className="text-[9px] text-cyan-400/80 font-mono">X</span>
            <input
              type="number"
              value={pixelOffsetX}
              onChange={(e) => setPixelOffsetX(parseInt(e.target.value) || 0)}
              className="w-[40px] h-[18px] bg-[#0f172a] text-cyan-400 text-[9px] font-mono text-center rounded border border-cyan-500/30 focus:border-cyan-400 focus:outline-none"
              step="1"
            />
          </div>
          <div className="flex items-center gap-0.5">
            <span className="text-[9px] text-cyan-400/80 font-mono">Y</span>
            <input
              type="number"
              value={pixelOffsetY}
              onChange={(e) => setPixelOffsetY(parseInt(e.target.value) || 0)}
              className="w-[40px] h-[18px] bg-[#0f172a] text-cyan-400 text-[9px] font-mono text-center rounded border border-cyan-500/30 focus:border-cyan-400 focus:outline-none"
              step="1"
            />
          </div>
        </div>
      </div>

      {/* Map Container */}
      <div
        ref={containerRef}
        className="w-full h-full cursor-grab active:cursor-grabbing overflow-hidden"
        onWheel={handleWheel}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseLeave}
      >
        {isLoading && (
          <div className="absolute inset-0 flex items-center justify-center bg-[#0f172a]">
            <Loader2 className="w-8 h-8 text-cyan-400 animate-spin" />
          </div>
        )}

        {error && (
          <div className="absolute inset-0 flex flex-col items-center justify-center bg-[#0f172a] p-4">
            <p className="text-cyan-400/80 text-sm text-center mb-2">加载地图失败</p>
            <p className="text-cyan-400/50 text-xs text-center mb-4">{error}</p>
            <button
              onClick={() => loadImage()}
              className="px-4 py-2 bg-cyan-500/20 text-cyan-400 text-xs rounded-lg border border-cyan-500/40 hover:bg-cyan-500/30 transition-all"
            >
              重试
            </button>
          </div>
        )}

        {imageUrl && !isLoading && (
          <div
            className="absolute top-1/2 left-1/2 will-change-transform"
            style={{
              transform: `translate(-50%, -50%) translate(${transform.translateX}px, ${transform.translateY}px) scale(${transform.scale})`,
            }}
          >
            <img
              ref={imageRef}
              src={imageUrl}
              alt="Grid Map"
              className="select-none pointer-events-none"
              draggable={false}
              onLoad={handleImageLoad}
              style={{
                maxWidth: 'none',
                maxHeight: 'none',
              }}
            />

            {/* Vehicle Position Indicator */}
            {vehicleData && robotData && (
              <svg
                style={{
                  position: 'absolute',
                  top: 0,
                  left: 0,
                  width: vehicleData.imgW,
                  height: vehicleData.imgH,
                  pointerEvents: 'none',
                  overflow: 'visible',
                }}
                viewBox={`0 0 ${vehicleData.imgW} ${vehicleData.imgH}`}
              >
                <g
                  transform={`translate(${vehicleData.px}, ${vehicleData.py}) rotate(${vehicleData.rotatedAngle})`}
                >
                  {/* Vehicle body */}
                  <rect
                    x={-vehicleData.vehicleWidthPx / 2}
                    y={-vehicleData.vehicleLengthPx / 2}
                    width={vehicleData.vehicleWidthPx}
                    height={vehicleData.vehicleLengthPx}
                    fill="rgba(0, 212, 255, 0.9)"
                    stroke="#00d4ff"
                    strokeWidth={2}
                    rx={3}
                  />
                  {/* Arrow pointing forward */}
                  <polygon
                    points={`0,${-vehicleData.vehicleLengthPx / 2 - 10} ${-6},${-vehicleData.vehicleLengthPx / 2 + 2} ${6},${-vehicleData.vehicleLengthPx / 2 + 2}`}
                    fill="#00d4ff"
                  />
                </g>
              </svg>
            )}

            {/* Station Markers */}
            {stationMarkers.markers.length > 0 && vehicleData && (
              <svg
                style={{
                  position: 'absolute',
                  top: 0,
                  left: 0,
                  width: vehicleData.imgW,
                  height: vehicleData.imgH,
                  pointerEvents: 'none',
                  overflow: 'visible',
                }}
                viewBox={`0 0 ${vehicleData.imgW} ${vehicleData.imgH}`}
              >
                {/* 糖堆中心点 + R 半径圆 */}
                {stationMarkers.sugarPile && (
                  <g>
                    <circle
                      cx={stationMarkers.sugarPile.cx}
                      cy={stationMarkers.sugarPile.cy}
                      r={stationMarkers.sugarPile.radiusPx}
                      fill="rgba(255, 215, 0, 0.1)"
                      stroke="#FFD700"
                      strokeWidth={1.5}
                      strokeDasharray="6 3"
                    />
                    <circle
                      cx={stationMarkers.sugarPile.cx}
                      cy={stationMarkers.sugarPile.cy}
                      r={4}
                      fill="#FFD700"
                    />
                  </g>
                )}
                {stationMarkers.markers.map((marker, idx) => (
                  <g key={`${marker.label}-${idx}`}>
                    <circle
                      cx={marker.px}
                      cy={marker.py}
                      r={5}
                      fill="#9333EA"
                      stroke="#c084fc"
                      strokeWidth={1}
                    />
                    <rect
                      x={marker.px + 4}
                      y={marker.py + 2}
                      width={20}
                      height={12}
                      rx={2}
                      fill="rgba(0,0,0,0.7)"
                    />
                    <text
                      x={marker.px + 14}
                      y={marker.py + 11}
                      fill="white"
                      fontSize="8"
                      fontFamily="monospace"
                      fontWeight="bold"
                      textAnchor="middle"
                    >
                      {marker.label}
                    </text>
                  </g>
                ))}
              </svg>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
