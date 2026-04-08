import { useState, useRef, useCallback, useEffect, useMemo } from 'react';
import { BarChart3, ZoomIn, ZoomOut, RotateCcw, Maximize, Loader2 } from 'lucide-react';
import { Card } from './ui/card';
import { useMapImage, useMapList, useMapInfo } from '../hooks/useMap';
import { useRobotController } from '../hooks/useRobotController';

interface Transform {
  scale: number;
  translateX: number;
  translateY: number;
  theta: number; // 旋转角度（弧度）
}

const MIN_SCALE = 0.5;
const MAX_SCALE = 5;
const ZOOM_STEP = 0.2;
const DEFAULT_THETA_DEG = -178.0472; // 顺时针旋转178.0472度

// 车辆实际尺寸（米）
const VEHICLE_WIDTH_M = 0.8;  // 800mm
const VEHICLE_LENGTH_M = 1.4; // 1400mm

export function MapPanel() {
  // 使用第一个可用的地图
  const { maps } = useMapList();
  const currentMap = maps[0]?.name || 'map_gridmap';

  // theta输入值（角度）
  const [thetaInput, setThetaInput] = useState<string>(String(DEFAULT_THETA_DEG));

  // 传递theta参数给useMapImage
  const currentThetaDeg = parseFloat(thetaInput) || 0;
  const { imageUrl, isLoading, error, loadImage } = useMapImage(currentMap, currentThetaDeg);
  const { info: mapInfo } = useMapInfo(currentMap, currentThetaDeg);
  const { status } = useRobotController();

  // 变换状态
  const [transform, setTransform] = useState<Transform>({
    scale: 1,
    translateX: 0,
    translateY: 0,
    theta: (DEFAULT_THETA_DEG * Math.PI) / 180, // 默认旋转角度（弧度）
  });

  // 拖拽状态
  const [isDragging, setIsDragging] = useState(false);
  const dragStartRef = useRef({ x: 0, y: 0 });
  const containerRef = useRef<HTMLDivElement>(null);
  const imageRef = useRef<HTMLImageElement>(null);

  // 机器人状态（用于显示和地图指示器）
  // 机器人坐标已经是转换后的坐标系，单位mm
  const robotStatus = status?.connected ? status : null;

  // 计算车辆在图片上的像素位置和尺寸
  // 坐标单位mm -> 地图单位m
  const vehicleData = useMemo(() => {
    if (!robotStatus || !mapInfo || !imageRef.current) return null;

    const img = imageRef.current;
    const rb = mapInfo.rotated_bounds;
    const res = mapInfo.resolution;

    // 机器人坐标 mm -> m
    const xM = robotStatus.x / 1000;
    const yM = robotStatus.y / 1000;

    // 车辆中心像素位置（相对于PNG图片）
    const px = ((xM - rb.min_x) / res) * (img.naturalWidth / mapInfo.img_width_px);
    const py = ((rb.max_y - yM) / res) * (img.naturalHeight / mapInfo.img_height_px);

    // 车辆像素尺寸
    const vehicleWidthPx = (VEHICLE_WIDTH_M / res) * (img.naturalWidth / mapInfo.img_width_px);
    const vehicleLengthPx = (VEHICLE_LENGTH_M / res) * (img.naturalHeight / mapInfo.img_height_px);

    return { px, py, vehicleWidthPx, vehicleLengthPx };
  }, [robotStatus, mapInfo]);

  // 计算合适的初始缩放，使图片适应容器
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
    if (imageUrl && !isLoading) {
      // 等待图片实际加载完成
      const timer = setTimeout(() => {
        fitToContainer();
      }, 100);
      return () => clearTimeout(timer);
    }
  }, [imageUrl, isLoading, fitToContainer]);

  // 处理theta输入变化
  const handleThetaChange = useCallback((value: string) => {
    setThetaInput(value);
    const thetaDeg = parseFloat(value) || 0;
    const thetaRad = (thetaDeg * Math.PI) / 180;
    setTransform((prev) => ({
      ...prev,
      theta: thetaRad,
    }));
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
    setTransform((prev) => ({
      ...prev,
      theta: (DEFAULT_THETA_DEG * Math.PI) / 180,
    }));
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
    if (e.button !== 0) return; // 只响应左键
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
    <Card className="relative overflow-hidden h-full p-0 bg-[#1c1c1e] border-[#FD802E]/20 rounded-[10px] shadow-[0_0_25px_rgba(253,128,46,0.1)] group">
      {/* Top Status Capsule */}
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-4 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        <BarChart3 className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">{currentMap.toUpperCase()}</span>
        <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 font-mono font-bold">
          X {robotStatus ? robotStatus.x.toFixed(3) : '--'}
        </span>
        <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 font-mono font-bold">
          Y {robotStatus ? robotStatus.y.toFixed(3) : '--'}
        </span>
        <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 font-mono font-bold">
          A {robotStatus ? robotStatus.a.toFixed(1) : '--'}°
        </span>
      </div>

      {/* Theta Input */}
      <div className="absolute top-[10px] right-3 z-10 flex items-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-3 py-2 rounded-lg border border-[#FD802E]/40">
        <span className="text-[10px] text-[#FD802E]/80 font-mono">θ</span>
        <input
          type="number"
          value={thetaInput}
          onChange={(e) => handleThetaChange(e.target.value)}
          className="w-[70px] h-[20px] bg-[#1c1c1e] text-[#FD802E] text-[10px] font-mono text-center rounded border border-[#FD802E]/30 focus:border-[#FD802E] focus:outline-none"
          placeholder="-178.0472"
          step="0.0001"
        />
        <span className="text-[10px] text-[#FD802E]/80 font-mono">°</span>
      </div>

      {/* Toolbar */}
      <div className="absolute top-[60px] left-3 z-10 flex flex-col gap-2">
        <button
          onClick={handleZoomIn}
          className="w-8 h-8 flex items-center justify-center bg-[#1c1c1e]/90 backdrop-blur-md rounded-lg border border-[#FD802E]/40 text-[#FD802E] hover:bg-[#FD802E]/20 transition-all"
          title="放大"
        >
          <ZoomIn className="w-4 h-4" />
        </button>
        <button
          onClick={handleZoomOut}
          className="w-8 h-8 flex items-center justify-center bg-[#1c1c1e]/90 backdrop-blur-md rounded-lg border border-[#FD802E]/40 text-[#FD802E] hover:bg-[#FD802E]/20 transition-all"
          title="缩小"
        >
          <ZoomOut className="w-4 h-4" />
        </button>
        <button
          onClick={handleReset}
          className="w-8 h-8 flex items-center justify-center bg-[#1c1c1e]/90 backdrop-blur-md rounded-lg border border-[#FD802E]/40 text-[#FD802E] hover:bg-[#FD802E]/20 transition-all"
          title="重置"
        >
          <RotateCcw className="w-4 h-4" />
        </button>
        <button
          onClick={handleFit}
          className="w-8 h-8 flex items-center justify-center bg-[#1c1c1e]/90 backdrop-blur-md rounded-lg border border-[#FD802E]/40 text-[#FD802E] hover:bg-[#FD802E]/20 transition-all"
          title="适应窗口"
        >
          <Maximize className="w-4 h-4" />
        </button>
      </div>

      {/* Zoom Level Indicator */}
      <div className="absolute bottom-3 left-3 z-10 bg-[#1c1c1e]/90 backdrop-blur-md px-3 py-1.5 rounded-lg border border-[#FD802E]/40">
        <span className="text-[10px] text-[#FD802E] font-mono">
          {(transform.scale * 100).toFixed(0)}%
        </span>
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
          <div className="absolute inset-0 flex items-center justify-center bg-[#1c1c1e]">
            <Loader2 className="w-8 h-8 text-[#FD802E] animate-spin" />
          </div>
        )}

        {error && (
          <div className="absolute inset-0 flex flex-col items-center justify-center bg-[#1c1c1e] p-4">
            <p className="text-[#FD802E]/80 text-sm text-center mb-2">加载地图失败</p>
            <p className="text-[#FD802E]/50 text-xs text-center mb-4">{error}</p>
            <button
              onClick={() => loadImage()}
              className="px-4 py-2 bg-[#FD802E]/20 text-[#FD802E] text-xs rounded-lg border border-[#FD802E]/40 hover:bg-[#FD802E]/30 transition-all"
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
              style={{
                maxWidth: 'none',
                maxHeight: 'none',
              }}
            />

            {/* Vehicle Position Indicator */}
            {vehicleData && robotStatus && imageRef.current && (
              <svg
                style={{
                  position: 'absolute',
                  top: 0,
                  left: 0,
                  width: imageRef.current.naturalWidth,
                  height: imageRef.current.naturalHeight,
                  pointerEvents: 'none',
                  overflow: 'visible',
                }}
              >
                <g
                  transform={`translate(${vehicleData.px}, ${vehicleData.py}) rotate(${-robotStatus.a})`}
                >
                  {/* Vehicle body - 根据实际车体尺寸 */}
                  <rect
                    x={-vehicleData.vehicleWidthPx / 2}
                    y={-vehicleData.vehicleLengthPx / 2}
                    width={vehicleData.vehicleWidthPx}
                    height={vehicleData.vehicleLengthPx}
                    fill="rgba(253, 128, 46, 0.9)"
                    stroke="#FD802E"
                    strokeWidth={2}
                    rx={3}
                  />
                  {/* Arrow pointing forward (车头方向) */}
                  <polygon
                    points={`0,${-vehicleData.vehicleLengthPx / 2 - 10} ${-6},${-vehicleData.vehicleLengthPx / 2 + 2} ${6},${-vehicleData.vehicleLengthPx / 2 + 2}`}
                    fill="#FD802E"
                  />
                </g>
              </svg>
            )}
          </div>
        )}
      </div>
    </Card>
  );
}