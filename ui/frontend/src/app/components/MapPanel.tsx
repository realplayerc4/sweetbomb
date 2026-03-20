import { useState, useRef, useCallback, useEffect } from 'react';
import { BarChart3, ZoomIn, ZoomOut, RotateCcw, Maximize, Loader2 } from 'lucide-react';
import { Card } from './ui/card';
import { useMapImage, useMapList } from '../hooks/useMap';

interface Transform {
  scale: number;
  translateX: number;
  translateY: number;
}

const MIN_SCALE = 0.5;
const MAX_SCALE = 5;
const ZOOM_STEP = 0.2;

export function MapPanel() {
  // 使用第一个可用的地图
  const { maps } = useMapList();
  const currentMap = maps[0]?.name || 'map_gridmap';

  const { imageUrl, isLoading, error, loadImage } = useMapImage(currentMap);

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

  // 计算合适的初始缩放，使图片适应容器
  const fitToContainer = useCallback(() => {
    if (!containerRef.current || !imageRef.current) return;

    const container = containerRef.current.getBoundingClientRect();
    const img = imageRef.current;

    const scaleX = (container.width - 40) / img.naturalWidth;
    const scaleY = (container.height - 40) / img.naturalHeight;
    const scale = Math.min(scaleX, scaleY, 1);

    setTransform({
      scale,
      translateX: 0,
      translateY: 0,
    });
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
  };

  const handleFit = () => {
    if (!containerRef.current || !imageRef.current) return;

    const container = containerRef.current.getBoundingClientRect();
    const img = imageRef.current;

    const scaleX = (container.width - 40) / img.naturalWidth;
    const scaleY = (container.height - 40) / img.naturalHeight;
    const scale = Math.min(scaleX, scaleY);

    setTransform({
      scale,
      translateX: 0,
      translateY: 0,
    });
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
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        <BarChart3 className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">Map</span>
        <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
          | {currentMap.toUpperCase()}
        </span>
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
          </div>
        )}
      </div>
    </Card>
  );
}
