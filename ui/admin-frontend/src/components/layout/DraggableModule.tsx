import { useState } from 'react';
import Draggable from 'react-draggable';
import { ResizableBox } from 'react-resizable';
import 'react-resizable/css/styles.css';
import { cn } from '@/lib/utils';
import { Minimize, Maximize2, X } from 'lucide-react';

interface DraggableModuleProps {
  id: string;
  title: string;
  icon?: React.ReactNode;
  width: number;
  height: number;
  position: { x: number; y: number };
  onPositionChange: (id: string, position: { x: number; y: number }) => void;
  onSizeChange: (id: string, size: { width: number; height: number }) => void;
  children: React.ReactNode;
}

export function DraggableModule({ id, title, icon, width, height, position, onPositionChange, onSizeChange, children }: DraggableModuleProps) {
  const [isMinimized, setIsMinimized] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);
  const [maximizeData, setMaximizeData] = useState<{ width: number; height: number; position: { x: number; y: number } } | null>(null);

  const GRID_SIZE = 10;
  const SNAP_THRESHOLD = 15;

  const handleDrag = (e: any, data: any) => {
    // 网格对齐
    const snappedX = Math.round(data.x / GRID_SIZE) * GRID_SIZE;
    const snappedY = Math.round(data.y / GRID_SIZE) * GRID_SIZE;
    
    // 边界控制，允许向上拖拽，防止模块拖出页面右侧和底部
    const maxX = window.innerWidth - width - 32;
    const maxY = window.innerHeight - height - 100;
    const boundedX = Math.max(0, Math.min(snappedX, maxX));
    const boundedY = Math.min(snappedY, maxY);
    
    onPositionChange(id, { x: boundedX, y: boundedY });
  };

  const handleResize = (e: any, data: any) => {
    onSizeChange(id, { width: data.size.width, height: data.size.height });
  };

  const handleMinimize = () => {
    setIsMinimized(!isMinimized);
  };

  const handleMaximize = () => {
    if (isMaximized && maximizeData) {
      onSizeChange(id, { width: maximizeData.width, height: maximizeData.height });
      onPositionChange(id, maximizeData.position);
    } else {
      setMaximizeData({ width, height, position });
      onSizeChange(id, { width: window.innerWidth - 32, height: window.innerHeight - 100 });
      onPositionChange(id, { x: 16, y: 80 });
    }
    setIsMaximized(!isMaximized);
  };

  return (
    <Draggable
      position={position}
      onDrag={handleDrag}
      cancel="button, input, select, textarea"
      bounds="parent"
    >
      <ResizableBox
        width={width}
        height={isMinimized ? 48 : height}
        onResize={handleResize}
        minConstraints={[300, isMinimized ? 48 : 200]}
        maxConstraints={[window.innerWidth - 32, window.innerHeight - 100]}
        resizeHandles={isMinimized ? [] : ['se']}
        className={cn('draggable-module shadow-lg z-10 border border-blue-500/30')}
      >
        <div className="h-full flex flex-col">
          <div className="module-header px-4 py-3 flex items-center justify-between">
            <div className="flex items-center gap-2">
              {icon && <span className="text-tech-blue">{icon}</span>}
              <h3 className="text-sm font-semibold text-tech-blue tracking-wider">{title}</h3>
            </div>
            <div className="flex items-center gap-2">
              <button
                onClick={handleMinimize}
                className="p-1.5 rounded hover:bg-white/10 text-text-muted hover:text-text-secondary transition-colors"
              >
                <Minimize className="w-3.5 h-3.5" />
              </button>
              <button
                onClick={handleMaximize}
                className="p-1.5 rounded hover:bg-white/10 text-text-muted hover:text-text-secondary transition-colors"
              >
                <Maximize2 className="w-3.5 h-3.5" />
              </button>
            </div>
          </div>
          {!isMinimized && (
            <div className="flex-1 p-4 overflow-auto">
              {children}
            </div>
          )}
        </div>
      </ResizableBox>
    </Draggable>
  );
}
