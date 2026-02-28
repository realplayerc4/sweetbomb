import { Cloud } from 'lucide-react';

interface PointCloudProps {
  isActive: boolean;
  points: Float32Array | null;
  metrics?: { pointCount: number } | null;
}

export function PointCloud({ isActive, points, metrics }: PointCloudProps) {
  // 计算点云边界框
  const bounds = points ? {
    minX: Infinity,
    maxX: -Infinity,
    minY: Infinity,
    maxY: -Infinity,
    minZ: Infinity,
    maxZ: -Infinity,
  } : null;

  if (points && bounds) {
    for (let i = 0; i < points.length; i += 3) {
      const x = points[i];
      const y = points[i + 1];
      const z = points[i + 2];

      bounds.minX = Math.min(bounds.minX, x);
      bounds.maxX = Math.max(bounds.maxX, x);
      bounds.minY = Math.min(bounds.minY, y);
      bounds.maxY = Math.max(bounds.maxY, y);
      bounds.minZ = Math.min(bounds.minZ, z);
      bounds.maxZ = Math.max(bounds.maxZ, z);
    }
  }

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md">
      {/* 悬浮状态胶囊 */}
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        {isActive ? (
          <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
        ) : (
          <div className="w-3 h-3 bg-slate-500 rounded-full" />
        )}
        <Cloud className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> Point Cloud </span>
        {metrics && isActive && (
          <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
            | {metrics.pointCount.toLocaleString()} POINTS
          </span>
        )}
      </div>

      {/* 点云数据显示区域 */}
      <div className="w-full h-full flex items-center justify-center p-8">
        {!isActive ? (
          <div className="text-center">
            <div className="w-16 h-16 rounded-full bg-slate-700/50 flex items-center justify-center mx-auto mb-3">
              <Cloud className="w-8 h-8 text-slate-500" />
            </div>
            <p className="text-slate-500 text-sm">等待流启动...</p>
            <p className="text-slate-600 text-xs mt-1">ROS 坐标系</p>
          </div>
        ) : bounds ? (
          <div className="w-full space-y-3">
            {/* 点云统计 */}
            <div className="bg-black/20 rounded-lg p-4 border border-white/5">
              <div className="grid grid-cols-3 gap-4 text-xs font-mono">
                <div>
                  <p className="text-slate-500 mb-1">顶点数</p>
                  <p className="text-white font-bold text-sm">{metrics?.pointCount.toLocaleString() || 0}</p>
                </div>
                <div>
                  <p className="text-slate-500 mb-1">数据大小</p>
                  <p className="text-white font-bold text-sm">{(points!.length * 4 / 1024).toFixed(1)} KB</p>
                </div>
                <div>
                  <p className="text-slate-500 mb-1">坐标系</p>
                  <p className="text-[#FD802E] font-bold text-sm">ROS (X,Y,Z)</p>
                </div>
              </div>
            </div>

            {/* 坐标范围 */}
            <div className="grid grid-cols-3 gap-3">
              <div className="bg-black/20 rounded-lg p-3 border border-white/5">
                <p className="text-slate-500 text-[10px] mb-2 font-mono">X 轴范围 (前)</p>
                <div className="text-xs font-mono space-y-1">
                  <p><span className="text-slate-500">MIN:</span> <span className="text-white">{bounds.minX.toFixed(3)} m</span></p>
                  <p><span className="text-slate-500">MAX:</span> <span className="text-white">{bounds.maxX.toFixed(3)} m</span></p>
                </div>
              </div>
              <div className="bg-black/20 rounded-lg p-3 border border-white/5">
                <p className="text-slate-500 text-[10px] mb-2 font-mono">Y 轴范围 (左)</p>
                <div className="text-xs font-mono space-y-1">
                  <p><span className="text-slate-500">MIN:</span> <span className="text-white">{bounds.minY.toFixed(3)} m</span></p>
                  <p><span className="text-slate-500">MAX:</span> <span className="text-white">{bounds.maxY.toFixed(3)} m</span></p>
                </div>
              </div>
              <div className="bg-black/20 rounded-lg p-3 border border-white/5">
                <p className="text-slate-500 text-[10px] mb-2 font-mono">Z 轴范围 (上)</p>
                <div className="text-xs font-mono space-y-1">
                  <p><span className="text-slate-500">MIN:</span> <span className="text-white">{bounds.minZ.toFixed(3)} m</span></p>
                  <p><span className="text-slate-500">MAX:</span> <span className="text-white">{bounds.maxZ.toFixed(3)} m</span></p>
                </div>
              </div>
            </div>
          </div>
        ) : null}
      </div>
    </div>
  );
}
