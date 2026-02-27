import { Settings2 } from 'lucide-react';

interface SpatialConfigProps {
    pcCamZ: number;
    pcCamX: number;
    cameraHeight: number;
    bucketHeight: number;
    tolerance: number;
    setPcCamZ: (v: number) => void;
    setPcCamX: (v: number) => void;
    setCameraHeight: (v: number) => void;
    setBucketHeight: (v: number) => void;
    setTolerance: (v: number) => void;
}

export function SpatialConfigPanel({
    pcCamZ,
    pcCamX,
    cameraHeight,
    bucketHeight,
    tolerance,
    setPcCamZ,
    setPcCamX,
    setCameraHeight,
    setBucketHeight,
    setTolerance,
}: SpatialConfigProps) {
    return (
        <div className="bg-[#1c1c1e] rounded-2xl shadow-md relative h-full overflow-hidden">
            {/* 悬浮状态胶囊 - 放置在容器顶部，不受 padding 影响 */}
            <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
                <Settings2 className="w-3.5 h-3.5 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">空间配置 </span>
                <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
                    | SPATIAL CONFIG
                </span>
            </div>

            {/* 控制内容区域 - 增加顶部内边距避开胶囊 */}
            <div className="p-6 pt-16 flex flex-col gap-5">
                {[
                    { label: "观测高度 (Z)", value: pcCamZ, unit: "m", min: 1.0, max: 10.0, step: 0.1, onChange: setPcCamZ },
                    { label: "观测位置 (X)", value: pcCamX, unit: "m", min: -10.0, max: 10.0, step: 0.1, onChange: setPcCamX },
                    { label: "相机高度基准", value: cameraHeight, unit: "m", min: 0.0, max: 2.0, step: 0.01, onChange: setCameraHeight },
                    { label: "铲斗高度基准", value: bucketHeight, unit: "m", min: 0.0, max: 1.0, step: 0.01, onChange: setBucketHeight },
                    { label: "切面厚度 (容差)", value: tolerance, unit: "±m", min: 0.05, max: 1.0, step: 0.01, onChange: setTolerance },
                ].map((cfg) => (
                    <div key={cfg.label} className="flex flex-col gap-2">
                        <div className="flex justify-between text-[11px] font-mono">
                            <span className="text-slate-500">{cfg.label}</span>
                            <span className="text-orange-500 font-bold">{cfg.value.toFixed(2)}{cfg.unit}</span>
                        </div>
                        <input
                            type="range"
                            min={cfg.min}
                            max={cfg.max}
                            step={cfg.step}
                            value={cfg.value}
                            onChange={(e) => cfg.onChange(parseFloat(e.target.value))}
                            className="w-full h-1 bg-white/5 rounded-lg appearance-none cursor-pointer accent-orange-500"
                        />
                    </div>
                ))}
            </div>
        </div>
    );
}
