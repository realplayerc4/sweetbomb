import { BarChart3 } from 'lucide-react';
import { Card } from './ui/card';

export function MapPanel() {
    return (
        <Card className="relative overflow-hidden h-full p-0 bg-[#1c1c1e] border-[#FD802E]/20 rounded-[10px] shadow-[0_0_25px_rgba(253,128,46,0.1)] group">
            {/* Top Status Capsule */}
            <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
                <BarChart3 className="w-3.5 h-3.5 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">Map</span>
                <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
                    | GRID MAP
                </span>
            </div>

            {/* Map Placeholder - 保留背景但不显示边框 */}
            <div className="absolute inset-0 bg-slate-900/30 m-4 mt-16">
                {/* Reserved for Grid Map */}
            </div>

            {/* Subtle overlay for industrial look - matching VideoView */}
            <div className="absolute inset-0 pointer-events-none border-[12px] border-black/5" />
        </Card>
    );
}
