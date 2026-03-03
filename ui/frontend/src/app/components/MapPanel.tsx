import { BarChart3 } from 'lucide-react';
import { Card } from './ui/card';

export function MapPanel() {
    return (
        <Card className="relative overflow-hidden h-full p-0 bg-[#1c1c1e] border-[#2a2a2e] rounded-[10px] shadow-md group">
            {/* Top Status Capsule */}
            <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
                <BarChart3 className="w-3.5 h-3.5 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">Map</span>
                <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
                    | GRID MAP
                </span>
            </div>

            {/* Map Placeholder */}
            <div className="absolute inset-0 bg-slate-900/50 flex items-center justify-center border-2 border-dashed border-slate-700/30 rounded-xl m-4 mt-16">
                <span className="text-slate-600 font-mono tracking-widest uppercase text-xs animate-pulse">Waiting for Map Data...</span>
            </div>

            {/* Subtle overlay for industrial look - matching VideoView */}
            <div className="absolute inset-0 pointer-events-none border-[12px] border-black/5" />
        </Card>
    );
}
