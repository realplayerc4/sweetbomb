import { useState } from 'react';
import { Pickaxe, RotateCw, AlertOctagon, Cpu, ChevronUp, ChevronDown, ChevronLeft, ChevronRight, LogOut } from 'lucide-react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { useRobotController } from '../hooks/useRobotController';
import { MoveDirection } from '../services/robotApi';
import { cn } from '../lib/utils';

interface RobotControlPanelProps {
    className?: string;
}
export function RobotControlPanel({ className }: RobotControlPanelProps) {
    const {
        status,
        move,
        stop,
        scoop,
        dump,
        dock,
    } = useRobotController();

    const [isRemote] = useState(true);
    const [moveSpeed] = useState(0.5); // Fixed display value

    const handleDirectionClick = async (direction: MoveDirection) => {
        try {
            await move(direction, moveSpeed, 0.5); // Default duration 0.5s
        } catch (e) {
            console.error('Direction move failed:', e);
        }
    };

    const getStateLabel = (statusStr: string) => {
        const labels: Record<string, string> = {
            'idle': '空闲',
            'moving': '移动中',
            'scooping': '铲取中',
            'dumping': '倾倒中',
            'error': '错误',
            'emergency_stop': '紧急停止',
        };
        return labels[statusStr] || statusStr;
    };


    return (
        <Card className={cn(
            "relative flex flex-col h-full bg-[#1c1c1e] border-[#FD802E]/20 shadow-[0_0_25px_rgba(253,128,46,0.1)] rounded-[10px] overflow-hidden",
            className
        )}>
            {/* Top Bar Area - Independent Absolute Elements for Pixel-Perfect Centering */}
            {/* 1. Left: Set Speed */}
            <div className="absolute top-[16px] left-[32px] z-[100] flex flex-col items-start">
                <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/40 uppercase">System Ready // Speed</span>
                <span className="text-[12px] font-black text-[#FD802E] tracking-tighter">设定速度: {moveSpeed.toFixed(1)} m/s</span>
            </div>

            {/* 2. Center: Status Capsule - PHYSICALLY CENTERED */}
            <div className="absolute top-[12px] left-1/2 -translate-x-1/2 z-[110] flex items-center gap-3 bg-[#1c1c1e]/95 backdrop-blur-xl px-8 py-2.5 rounded-full border border-[#FD802E]/40 shadow-[0_0_25px_rgba(253,128,46,0.5)]">
                <div className={cn('w-2 h-2 rounded-full animate-pulse shadow-[0_0_12px_rgba(253,128,46,1)]',
                    status?.status === 'idle' ? 'bg-green-500' : 'bg-[#FD802E]')} />
                <Cpu className="w-4 h-4 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-black tracking-[0.2em] uppercase font-mono whitespace-nowrap">
                    机器人状态 | {getStateLabel(status?.status || 'idle')}
                </span>
            </div>

            {/* 3. Right: Energy */}
            <div className="absolute top-[12px] right-[32px] z-[100] flex items-center gap-4">
                {status?.charge !== undefined && (
                    <div className="flex flex-col items-end">
                        <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/60 uppercase">Energy</span>
                        <span className="text-[16px] font-black text-[#FD802E] leading-none">{status.charge.toFixed(0)}%</span>
                    </div>
                )}
            </div>

            {/* Main Interactive Layer */}
            <div className="flex-1 relative mt-[60px] mb-[100px]">

                {/* Left Controls: Vertical Bucket Bars (50px from left, 20px gap) */}
                <div className="absolute left-[50px] top-1/2 -translate-y-1/2 flex gap-[20px] items-center">
                    {/* BOOM Bar - Display Only */}
                    <div className="flex flex-col items-center gap-2">
                        <span className="text-[10px] font-black text-[#FD802E] uppercase tracking-widest bg-[#FD802E]/10 px-2 py-0.5 rounded-sm">举升</span>
                        <div className="relative h-[200px] w-[25px] bg-[#2a2a2e] rounded border border-[#FD802E]/30">
                            {/* 填充层 */}
                            <div
                                style={{
                                    position: 'absolute',
                                    bottom: 0,
                                    left: 0,
                                    right: 0,
                                    height: `${status && typeof status.boom === 'number' ? Math.max(5, Math.min(100, ((status.boom - 10) / 280) * 100)) : 5}%`,
                                    backgroundColor: '#FD802E',
                                    borderRadius: '0 0 4px 4px',
                                }}
                            />
                            {/* 百分比文字 */}
                            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                                <span className="text-[10px] font-bold text-white/70">
                                    {status && typeof status.boom === 'number' ? ((status.boom - 10) / 280 * 100).toFixed(0) : '--'}%
                                </span>
                            </div>
                        </div>
                        <span className="text-[12px] font-black text-[#FD802E] tabular-nums">
                            {status && typeof status.boom === 'number' ? status.boom.toFixed(1) + ' mm' : '-- mm'}
                        </span>
                    </div>

                    {/* BUCKET Bar - Display Only */}
                    <div className="flex flex-col items-center gap-2">
                        <span className="text-[10px] font-black text-[#FD802E] uppercase tracking-widest bg-[#FD802E]/10 px-2 py-0.5 rounded-sm">旋转</span>
                        <div className="relative h-[200px] w-[25px] bg-[#2a2a2e] rounded border border-[#FD802E]/30">
                            {/* 填充层 */}
                            <div
                                style={{
                                    position: 'absolute',
                                    bottom: 0,
                                    left: 0,
                                    right: 0,
                                    height: `${status && typeof status.bucket === 'number' ? Math.max(5, Math.min(100, ((status.bucket + 50) / 130) * 100)) : 5}%`,
                                    backgroundColor: '#FD802E',
                                    borderRadius: '0 0 4px 4px',
                                }}
                            />
                            {/* 百分比文字 */}
                            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                                <span className="text-[10px] font-bold text-white/70">
                                    {status && typeof status.bucket === 'number' ? ((status.bucket + 50) / 130 * 100).toFixed(0) : '--'}%
                                </span>
                            </div>
                        </div>
                        <span className="text-[12px] font-black text-[#FD802E] tabular-nums">
                            {status && typeof status.bucket === 'number' ? status.bucket.toFixed(1) + ' mm' : '-- mm'}
                        </span>
                    </div>
                </div>

                {/* Center Controls: Direction Arrows (Center Middle) */}
                <div className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2">
                    <div className="p-8 rounded-[2rem] bg-[#FD802E]/5 border border-[#FD802E]/10 shadow-[inset_0_0_30px_rgba(253,128,46,0.1)]">
                        <div className="grid grid-cols-3 gap-4">
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.FORWARD)}
                                disabled={!isRemote || status?.status === 'emergency_stop'}
                            >
                                <ChevronUp className="w-10 h-10" />
                            </Button>
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.LEFT)}
                                disabled={!isRemote || status?.status === 'emergency_stop'}
                            >
                                <ChevronLeft className="w-10 h-10" />
                            </Button>
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/20 text-[#FD802E] border-2 border-[#FD802E]/60 rounded-2xl hover:bg-[#FD802E]/40 transition-all shadow-[0_0_15px_rgba(253,128,46,0.3)]"
                                onClick={() => handleDirectionClick(MoveDirection.STOP)}
                                disabled={!isRemote || status?.status === 'emergency_stop'}
                            >
                                <div className="w-6 h-6 bg-current rounded-sm shadow-[0_0_10px_currentColor]" />
                            </Button>
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.RIGHT)}
                                disabled={!isRemote || status?.status === 'emergency_stop'}
                            >
                                <ChevronRight className="w-10 h-10" />
                            </Button>
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.BACKWARD)}
                                disabled={!isRemote || status?.status === 'emergency_stop'}
                            >
                                <ChevronDown className="w-10 h-10" />
                            </Button>
                            <div />
                        </div>
                    </div>
                </div>
            </div>

            {/* Bottom Action Footer - Symmetrical v7 Layout */}
            <div className="absolute bottom-[20px] left-0 right-0 flex justify-center items-center gap-[5px] z-[120]">
                {/* 1. Scoop */}
                <Button
                    onClick={scoop}
                    disabled={!isRemote || status?.status === 'emergency_stop'}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <Pickaxe className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">铲取</span>
                </Button>

                {/* 2. Dump */}
                <Button
                    onClick={dump}
                    disabled={!isRemote || status?.status === 'emergency_stop'}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <RotateCw className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">倾倒</span>
                </Button>

                {/* 3. Stop */}
                <Button
                    onClick={stop}
                    disabled={!isRemote || status?.status === 'emergency_stop'}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-red-500 hover:text-white transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <AlertOctagon className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">停止</span>
                </Button>

                {/* 4. Dock */}
                <Button
                    onClick={dock}
                    disabled={!isRemote || status?.status === 'emergency_stop'}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <LogOut className="w-5 h-5 -rotate-90" />
                    <span className="text-[10px] tracking-[0.2em]">回桩</span>
                </Button>
            </div>
        </Card>
    );
}