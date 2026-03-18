import { useState } from 'react';
import { Pickaxe, RotateCw, AlertOctagon, RotateCcw, Cpu, ChevronUp, ChevronDown, ChevronLeft, ChevronRight, LogOut } from 'lucide-react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { Slider } from './ui/slider';
import { useRobotController } from '../hooks/useRobotController';
import { MoveDirection, RobotState } from '../services/robotApi';
import { cn } from '../lib/utils';

interface RobotControlPanelProps {
    className?: string;
}
export function RobotControlPanel({ className }: RobotControlPanelProps) {
    const {
        status,
        move,
        stop,
        reset,
        setServo,
        scoop,
        dump,
        dock,
    } = useRobotController();

    const [isRemote, setIsRemote] = useState(true);
    const [servoAngles, setServoAngles] = useState({ lift: 0, dump: 0 });
    const [moveSpeed] = useState(0.5); // Fixed display value

    const handleDirectionClick = async (direction: MoveDirection) => {
        try {
            await move(direction, moveSpeed, 0.5); // Default duration 0.5s
        } catch (e) {
            console.error('Direction move failed:', e);
        }
    };

    const handleServoChange = async (servoId: string, angle: number) => {
        setServoAngles(prev => ({ ...prev, [servoId]: angle }));
        try {
            await setServo(servoId, angle);
        } catch (e) {
            console.error('Servo control failed:', e);
        }
    };

    const getStateLabel = (state: RobotState) => {
        const labels: Record<RobotState, string> = {
            [RobotState.IDLE]: '空闲',
            [RobotState.MOVING]: '移动中',
            [RobotState.SCOOPING]: '铲取中',
            [RobotState.DUMPING]: '倾倒中',
            [RobotState.ERROR]: '错误',
            [RobotState.EMERGENCY_STOP]: '紧急停止',
        };
        return labels[state] || state;
    };


    return (
        <Card className={cn(
            "relative flex flex-col h-full bg-[#1c1c1e] border-[#FD802E]/20 shadow-[0_0_25px_rgba(253,128,46,0.1)] rounded-[10px] overflow-hidden",
            className
        )}>
            {/* 右侧连接状态与开关 - 定位与上箭头平齐 */}
            <div className="absolute right-[50px] top-[calc(50%-75px)] -translate-y-1/2 z-[200] flex flex-col items-end gap-3">
                {/* REMOTE 拨档开关 (左右带图标) */}
            </div>

            {/* Top Bar Area - Independent Absolute Elements for Pixel-Perfect Centering */}
            {/* 1. Left: Set Speed */}
            <div className="absolute top-[16px] left-[32px] z-[100] flex flex-col items-start">
                <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/40 uppercase">System Ready // Speed</span>
                <span className="text-[12px] font-black text-[#FD802E] tracking-tighter">设定速度: {moveSpeed.toFixed(1)} m/s</span>
            </div>

            {/* 2. Center: Status Capsule - PHYSICALLY CENTERED */}
            <div className="absolute top-[12px] left-1/2 -translate-x-1/2 z-[110] flex items-center gap-3 bg-[#1c1c1e]/95 backdrop-blur-xl px-8 py-2.5 rounded-full border border-[#FD802E]/40 shadow-[0_0_25px_rgba(253,128,46,0.5)]">
                <div className={cn('w-2 h-2 rounded-full animate-pulse shadow-[0_0_12px_rgba(253,128,46,1)]',
                    status?.state === RobotState.IDLE ? 'bg-green-500' : 'bg-[#FD802E]')} />
                <Cpu className="w-4 h-4 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-black tracking-[0.2em] uppercase font-mono whitespace-nowrap">
                    机器人控制 | {getStateLabel(status?.state || RobotState.IDLE)}
                </span>
            </div>

            {/* 3. Right: Telemetry & Mode */}
            <div className="absolute top-[12px] right-[32px] z-[100] flex items-center gap-4">
                {/* Mode 开关 */}
                <div className="flex flex-col items-end gap-1">
                    <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/40 uppercase">Mode</span>
                    <div className="flex items-center gap-2 bg-[#1c1c1e] p-1 rounded-sm border border-[#FD802E]/40 shadow-[0_0_10px_rgba(253,128,46,0.1)]">
                        <div
                            className={cn("text-[9px] font-bold uppercase tracking-wider px-2 cursor-pointer transition-colors duration-200",
                                !isRemote ? "text-[#FD802E] drop-shadow-[0_0_8px_rgba(253,128,46,0.8)]" : "text-muted-foreground hover:text-white")}
                            onClick={() => setIsRemote(false)}
                        >
                            Local
                        </div>
                        <label className="relative inline-flex items-center cursor-pointer">
                            <input
                                type="checkbox"
                                className="sr-only peer"
                                checked={isRemote}
                                onChange={(e) => setIsRemote(e.target.checked)}
                            />
                            <div className="w-8 h-4 bg-[#2a2a2e] rounded-full peer peer-checked:after:translate-x-full after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-gradient-to-b after:from-[#ffa060] after:to-[#FD802E] after:rounded-full after:h-3 after:w-3 after:transition-all shadow-inner"></div>
                        </label>
                        <div
                            className={cn("text-[9px] font-black uppercase tracking-wider px-2 cursor-pointer transition-colors duration-200",
                                isRemote ? "text-[#FD802E] drop-shadow-[0_0_8px_rgba(253,128,46,0.8)]" : "text-muted-foreground hover:text-white")}
                            onClick={() => setIsRemote(true)}
                        >
                            Remote
                        </div>
                    </div>
                </div>

                {status && (
                    <div className="flex flex-col items-end opacity-40">
                        <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/40 uppercase">Energy</span>
                        <span className="text-[12px] font-black text-[#FD802E] leading-none">{status.battery_level.toFixed(0)}%</span>
                    </div>
                )}
            </div>

            {/* Main Interactive Layer - Absolute Positioning for Precision */}
            <div className="flex-1 relative mt-[60px] mb-[100px]">

                {/* Left Controls: Vertical Bucket Bars (50px from left, 20px gap) */}
                <div className="absolute left-[50px] top-1/2 -translate-y-1/2 flex gap-[20px] items-center">
                    {/* Lift Bar */}
                    <div className="flex flex-col items-center gap-3">
                        <span className="text-[10px] font-black text-[#FD802E] uppercase tracking-widest bg-[#FD802E]/10 px-2 py-0.5 rounded-sm">举升</span>
                        <div className="relative h-[220px] w-7 bg-[#FD802E]/5 rounded-xl flex flex-col items-center py-3 border border-[#FD802E]/10 shadow-[inset_0_0_20px_rgba(0,0,0,0.3)]">
                            <ChevronUp className="w-4 h-4 text-[#FD802E] mb-2 opacity-40" />
                            <div className="flex-1 w-2 bg-[#FD802E]/10 rounded-full relative overflow-hidden">
                                <div
                                    className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-[#FD802E] to-[#ffa060] transition-all duration-300 shadow-[0_0_20px_rgba(253,128,46,0.6)]"
                                    style={{ height: `${(servoAngles.lift / 180) * 100}%` }}
                                />
                                <Slider
                                    orientation="vertical"
                                    value={[servoAngles.lift]}
                                    onValueChange={([v]: [number]) => handleServoChange('lift', v)}
                                    min={0}
                                    max={180}
                                    step={1}
                                    className="absolute inset-0 z-10 opacity-0 cursor-ns-resize disabled:cursor-not-allowed"
                                    disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                                />
                            </div>
                            <ChevronDown className="w-4 h-4 text-[#FD802E] mt-2 opacity-40" />
                        </div>
                        <span className="text-[11px] font-black text-[#FD802E] tabular-nums">{servoAngles.lift}°</span>
                    </div>

                    {/* Dump Bar */}
                    <div className="flex flex-col items-center gap-3">
                        <span className="text-[10px] font-black text-[#FD802E] uppercase tracking-widest bg-[#FD802E]/10 px-2 py-0.5 rounded-sm">旋转</span>
                        <div className="relative h-[220px] w-7 bg-[#FD802E]/5 rounded-xl flex flex-col items-center py-3 border border-[#FD802E]/10 shadow-[inset_0_0_20px_rgba(0,0,0,0.3)]">
                            <ChevronUp className="w-4 h-4 text-[#FD802E] mb-2 opacity-40" />
                            <div className="flex-1 w-2 bg-[#FD802E]/10 rounded-full relative overflow-hidden">
                                <div
                                    className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-[#FD802E] to-[#ffa060] transition-all duration-300 shadow-[0_0_20px_rgba(253,128,46,0.6)]"
                                    style={{ height: `${(servoAngles.dump / 180) * 100}%` }}
                                />
                                <Slider
                                    orientation="vertical"
                                    value={[servoAngles.dump]}
                                    onValueChange={([v]: [number]) => handleServoChange('dump', v)}
                                    min={0}
                                    max={180}
                                    step={1}
                                    className="absolute inset-0 z-10 opacity-0 cursor-ns-resize disabled:cursor-not-allowed"
                                    disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                                />
                            </div>
                            <ChevronDown className="w-4 h-4 text-[#FD802E] mt-2 opacity-40" />
                        </div>
                        <span className="text-[11px] font-black text-[#FD802E] tabular-nums">{servoAngles.dump}°</span>
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
                                disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <ChevronUp className="w-10 h-10" />
                            </Button>
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.LEFT)}
                                disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <ChevronLeft className="w-10 h-10" />
                            </Button>
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/20 text-[#FD802E] border-2 border-[#FD802E]/60 rounded-2xl hover:bg-[#FD802E]/40 transition-all shadow-[0_0_15px_rgba(253,128,46,0.3)]"
                                onClick={() => handleDirectionClick(MoveDirection.STOP)}
                                disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <div className="w-6 h-6 bg-current rounded-sm shadow-[0_0_10px_currentColor]" />
                            </Button>
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.RIGHT)}
                                disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <ChevronRight className="w-10 h-10" />
                            </Button>
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-[68px] h-[68px] bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.BACKWARD)}
                                disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
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
                    disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <Pickaxe className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">铲取</span>
                </Button>

                {/* 2. Dump */}
                <Button
                    onClick={dump}
                    disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <RotateCw className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">倾倒</span>
                </Button>

                {/* 3. Center: EMERGENCY STOP (Anchors the group) */}
                <Button
                    onClick={stop}
                    disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[160px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/20 text-[#FD802E] border border-[#FD802E]/40 rounded-2xl hover:bg-red-500 hover:text-white transition-all font-black shadow-2xl shadow-[#FD802E]/20 ring-1 ring-[#FD802E]/20 disabled:opacity-30 disabled:grayscale"
                >
                    <AlertOctagon className="w-6 h-6" />
                    <span className="text-[14px] tracking-[0.3em]">停止</span>
                </Button>

                {/* 4. Dock */}
                <Button
                    onClick={dock}
                    disabled={!isRemote || status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <LogOut className="w-5 h-5 -rotate-90" />
                    <span className="text-[10px] tracking-[0.2em]">回桩</span>
                </Button>

                {/* 5. Reset */}
                <Button
                    onClick={reset}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E] text-black border border-[#FD802E] rounded-2xl hover:bg-[#FD802E]/90 hover:-translate-y-1 transition-all font-black shadow-[0_0_20px_rgba(253,128,46,0.4)] disabled:opacity-30 disabled:grayscale"
                    disabled={!isRemote}
                >
                    <RotateCcw className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">重置</span>
                </Button>
            </div>
        </Card>
    );
}
