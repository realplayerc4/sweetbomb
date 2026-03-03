import { useState } from 'react';
import { Pickaxe, RotateCw, AlertOctagon, RotateCcw, Cpu, ChevronUp, ChevronDown, LogOut } from 'lucide-react';
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
        error,
        clearError,
    } = useRobotController();

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

    if (error) {
        return (
            <Card className={cn('relative p-6 bg-[#1A1A1E] border-[#2a2a2e] shadow-2xl rounded-[10px]', className)}>
                <div className="text-red-500 text-center py-10">
                    <p className="font-medium text-lg text-[#FD802E]">系统错误</p>
                    <p className="text-sm mt-1 mb-4 opacity-80">{error}</p>
                    <Button onClick={clearError} variant="outline" className="border-[#FD802E]/50 text-[#FD802E] hover:bg-[#FD802E]/10">
                        清除错误
                    </Button>
                </div>
            </Card>
        );
    }

    return (
        <Card className={cn('relative bg-[#1A1A1E] border-[#2a2a2e] shadow-2xl rounded-[10px] flex flex-col h-full overflow-hidden min-h-[400px]', className)}>

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

            {/* 3. Right: Telemetry */}
            {status && (
                <div className="absolute top-[16px] right-[32px] z-[100] flex flex-col items-end opacity-40">
                    <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/40 uppercase">Energy Level</span>
                    <span className="text-[12px] font-black text-[#FD802E] leading-none">{status.battery_level.toFixed(0)}%</span>
                </div>
            )}

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
                                    className="absolute inset-0 z-10 opacity-0 cursor-ns-resize"
                                    disabled={status?.state === RobotState.EMERGENCY_STOP}
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
                                    className="absolute inset-0 z-10 opacity-0 cursor-ns-resize"
                                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                                />
                            </div>
                            <ChevronDown className="w-4 h-4 text-[#FD802E] mt-2 opacity-40" />
                        </div>
                        <span className="text-[11px] font-black text-[#FD802E] tabular-nums">{servoAngles.dump}°</span>
                    </div>
                </div>

                {/* Right Controls: Direction Arrows (Right Middle) */}
                <div className="absolute right-[50px] top-1/2 -translate-y-1/2">
                    <div className="p-6 rounded-[2rem] bg-[#FD802E]/5 border border-[#FD802E]/10 shadow-[inset_0_0_30px_rgba(253,128,46,0.1)]">
                        <div className="grid grid-cols-3 gap-4">
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-14 h-14 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.FORWARD)}
                                disabled={status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <ChevronUp className="w-8 h-8" />
                            </Button>
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-14 h-14 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.LEFT)}
                                disabled={status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <span className="text-3xl font-light pr-1">←</span>
                            </Button>
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-14 h-14 bg-[#FD802E]/20 text-[#FD802E] border-2 border-[#FD802E]/60 rounded-2xl hover:bg-[#FD802E]/40 transition-all shadow-[0_0_15px_rgba(253,128,46,0.3)]"
                                onClick={() => handleDirectionClick(MoveDirection.STOP)}
                                disabled={status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <div className="w-5 h-5 bg-current rounded-sm shadow-[0_0_10px_currentColor]" />
                            </Button>
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-14 h-14 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.RIGHT)}
                                disabled={status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <span className="text-3xl font-light pl-1">→</span>
                            </Button>
                            <div />
                            <Button
                                variant="ghost"
                                size="icon"
                                className="w-14 h-14 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/30 hover:scale-110 transition-all font-black"
                                onClick={() => handleDirectionClick(MoveDirection.BACKWARD)}
                                disabled={status?.state === RobotState.EMERGENCY_STOP}
                            >
                                <ChevronDown className="w-8 h-8" />
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
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg"
                >
                    <Pickaxe className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">铲取</span>
                </Button>

                {/* 2. Dump */}
                <Button
                    onClick={dump}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg"
                >
                    <RotateCw className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">倾倒</span>
                </Button>

                {/* 3. Center: EMERGENCY STOP (Anchors the group) */}
                <Button
                    onClick={stop}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[160px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/20 text-[#FD802E] border border-[#FD802E]/40 rounded-2xl hover:bg-red-500 hover:text-white transition-all font-black shadow-2xl shadow-[#FD802E]/20 ring-1 ring-[#FD802E]/20"
                >
                    <AlertOctagon className="w-6 h-6" />
                    <span className="text-[14px] tracking-[0.3em]">停止</span>
                </Button>

                {/* 4. Dock */}
                <Button
                    onClick={dock}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg"
                >
                    <LogOut className="w-5 h-5 -rotate-90" />
                    <span className="text-[10px] tracking-[0.2em]">回桩</span>
                </Button>

                {/* 5. Reset */}
                <Button
                    onClick={reset}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg"
                >
                    <RotateCcw className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">重置</span>
                </Button>
            </div>
        </Card>
    );
}
