/**
 * Robot Control Panel
 * Manual control interface for the sugar harvesting robot.
 */

import { useState } from 'react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { Joystick } from './ui/Joystick';
import { Slider } from './ui/slider';
import { Label } from './ui/label';
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
        error,
        clearError,
    } = useRobotController();

    const [servoAngles, setServoAngles] = useState({ lift: 0, dump: 0 });
    const [moveSpeed, setMoveSpeed] = useState(0.5);
    const [moveDuration, setMoveDuration] = useState(1.0);

    const handleJoystickMove = async (direction: 'forward' | 'backward' | 'left' | 'right' | null, intensity: number) => {
        if (!direction) return;

        const directionMap = {
            'forward': MoveDirection.FORWARD,
            'backward': MoveDirection.BACKWARD,
            'left': MoveDirection.LEFT,
            'right': MoveDirection.RIGHT,
        };

        const adjustedSpeed = Math.max(0.1, moveSpeed * intensity);

        try {
            await move(directionMap[direction], adjustedSpeed, 0.1);
        } catch (e) {
            console.error('Move failed:', e);
        }
    };

    const handleJoystickStop = async () => {
        try {
            await stop();
        } catch (e) {
            console.error('Stop failed:', e);
        }
    };

    const handleDirectionClick = async (direction: MoveDirection) => {
        try {
            await move(direction, moveSpeed, moveDuration);
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

    const getStateColor = (state: RobotState) => {
        switch (state) {
            case RobotState.IDLE:
                return 'text-green-500';
            case RobotState.MOVING:
                return 'text-blue-500';
            case RobotState.SCOOPING:
                return 'text-yellow-500';
            case RobotState.DUMPING:
                return 'text-orange-500';
            case RobotState.ERROR:
                return 'text-red-500';
            case RobotState.EMERGENCY_STOP:
                return 'text-red-600';
            default:
                return 'text-slate-500';
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
            <Card className={cn('p-4', className)}>
                <div className="text-red-500 text-center">
                    <p className="font-medium">错误</p>
                    <p className="text-sm">{error}</p>
                    <Button onClick={clearError} variant="outline" className="mt-2">
                        关闭
                    </Button>
                </div>
            </Card>
        );
    }

    return (
        <Card className={cn('p-4 space-y-4', className)}>
            {/* Status Header */}
            <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">机器人控制</h3>
                {status && (
                    <div className={cn('text-sm font-medium', getStateColor(status.state))}>
                        {getStateLabel(status.state)}
                    </div>
                )}
            </div>

            {/* Battery Level */}
            {status && (
                <div className="flex items-center gap-2">
                    <span className="text-sm text-slate-400">电池:</span>
                    <div className="flex-1 h-2 bg-slate-700 rounded-full overflow-hidden">
                        <div
                            className={cn(
                                'h-full transition-all',
                                status.battery_level > 50 ? 'bg-green-500' :
                                status.battery_level > 20 ? 'bg-yellow-500' : 'bg-red-500'
                            )}
                            style={{ width: `${status.battery_level}%` }}
                        />
                    </div>
                    <span className="text-sm">{status.battery_level.toFixed(0)}%</span>
                </div>
            )}

            {/* Joystick */}
            <div className="flex justify-center">
                <Joystick
                    onMove={handleJoystickMove}
                    onStop={handleJoystickStop}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                />
            </div>

            {/* Direction Buttons */}
            <div className="grid grid-cols-3 gap-2">
                <div />
                <Button
                    variant="outline"
                    size="icon"
                    onClick={() => handleDirectionClick(MoveDirection.FORWARD)}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    ↑
                </Button>
                <div />
                <Button
                    variant="outline"
                    size="icon"
                    onClick={() => handleDirectionClick(MoveDirection.LEFT)}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    ←
                </Button>
                <Button
                    variant="outline"
                    size="icon"
                    onClick={() => handleDirectionClick(MoveDirection.STOP)}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    ■
                </Button>
                <Button
                    variant="outline"
                    size="icon"
                    onClick={() => handleDirectionClick(MoveDirection.RIGHT)}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    →
                </Button>
                <div />
                <Button
                    variant="outline"
                    size="icon"
                    onClick={() => handleDirectionClick(MoveDirection.BACKWARD)}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    ↓
                </Button>
                <div />
            </div>

            {/* Speed & Duration Controls */}
            <div className="space-y-3">
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">移动速度</Label>
                        <span className="text-xs text-slate-400">{(moveSpeed * 100).toFixed(0)}%</span>
                    </div>
                    <Slider
                        value={[moveSpeed * 100]}
                        onValueChange={([v]: [number]) => setMoveSpeed(v / 100)}
                        min={10}
                        max={100}
                        step={10}
                        disabled={status?.state === RobotState.EMERGENCY_STOP}
                    />
                </div>
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">持续时间</Label>
                        <span className="text-xs text-slate-400">{moveDuration.toFixed(1)}s</span>
                    </div>
                    <Slider
                        value={[moveDuration * 10]}
                        onValueChange={([v]: [number]) => setMoveDuration(v / 10)}
                        min={1}
                        max={50}
                        step={1}
                        disabled={status?.state === RobotState.EMERGENCY_STOP}
                    />
                </div>
            </div>

            {/* Servo Controls */}
            <div className="space-y-3 pt-2 border-t border-slate-700">
                <h4 className="text-sm font-medium">伺服控制</h4>

                {/* Lift Servo */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">铲斗举升</Label>
                        <span className="text-xs text-slate-400">{servoAngles.lift}°</span>
                    </div>
                    <Slider
                        value={[servoAngles.lift]}
                        onValueChange={([v]: [number]) => handleServoChange('lift', v)}
                        min={0}
                        max={180}
                        step={1}
                        disabled={status?.state === RobotState.EMERGENCY_STOP}
                    />
                </div>

                {/* Dump Servo */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">翻斗</Label>
                        <span className="text-xs text-slate-400">{servoAngles.dump}°</span>
                    </div>
                    <Slider
                        value={[servoAngles.dump]}
                        onValueChange={([v]: [number]) => handleServoChange('dump', v)}
                        min={0}
                        max={180}
                        step={1}
                        disabled={status?.state === RobotState.EMERGENCY_STOP}
                    />
                </div>
            </div>

            {/* Action Buttons */}
            <div className="grid grid-cols-2 gap-2 pt-2 border-t border-slate-700">
                <Button
                    variant="default"
                    onClick={scoop}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    铲取
                </Button>
                <Button
                    variant="default"
                    onClick={dump}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    倾倒
                </Button>
            </div>

            {/* Emergency Controls */}
            <div className="grid grid-cols-2 gap-2 pt-2 border-t border-slate-700">
                <Button
                    variant="destructive"
                    onClick={stop}
                    disabled={status?.state === RobotState.EMERGENCY_STOP}
                >
                    紧急停止
                </Button>
                <Button
                    variant="outline"
                    onClick={reset}
                >
                    重置状态
                </Button>
            </div>

            {/* Position Info */}
            {status && (
                <div className="text-xs text-slate-400 space-y-1 pt-2 border-t border-slate-700">
                    <div>位置: [{status.current_position.map((v, i) => `${['X', 'Y', 'Z'][i]}=${v.toFixed(2)}m`).join(', ')}]</div>
                    <div>朝向: Roll={status.orientation[0].toFixed(1)}° Pitch={status.orientation[1].toFixed(1)}° Yaw={status.orientation[2].toFixed(1)}°</div>
                </div>
            )}
        </Card>
    );
}
