/**
 * Sugar Harvest Panel
 * Configuration and control for the autonomous sugar harvesting cycle.
 */

import { useState } from 'react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { Slider } from './ui/slider';
import { Label } from './ui/label';
import { useRobotController } from '../hooks/useRobotController';
import type { SugarHarvestConfig } from '../services/robotApi';
import { cn } from '../lib/utils';

interface SugarHarvestPanelProps {
    className?: string;
}

export function SugarHarvestPanel({ className }: SugarHarvestPanelProps) {
    const {
        startSugarHarvest,
        stopSugarHarvest,
        harvestStatus,
        isHarvestRunning,
    } = useRobotController();

    const [config, setConfig] = useState<SugarHarvestConfig>({
        navigation_point: [1.0, 0.0],
        dump_point: [0.0, 1.0],
        bucket_width_m: 0.6,
        approach_offset_m: 0.05,
        scoop_position: 90.0,
        dump_position: 135.0,
        max_cycles: 10,
        height_threshold_m: 0.20,
    });

    const handleStart = async () => {
        try {
            await startSugarHarvest(config);
        } catch (e: any) {
            console.error('Failed to start sugar harvest:', e);
        }
    };

    const handleStop = async () => {
        try {
            await stopSugarHarvest();
        } catch (e: any) {
            console.error('Failed to stop sugar harvest:', e);
        }
    };

    const updateNavigationPoint = (index: number, value: string) => {
        const num = parseFloat(value) || 0;
        setConfig(prev => ({
            ...prev,
            navigation_point: [
                index === 0 ? num : prev.navigation_point[0],
                index === 1 ? num : prev.navigation_point[1],
            ] as [number, number],
        }));
    };

    const updateDumpPoint = (index: number, value: string) => {
        const num = parseFloat(value) || 0;
        setConfig(prev => ({
            ...prev,
            dump_point: [
                index === 0 ? num : prev.dump_point[0],
                index === 1 ? num : prev.dump_point[1],
            ] as [number, number],
        }));
    };

    const needsPushMode = harvestStatus?.sugar_height !== undefined &&
        harvestStatus.sugar_height < config.height_threshold_m!;

    return (
        <Card className={cn('p-4 space-y-4', className)}>
            {/* Header */}
            <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">铲糖自主循环</h3>
                {isHarvestRunning && (
                    <div className="flex items-center gap-2">
                        <div className="w-2 h-2 rounded-full bg-green-500 animate-pulse" />
                        <span className="text-sm text-green-500">运行中</span>
                    </div>
                )}
            </div>

            {/* Cycle Progress */}
            {harvestStatus && (
                <div className="bg-slate-800 rounded-lg p-3 space-y-2">
                    <div className="flex items-center justify-between text-sm">
                        <span className="text-slate-400">循环进度</span>
                        <span className="font-medium">
                            {harvestStatus.current_cycle || 0} / {harvestStatus.max_cycles || config.max_cycles}
                        </span>
                    </div>

                    {/* Progress Bar */}
                    <div className="h-2 bg-slate-700 rounded-full overflow-hidden">
                        <div
                            className="h-full bg-primary transition-all duration-300"
                            style={{
                                width: `${((harvestStatus.current_cycle || 0) / (harvestStatus.max_cycles || config.max_cycles)) * 100}%`
                            }}
                        />
                    </div>

                    {/* Sugar Height Indicator */}
                    {harvestStatus.sugar_height !== undefined && (
                        <div className={cn(
                            'flex items-center justify-between text-sm p-2 rounded',
                            needsPushMode ? 'bg-yellow-900/30 text-yellow-400' : 'bg-green-900/30 text-green-400'
                        )}>
                            <span>糖堆高度</span>
                            <span className="font-medium">
                                {(harvestStatus.sugar_height * 100).toFixed(1)} cm
                            </span>
                        </div>
                    )}

                    {needsPushMode && (
                        <div className="text-xs text-yellow-400 text-center">
                            ⚠️ 糖堆高度不足，即将切换到推垛模式
                        </div>
                    )}
                </div>
            )}

            {/* Navigation Points */}
            <div className="space-y-3">
                <h4 className="text-sm font-medium">导航点配置</h4>

                {/* Navigation Point */}
                <div className="space-y-2">
                    <Label className="text-xs text-slate-400">取糖导航点 (X, Y)</Label>
                    <div className="grid grid-cols-2 gap-2">
                        <div>
                            <Label className="text-xs">X</Label>
                            <input
                                type="number"
                                step="0.1"
                                value={config.navigation_point![0]}
                                onChange={(e) => updateNavigationPoint(0, e.target.value)}
                                disabled={isHarvestRunning}
                                className="w-full px-2 py-1 bg-slate-800 border border-slate-700 rounded text-sm focus:outline-none focus:ring-2 focus:ring-primary disabled:opacity-50"
                            />
                        </div>
                        <div>
                            <Label className="text-xs">Y</Label>
                            <input
                                type="number"
                                step="0.1"
                                value={config.navigation_point![1]}
                                onChange={(e) => updateNavigationPoint(1, e.target.value)}
                                disabled={isHarvestRunning}
                                className="w-full px-2 py-1 bg-slate-800 border border-slate-700 rounded text-sm focus:outline-none focus:ring-2 focus:ring-primary disabled:opacity-50"
                            />
                        </div>
                    </div>
                </div>

                {/* Dump Point */}
                <div className="space-y-2">
                    <Label className="text-xs text-slate-400">卸载点 (X, Y)</Label>
                    <div className="grid grid-cols-2 gap-2">
                        <div>
                            <Label className="text-xs">X</Label>
                            <input
                                type="number"
                                step="0.1"
                                value={config.dump_point![0]}
                                onChange={(e) => updateDumpPoint(0, e.target.value)}
                                disabled={isHarvestRunning}
                                className="w-full px-2 py-1 bg-slate-800 border border-slate-700 rounded text-sm focus:outline-none focus:ring-2 focus:ring-primary disabled:opacity-50"
                            />
                        </div>
                        <div>
                            <Label className="text-xs">Y</Label>
                            <input
                                type="number"
                                step="0.1"
                                value={config.dump_point![1]}
                                onChange={(e) => updateDumpPoint(1, e.target.value)}
                                disabled={isHarvestRunning}
                                className="w-full px-2 py-1 bg-slate-800 border border-slate-700 rounded text-sm focus:outline-none focus:ring-2 focus:ring-primary disabled:opacity-50"
                            />
                        </div>
                    </div>
                </div>
            </div>

            {/* Cycle Settings */}
            <div className="space-y-3 pt-2 border-t border-slate-700">
                <h4 className="text-sm font-medium">循环设置</h4>

                {/* Max Cycles */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">最大循环次数</Label>
                        <span className="text-xs text-slate-400">{config.max_cycles}</span>
                    </div>
                    <Slider
                        value={[config.max_cycles!]}
                        onValueChange={([v]) => setConfig(prev => ({ ...prev, max_cycles: v }))}
                        min={1}
                        max={100}
                        step={1}
                        disabled={isHarvestRunning}
                    />
                </div>

                {/* Height Threshold */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">推垛模式切换高度 (cm)</Label>
                        <span className="text-xs text-slate-400">{(config.height_threshold_m! * 100).toFixed(0)}</span>
                    </div>
                    <Slider
                        value={[config.height_threshold_m! * 100]}
                        onValueChange={([v]) => setConfig(prev => ({ ...prev, height_threshold_m: v / 100 }))}
                        min={10}
                        max={50}
                        step={5}
                        disabled={isHarvestRunning}
                    />
                </div>

                {/* Bucket Width */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">铲斗宽度 (m)</Label>
                        <span className="text-xs text-slate-400">{config.bucket_width_m!.toFixed(2)}</span>
                    </div>
                    <Slider
                        value={[config.bucket_width_m! * 100]}
                        onValueChange={([v]) => setConfig(prev => ({ ...prev, bucket_width_m: v / 100 }))}
                        min={40}
                        max={100}
                        step={5}
                        disabled={isHarvestRunning}
                    />
                </div>
            </div>

            {/* Servo Settings */}
            <div className="space-y-3 pt-2 border-t border-slate-700">
                <h4 className="text-sm font-medium">伺服角度设置</h4>

                {/* Scoop Position */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">铲取位置 (°)</Label>
                        <span className="text-xs text-slate-400">{config.scoop_position!.toFixed(0)}</span>
                    </div>
                    <Slider
                        value={[config.scoop_position!]}
                        onValueChange={([v]) => setConfig(prev => ({ ...prev, scoop_position: v }))}
                        min={0}
                        max={180}
                        step={5}
                        disabled={isHarvestRunning}
                    />
                </div>

                {/* Dump Position */}
                <div className="space-y-1">
                    <div className="flex items-center justify-between">
                        <Label className="text-xs">倾倒位置 (°)</Label>
                        <span className="text-xs text-slate-400">{config.dump_position!.toFixed(0)}</span>
                    </div>
                    <Slider
                        value={[config.dump_position!]}
                        onValueChange={([v]) => setConfig(prev => ({ ...prev, dump_position: v }))}
                        min={0}
                        max={180}
                        step={5}
                        disabled={isHarvestRunning}
                    />
                </div>
            </div>

            {/* Control Buttons */}
            <div className="grid grid-cols-2 gap-2 pt-2 border-t border-slate-700">
                {!isHarvestRunning ? (
                    <Button
                        variant="default"
                        onClick={handleStart}
                        className="col-span-2"
                    >
                        启动铲糖循环
                    </Button>
                ) : (
                    <Button
                        variant="destructive"
                        onClick={handleStop}
                        className="col-span-2"
                    >
                        停止铲糖循环
                    </Button>
                )}
            </div>

            {/* Workflow Info */}
            <div className="text-xs text-slate-400 space-y-1 pt-2 border-t border-slate-700">
                <p className="font-medium">循环流程：</p>
                <ol className="list-decimal list-inside space-y-0.5">
                    <li>导航到取糖点</li>
                    <li>RealSense 分析糖堆距离和高度</li>
                    <li>检查糖堆高度（&lt; 20cm 切换推垛模式）</li>
                    <li>前进铲糖</li>
                    <li>原路倒退</li>
                    <li>导航到卸载点</li>
                    <li>翻斗卸载</li>
                    <li>重复直到达到最大循环次数</li>
                </ol>
            </div>
        </Card>
    );
}
