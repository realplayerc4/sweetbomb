import { useState, useEffect } from 'react';
import { Card } from './ui/card';
import { Button } from './ui/button';
import { cn } from '../lib/utils';
import { Network, Play, Pause, Zap, Download, Save } from 'lucide-react';
import { useRobotController } from '../hooks/useRobotController';
import { usePathMap } from '../hooks/usePathMap';
import type { SugarHarvestConfig } from '../services/robotApi';

interface BTNodeConfig {
    name: string;
    type: 'sequence' | 'selector' | 'action' | 'condition' | 'repeat';
    status?: 'idle' | 'running' | 'success' | 'failure';
    children?: BTNodeConfig[];
}

interface BehaviorTreeVizProps {
    className?: string;
    currentNode: string;
    status: 'idle' | 'running' | 'success' | 'failure';
    sugarHeight: number;
    heightThreshold: number;
}

const SUGAR_HARVEST_TREE: BTNodeConfig = {
    name: 'SugarHarvestMainLoop',
    type: 'repeat',
    children: [
        { name: 'NavigateToSugarPoint', type: 'action' },
        { name: 'CheckShovelFlat', type: 'action' },
        { name: 'NavigateToDumpPoint', type: 'action' },
        { name: 'DumpAction', type: 'action' },
    ],
};

const STEP_NAMES: Record<string, string> = {
    'NavigateToSugarPoint': '导航到取糖点',
    'AnalyzeSugarDistance': '分析距离与高度',
    'MoveForwardToScoop': '前进铲糖 / 推垛',
    'ReverseToNavPoint': '原路倒退',
    'NavigateToDumpPoint': '导航到卸载点',
    'DumpAction': '翻斗卸载',
    'SwitchToPushMode': '切换推垛姿态',
    'SugarHarvestMainLoop': '铲糖主循环',
    'PushModeMainLoop': '推垛模式',
    'HeightCheck': '检查糖堆高度',
    'CheckShovelFlat': '翻斗加载',
};

const STEP_ICONS: Record<string, string> = {
    'NavigateToSugarPoint': '📍',
    'AnalyzeSugarDistance': '📏',
    'MoveForwardToScoop': '⬆️',
    'ReverseToNavPoint': '⬇️',
    'NavigateToDumpPoint': '📍',
    'DumpAction': '🗑️',
    'SwitchToPushMode': '🔄',
    'SugarHarvestMainLoop': '🔄',
    'FullHarvestCycle': '⚙️',
    'HeightCheck': '📏',
    'ContinueHarvest': '✅',
    'CheckShovelFlat': '⚖️',
};

function TreeNode({ node, currentNode, depth = 0 }: { node: BTNodeConfig; currentNode: string; depth?: number }) {
    const isCurrent = node.name === currentNode;
    const isSuccess = node.status === 'success' || (depth === 1 && !isCurrent);

    return (
        <div
            className={cn('flex flex-col items-start', depth > 0 && 'ml-6')}
            data-testid="bt-node"
        >
            <div
                className={cn(
                    'group inline-flex items-center gap-2.5 px-3 py-1.5 mb-3 rounded-[8px] text-[12px] font-bold tracking-wider transition-all duration-300 w-auto select-none backdrop-blur-md',
                    isSuccess
                        ? 'bg-[#1c1c1e] text-[#FD802E] border border-white/5 shadow-sm hover:bg-[#2a2a2e]/80 hover:border-white/10'
                        : isCurrent
                            ? 'bg-[#FD802E]/10 border border-[#FD802E]/60 text-white ring-1 ring-[#FD802E]/30 shadow-[0_0_20px_rgba(253,128,46,0.15)] scale-[1.02]'
                            : 'bg-[#1c1c1e]/40 border border-transparent text-slate-500 opacity-60 hover:opacity-100 hover:bg-[#1c1c1e]/80 hover:border-[#FD802E]/20'
                )}
            >
                <div className={cn(
                    "flex items-center justify-center w-5 h-5 rounded-[4px] bg-black/40 text-[10px] font-mono shadow-inner transition-colors",
                    isCurrent ? "text-[#FD802E] bg-[#FD802E]/20" : "text-slate-400 group-hover:text-[#FD802E]/80"
                )}>
                    {node.type === 'sequence' && '→'}
                    {node.type === 'selector' && '?'}
                    {node.type === 'repeat' && '↻'}
                    {node.type === 'action' && '⚡'}
                </div>
                <span className="flex items-center gap-2">
                    <span className="text-[14px] filter drop-shadow-sm transition-transform group-hover:scale-110">{STEP_ICONS[node.name] || '⚡'}</span>
                    <span className={cn(
                        "whitespace-nowrap transition-colors translate-y-[0.5px]",
                        (STEP_NAMES[node.name] === '翻斗加载' || STEP_NAMES[node.name] === '翻斗卸载')
                            ? "text-[#FD802E]"
                            : "text-slate-200 group-hover:text-white"
                    )}>
                        {STEP_NAMES[node.name] || node.name}
                    </span>
                </span>
                {isCurrent && (
                    <span className="ml-1.5 flex items-center justify-center relative w-2 h-2">
                        <span className="absolute w-2.5 h-2.5 rounded-full bg-[#FD802E] animate-ping opacity-75" />
                        <span className="relative w-1.5 h-1.5 rounded-full bg-[#FD802E]" />
                    </span>
                )}
                {isSuccess && (
                    <span className="ml-1.5 text-[12px] text-green-500/80 mr-0.5">✓</span>
                )}
            </div>

            {node.children && node.children.length > 0 && (
                <div className="flex flex-col items-start">
                    {node.children.map((child, idx) => (
                        <TreeNode key={`${child.name}-${idx}`} node={child} currentNode={currentNode} depth={depth + 1} />
                    ))}
                </div>
            )}
        </div>
    );
}

function CircularHarvestFlow({
    nodes,
    currentNode,
    onStartCycle,
    isHarvestRunning
}: {
    nodes: BTNodeConfig[],
    currentNode: string,
    onStartCycle?: () => void,
    isHarvestRunning?: boolean
}) {
    return (
        <div className="flex items-center justify-center gap-4 w-full">
            {nodes.map((node, i) => {
                const isCurrent = node.name === currentNode;
                const currentIdx = nodes.findIndex(n => n.name === currentNode);
                const isSuccess = currentIdx > -1 && i < currentIdx;
                const isLast = i === nodes.length - 1;

                const isNavNode = node.name === 'NavigateToSugarPoint' || node.name === 'NavigateToDumpPoint';
                const isInteractive = node.name === 'NavigateToSugarPoint' && !isHarvestRunning;

                return (
                    <div key={node.name} className="flex items-center gap-2">
                        <div
                            className={cn(
                                'group flex items-center gap-2 px-3 py-2 rounded-full font-bold tracking-wider transition-all duration-300 select-none backdrop-blur-md whitespace-nowrap shadow-lg',
                                isInteractive ? 'cursor-pointer' : 'cursor-default',
                                isSuccess
                                    ? 'bg-[#1c1c1e] text-[#FD802E] border border-white/5'
                                    : isCurrent
                                        ? 'bg-[#FD802E]/15 border border-[#FD802E]/60 text-white ring-2 ring-[#FD802E]/40 shadow-[0_0_25px_rgba(253,128,46,0.3)]'
                                        : isNavNode
                                            ? 'bg-[#FD802E]/20 text-[#FD802E] border-2 border-[#FD802E] shadow-[0_0_15px_rgba(253,128,46,0.2)] hover:bg-[#FD802E] hover:text-white hover:scale-105 scale-105'
                                            : 'bg-[#1c1c1e]/60 border border-transparent text-slate-400 hover:text-slate-200'
                            )}
                            onClick={isInteractive ? onStartCycle : undefined}
                        >
                            <span className="text-[14px] drop-shadow-sm">{STEP_ICONS[node.name] || '⚡'}</span>
                            <span className={cn(
                                "text-[12px]",
                                (STEP_NAMES[node.name] === '翻斗加载' || STEP_NAMES[node.name] === '翻斗卸载')
                                    ? "text-[#FD802E]"
                                    : ""
                            )}>
                                {STEP_NAMES[node.name] || node.name}
                            </span>

                            {isCurrent && (
                                <span className="ml-1 flex items-center justify-center relative w-2 h-2">
                                    <span className="absolute w-2.5 h-2.5 rounded-full bg-[#FD802E] animate-ping opacity-75" />
                                    <span className="relative w-1.5 h-1.5 rounded-full bg-[#FD802E]" />
                                </span>
                            )}
                            {isSuccess && (
                                <span className="ml-1 text-[10px] text-green-500/80 mr-0.5">✓</span>
                            )}
                        </div>

                        {!isLast && (
                            <span className={cn(
                                'text-[14px] transition-all duration-300',
                                (isCurrent || isSuccess) ? 'text-[#FD802E]' : 'text-slate-500'
                            )}>→</span>
                        )}
                    </div>
                );
            })}
        </div>
    );
}



export function BehaviorTreeViz({
    className,
    currentNode = '',
    status = 'idle',
    sugarHeight: _sugarHeight = 0.3,
    heightThreshold: _heightThreshold = 0.20,
}: BehaviorTreeVizProps) {
    void _sugarHeight;
    void _heightThreshold;
    const { startSugarHarvest, isHarvestRunning, status: robotStatus } = useRobotController();
    const { loadPathMap, updatePickStation, pickStations, isLoading: pathMapLoading } = usePathMap();

    // 糖堆位置：从 localStorage 恢复上次值
    const [sugarX, setSugarX] = useState(() => localStorage.getItem('sugarX') || '');
    const [sugarY, setSugarY] = useState(() => localStorage.getItem('sugarY') || '');
    const [sugarR, setSugarR] = useState(() => localStorage.getItem('sugarR') || '');
    const [sugarS, setSugarS] = useState(() => localStorage.getItem('sugarS') || '');

    // 保存糖堆位置到 localStorage
    useEffect(() => { localStorage.setItem('sugarX', sugarX); }, [sugarX]);
    useEffect(() => { localStorage.setItem('sugarY', sugarY); }, [sugarY]);
    useEffect(() => { localStorage.setItem('sugarR', sugarR); }, [sugarR]);
    useEffect(() => { localStorage.setItem('sugarS', sugarS); }, [sugarS]);

    const currentTree = SUGAR_HARVEST_TREE;

    return (
        <Card
            data-vibe="bt-panel-v5"
            className={cn('relative bg-[#1A1A1E] border-[#FD802E]/20 shadow-[0_0_25px_rgba(253,128,46,0.1)] rounded-[10px] flex flex-col h-full overflow-hidden min-h-[300px]', className)}
        >
            {/* Top Bar Area - Status Capsule CENTERED */}
            <div className="absolute top-[12px] left-1/2 -translate-x-1/2 z-[110] flex items-center gap-3 bg-[#1c1c1e]/95 backdrop-blur-xl px-8 py-2.5 rounded-full border border-[#FD802E]/40 shadow-[0_0_25px_rgba(253,128,46,0.5)]">
                <div className={cn('w-2 h-2 rounded-full shadow-[0_0_12px_rgba(253,128,46,1)]',
                    status === 'running' ? 'bg-[#FD802E] animate-pulse' : 'bg-green-500')} />
                <Network className="w-4 h-4 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-black tracking-[0.2em] uppercase font-mono whitespace-nowrap">
                    任务系统 | {status === 'running' ? '运行中' : '空闲'}
                </span>
            </div>

            {/* Position Area - Below Capsule, Left Aligned */}
            <div className="absolute top-[55px] left-[20px] flex flex-col items-start gap-1.5 z-[100]">
                {/* Charge Position */}
                <div className="flex items-center">
                    <span className="text-[12px] font-black text-slate-400 uppercase tracking-wider w-[70px] text-right pr-2">充电位置</span>
                    <span className="text-[12px] font-bold text-slate-400">X:</span>
                    <span className="text-[12px] text-slate-300 font-bold w-[60px] text-center tabular-nums">--</span>
                    <span className="text-[12px] text-slate-500 w-[20px]">mm</span>
                    <span className="text-[12px] text-slate-600 mx-1">|</span>
                    <span className="text-[12px] font-bold text-slate-400">Y:</span>
                    <span className="text-[12px] text-slate-300 font-bold w-[60px] text-center tabular-nums">--</span>
                    <span className="text-[12px] text-slate-500 w-[20px]">mm</span>
                    <span className="text-[12px] text-slate-600 mx-1">|</span>
                    <span className="text-[12px] font-bold text-slate-400">n:</span>
                    <span className="text-[12px] text-slate-300 font-bold w-[40px] text-center tabular-nums">--</span>
                </div>
                {/* Dump Position */}
                <div className="flex items-center">
                    <span className="text-[12px] font-black text-slate-400 uppercase tracking-wider w-[70px] text-right pr-2">卸载位置</span>
                    <span className="text-[12px] font-bold text-slate-400">X:</span>
                    <span className="text-[12px] text-slate-300 font-bold w-[60px] text-center tabular-nums">--</span>
                    <span className="text-[12px] text-slate-500 w-[20px]">mm</span>
                    <span className="text-[12px] text-slate-600 mx-1">|</span>
                    <span className="text-[12px] font-bold text-slate-400">Y:</span>
                    <span className="text-[12px] text-slate-300 font-bold w-[60px] text-center tabular-nums">--</span>
                    <span className="text-[12px] text-slate-500 w-[20px]">mm</span>
                    <span className="text-[12px] text-slate-600 mx-1">|</span>
                    <span className="text-[12px] font-bold text-slate-400">n:</span>
                    <span className="text-[12px] text-slate-300 font-bold w-[40px] text-center tabular-nums">--</span>
                </div>
                {/* Vehicle Position (display only) */}
                <div className="flex items-center">
                    <span className="text-[12px] font-black text-[#FD802E] uppercase tracking-wider w-[70px] text-right pr-2">车辆位置</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">X:</span>
                    <span className="text-[12px] text-white font-bold w-[60px] text-center tabular-nums">
                        {robotStatus && typeof robotStatus.x === 'number' ? robotStatus.x.toFixed(0) : '--'}
                    </span>
                    <span className="text-[12px] text-[#FD802E]/50 w-[20px]">mm</span>
                    <span className="text-[12px] text-[#FD802E]/30 mx-1">|</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">Y:</span>
                    <span className="text-[12px] text-white font-bold w-[60px] text-center tabular-nums">
                        {robotStatus && typeof robotStatus.y === 'number' ? robotStatus.y.toFixed(0) : '--'}
                    </span>
                    <span className="text-[12px] text-[#FD802E]/50 w-[20px]">mm</span>
                    <span className="text-[12px] text-[#FD802E]/30 mx-1">|</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">A:</span>
                    <span className="text-[12px] text-white font-bold w-[50px] text-center tabular-nums">
                        {robotStatus && typeof robotStatus.a === 'number' ? robotStatus.a.toFixed(1) : '--'}
                    </span>
                    <span className="text-[12px] text-[#FD802E]/50 w-[10px]">°</span>
                </div>
                {/* Sugar Position (display only, same style as vehicle) */}
                <div className="flex items-center">
                    <span className="text-[12px] font-black text-[#FD802E] uppercase tracking-wider w-[70px] text-right pr-2">糖堆位置</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">X:</span>
                    <span className="text-[12px] text-white font-bold w-[60px] text-center tabular-nums">
                        {sugarX || '--'}
                    </span>
                    <span className="text-[12px] text-[#FD802E]/50 w-[20px]">mm</span>
                    <span className="text-[12px] text-[#FD802E]/30 mx-1">|</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">Y:</span>
                    <span className="text-[12px] text-white font-bold w-[60px] text-center tabular-nums">
                        {sugarY || '--'}
                    </span>
                    <span className="text-[12px] text-[#FD802E]/50 w-[20px]">mm</span>
                    <span className="text-[12px] text-[#FD802E]/30 mx-1">|</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">R:</span>
                    <span className="text-[12px] text-white font-bold w-[60px] text-center tabular-nums">
                        {sugarR || '--'}
                    </span>
                    <span className="text-[12px] text-[#FD802E]/50 w-[20px]">mm</span>
                    <span className="text-[12px] text-[#FD802E]/30 mx-1">|</span>
                    <span className="text-[12px] font-bold text-[#FD802E]">S:</span>
                    <span className="text-[12px] text-white font-bold w-[40px] text-center tabular-nums">
                        {sugarS || '--'}
                    </span>
                </div>
            </div>

            {/* Main Interactive Layer - Centered */}
            <div className="flex-1 flex items-center justify-center mt-[130px] mb-[70px] overflow-hidden px-6">
                <div className="w-full max-w-[400px] flex flex-col items-center justify-center">
                    <div className="flex items-center justify-center">
                        {currentTree.children ? (
                            <CircularHarvestFlow
                                nodes={currentTree.children}
                                currentNode={currentNode}
                                isHarvestRunning={isHarvestRunning}
                                onStartCycle={async () => {
                                    if (!isHarvestRunning) {
                                        const defaultConfig: SugarHarvestConfig = {
                                            navigation_point: [1.0, 0.0],
                                            dump_point: [0.0, 1.0],
                                            bucket_width_m: 0.6,
                                            scoop_position: 90.0,
                                            dump_position: 135.0,
                                            max_cycles: 10,
                                            height_threshold_m: 0.20,
                                        };
                                        try {
                                            await startSugarHarvest(defaultConfig);
                                        } catch (e) {
                                            console.error("Failed to start harvest from BT interaction", e);
                                        }
                                    }
                                }}
                            />
                        ) : (
                            <TreeNode node={currentTree} currentNode={currentNode} />
                        )}
                    </div>
                </div>
            </div>

            {/* Bottom Action Buttons */}
            <div className="absolute bottom-[20px] left-1/2 -translate-x-1/2 flex justify-center items-center gap-[5px] z-[120]">
                <Button
                    onClick={async () => {
                        if (!isHarvestRunning) {
                            const config: SugarHarvestConfig = {
                                navigation_point: [parseFloat(sugarX) || 0, parseFloat(sugarY) || 0],
                                dump_point: [0, 0],
                                bucket_width_m: 0.6,
                                scoop_position: 90.0,
                                dump_position: 135.0,
                                max_cycles: parseInt(sugarS) || 1,
                                height_threshold_m: 0.20,
                            };
                            await startSugarHarvest(config);
                        }
                    }}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <Zap className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">自动任务</span>
                </Button>

                <Button
                    disabled={!isHarvestRunning}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30 disabled:grayscale"
                >
                    <Pause className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">暂停任务</span>
                </Button>

                <Button
                    disabled={isHarvestRunning}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E] text-black border border-[#FD802E] rounded-2xl hover:bg-[#FD802E]/90 hover:-translate-y-1 transition-all font-black shadow-[0_0_20px_rgba(253,128,46,0.4)] disabled:opacity-30 disabled:grayscale"
                >
                    <Play className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">启动任务</span>
                </Button>

                <Button
                    onClick={async () => {
                        const data = await loadPathMap();
                        if (data && data.pick_stations.length > 0) {
                            const ps = data.pick_stations[0];
                            setSugarX(String(ps.ox));
                            setSugarY(String(ps.oy));
                            setSugarR(String(ps.r));
                            setSugarS(String(ps.station_num));
                        }
                    }}
                    disabled={pathMapLoading}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-2xl hover:bg-[#FD802E]/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30"
                >
                    <Download className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">{pathMapLoading ? '读取中' : '读取任务'}</span>
                </Button>

                <Button
                    onClick={async () => {
                        if (pickStations.length > 0) {
                            const ps = pickStations[0];
                            const ok = await updatePickStation(ps.id, {
                                ox: parseFloat(sugarX) || ps.ox,
                                oy: parseFloat(sugarY) || ps.oy,
                                r: parseFloat(sugarR) || ps.r,
                                station_num: parseInt(sugarS) || ps.station_num,
                            });
                            if (ok) {
                                // 重新加载确认保存成功
                                await loadPathMap();
                            }
                        }
                    }}
                    disabled={pickStations.length === 0}
                    className="w-[110px] flex flex-col items-center justify-center gap-1.5 py-8 bg-purple-500/10 text-purple-400 border border-purple-500/20 rounded-2xl hover:bg-purple-500/20 hover:-translate-y-1 transition-all font-black shadow-lg disabled:opacity-30"
                >
                    <Save className="w-5 h-5" />
                    <span className="text-[10px] tracking-[0.2em]">保存参数</span>
                </Button>
            </div>
        </Card>
    );
}