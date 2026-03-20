import { useState } from 'react';
import { Card } from './ui/card';
import { cn } from '../lib/utils';
import { Network } from 'lucide-react';
import { useRobotController } from '../hooks/useRobotController';
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
    cycleCount: number;
    maxCycles: number;
    sugarHeight: number;
    heightThreshold: number;
    stepHistory: string[];
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

const PUSH_MODE_TREE: BTNodeConfig = {
    name: 'PushModeMainLoop',
    type: 'repeat',
    children: [
        { name: 'AnalyzeSugarDistance', type: 'action' },
        { name: 'HeightCheck', type: 'condition' },
        { name: 'SwitchToPushMode', type: 'action' },
        { name: 'MoveForwardToScoop', type: 'action' },
        { name: 'ReverseToNavPoint', type: 'action' }
    ]
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
    // For visual testing, we treat nodes as success if they have specific icons or are manually flagged
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
    status,
    onStartCycle,
    isHarvestRunning
}: {
    nodes: BTNodeConfig[],
    currentNode: string,
    status: string,
    onStartCycle?: () => void,
    isHarvestRunning?: boolean
}) {
    // Adjusted radii for elliptical flow (vertical spacing reduced by half)
    const radiusX = 80;
    const radiusY = 50;
    const totalNodes = nodes.length;
    const cx = 170; // Re-centered to avoid clipping when justified start

    return (
        <div className="relative w-full h-[180px] flex items-center justify-start ml-4 mt-4">
            {/* Background SVG Canvas for connecting arcs */}
            <svg className="absolute left-0 top-1/2 -translate-y-1/2 w-full h-[200px] pointer-events-none z-0 overflow-visible">
                <defs>
                    <marker id="arrow" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
                        <path d="M 0 0 L 10 5 L 0 10 z" fill="#4B5563" />
                    </marker>
                    <marker id="arrow-active" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
                        <path d="M 0 0 L 10 5 L 0 10 z" fill="#FD802E" />
                    </marker>
                </defs>

                {/* Ellgetical path with markers */}
                <ellipse cx={cx} cy="100" rx={radiusX} ry={radiusY} stroke="#2a2a2e" strokeWidth="2" strokeDasharray="4 6" fill="none" />

                {/* Generate directional arrows precisely spaced on the elliptical track */}
                {nodes.map((_, i) => {
                    const angleOffset = (i * 360 / totalNodes - 90 + (180 / totalNodes)) * (Math.PI / 180);
                    const cy = 100;
                    const startX = cx + Math.cos(angleOffset - 0.1) * radiusX;
                    const startY = cy + Math.sin(angleOffset - 0.1) * radiusY;
                    const endX = cx + Math.cos(angleOffset) * radiusX;
                    const endY = cy + Math.sin(angleOffset) * radiusY;

                    const currentIdx = nodes.findIndex(n => n.name === currentNode);
                    const isFlowActive = status === 'running' && (currentIdx === i || currentIdx === (i + 1) % totalNodes);

                    return (
                        <path
                            key={`arrow-${i}`}
                            d={`M ${startX} ${startY} L ${endX} ${endY}`}
                            stroke={isFlowActive ? '#FD802E' : '#4B5563'}
                            strokeWidth="2"
                            fill="none"
                            markerEnd={isFlowActive ? 'url(#arrow-active)' : 'url(#arrow)'}
                            className={cn('transition-all duration-300', isFlowActive && 'opacity-100')}
                        />
                    );
                })}

                {status === 'running' && (
                    <ellipse cx={cx} cy="100" rx={radiusX} ry={radiusY} stroke="#FD802E" strokeWidth="2" strokeDasharray="80 1000" fill="none" strokeLinecap="round" className="animate-[spin_3s_linear_infinite]" style={{ transformOrigin: `${cx}px 100px` }} />
                )}
            </svg>

            {nodes.map((node, i) => {
                const angle = (i * 360 / totalNodes - 90) * (Math.PI / 180);
                const x = Math.cos(angle) * radiusX;
                const y = Math.sin(angle) * radiusY;
                const isCurrent = node.name === currentNode;
                const currentIdx = nodes.findIndex(n => n.name === currentNode);
                const isSuccess = currentIdx > -1 && i < currentIdx;

                const isNavNode = node.name === 'NavigateToSugarPoint' || node.name === 'NavigateToDumpPoint';
                const isInteractive = node.name === 'NavigateToSugarPoint' && !isHarvestRunning;

                const baseNodeContent = (
                    <div
                        className={cn(
                            'group flex items-center gap-2.5 px-3 py-2 rounded-full font-bold tracking-wider transition-all duration-300 w-auto select-none backdrop-blur-md whitespace-nowrap shadow-lg',
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
                        <span className="text-[16px] drop-shadow-sm">{STEP_ICONS[node.name] || '⚡'}</span>
                        <span className={cn(
                            "text-[14px]",
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
                            <span className="ml-1 text-[12px] text-green-500/80 mr-0.5">✓</span>
                        )}
                    </div>
                );

                return (
                    <div
                        key={node.name}
                        className={cn(
                            'absolute flex flex-col items-center justify-center text-center transition-all duration-500',
                            isCurrent ? 'scale-110 z-10' : 'scale-100 z-0 opacity-90'
                        )}
                        style={{
                            left: `${cx}px`,
                            top: '50%',
                            transform: `translate(${x}px, calc(-50% + ${y}px)) translate(-50%, 0)`,
                        }}
                    >
                        {baseNodeContent}
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
    cycleCount = 0,
    maxCycles = 10,
    sugarHeight = 0.3,
    heightThreshold = 0.20,
    stepHistory = [],
}: BehaviorTreeVizProps) {
    const { startSugarHarvest, isHarvestRunning } = useRobotController();
    const needsPushMode = sugarHeight < heightThreshold;
    const [activeTab, setActiveTab] = useState<'harvest' | 'push'>('harvest');

    const currentTree = activeTab === 'harvest' ? SUGAR_HARVEST_TREE : PUSH_MODE_TREE;
    const currentTabName = activeTab === 'harvest' ? '铲糖主循环' : '推垛模式';

    return (
        <Card
            data-vibe="bt-panel-v5"
            className={cn('relative bg-[#1A1A1E] border-[#FD802E]/20 shadow-[0_0_25px_rgba(253,128,46,0.1)] rounded-[10px] flex flex-col h-full overflow-hidden min-h-[300px]', className)}
        >
            {/* Top Bar Area - Independent Absolute Elements for Pixel-Perfect Centering */}
            {/* 1. Left: System Cycles */}
            <div className="absolute top-[16px] left-[32px] z-[100] flex flex-col items-start">
                <span className="text-[7px] font-black tracking-[0.3em] text-[#FD802E]/40 uppercase">System Cycles</span>
                <span className="text-[12px] font-black text-[#FD802E] tracking-tighter">系统循环: {cycleCount} / {maxCycles}</span>
            </div>

            {/* 2. Center: Status Capsule - PHYSICALLY CENTERED */}
            <div className="absolute top-[12px] left-1/2 -translate-x-1/2 z-[110] flex items-center gap-3 bg-[#1c1c1e]/95 backdrop-blur-xl px-8 py-2.5 rounded-full border border-[#FD802E]/40 shadow-[0_0_25px_rgba(253,128,46,0.5)]">
                <div className={cn('w-2 h-2 rounded-full shadow-[0_0_12px_rgba(253,128,46,1)]',
                    status === 'running' ? 'bg-[#FD802E] animate-pulse' : 'bg-green-500')} />
                <Network className="w-4 h-4 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-black tracking-[0.2em] uppercase font-mono whitespace-nowrap">
                    {currentTabName} | {status === 'running' ? '运行中' : '空闲'}
                </span>
            </div>

            {/* 3. Right: Sugar Height Parameter */}
            <div className="absolute top-[16px] right-[32px] z-[100] flex flex-col items-end">
                <span className={cn('text-[7px] font-black tracking-[0.3em] uppercase',
                    needsPushMode ? 'text-red-500/60' : 'text-[#FD802E]/40'
                )}>Sugar Pile Height</span>
                <span className={cn('text-[12px] font-black leading-none',
                    needsPushMode ? 'text-red-500' : 'text-[#FD802E]'
                )}>堆体高度: {(sugarHeight * 100).toFixed(1)} cm</span>
            </div>

            {/* Main Interactive Layer - Absolute Positioning for Precision */}
            <div className="flex-1 relative mt-[60px] mb-[60px] overflow-y-auto no-scrollbar px-6">
                {/* Tree Visual Area */}
                <div className="space-y-4">
                    <div className="text-[10px] font-black text-[#FD802E] uppercase tracking-[0.4em] px-1">
                        {activeTab === 'harvest' ? 'HARVEST FLOW' : 'PUSH FLOW'}
                    </div>
                    <div className="max-h-[500px]">
                        {activeTab === 'harvest' && currentTree.children ? (
                            <CircularHarvestFlow
                                nodes={currentTree.children}
                                currentNode={currentNode}
                                status={status}
                                isHarvestRunning={isHarvestRunning}
                                onStartCycle={async () => {
                                    if (!isHarvestRunning) {
                                        // Trigger system cycle with default config inline. Usually this comes from panel, but User wants clickable Flow
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

                {/* Footnotes / History */}
                {stepHistory.length > 0 && (
                    <div className="mt-8 pt-4 border-t border-white/5">
                        <div className="text-[8px] font-black text-slate-600 uppercase tracking-[0.2em] mb-2">RECENT_LOGS</div>
                        <div className="flex flex-wrap gap-2 opacity-40">
                            {stepHistory.slice(-3).reverse().map((step, idx) => (
                                <span key={idx} className="px-2 py-0.5 rounded-full bg-white/5 text-[8px] text-[#FD802E] font-bold">
                                    {STEP_NAMES[step] || step}
                                </span>
                            ))}
                        </div>
                    </div>
                )}
            </div>
            {/* Bottom Tabs (Excel Sheet Style) */}
            <div className="absolute bottom-0 left-0 right-0 flex justify-start px-2 bg-[#121214] pt-2 border-t border-[#FD802E]/20 rounded-b-[10px] z-[120]">
                <button
                    onClick={() => setActiveTab('harvest')}
                    className={cn(
                        'relative px-5 py-2.5 text-[12px] font-bold tracking-wider rounded-t-lg transition-all',
                        activeTab === 'harvest'
                            ? 'bg-[#1c1c1e] text-[#FD802E] border-t border-l border-r border-[#2a2a2e] shadow-[0_-4px_10px_rgba(0,0,0,0.2)] z-10'
                            : 'bg-transparent text-slate-500 hover:text-slate-300 hover:bg-white/5 border-t border-l border-r border-transparent'
                    )}
                >
                    铲糖主循环
                    {activeTab === 'harvest' && (
                        <span className="absolute bottom-[-1px] left-0 right-0 h-[3px] bg-[#1c1c1e]" />
                    )}
                </button>
                <button
                    onClick={() => setActiveTab('push')}
                    className={cn(
                        'relative px-5 py-2.5 text-[12px] font-bold tracking-wider rounded-t-lg transition-all',
                        activeTab === 'push'
                            ? 'bg-[#1c1c1e] text-[#FD802E] border-t border-l border-r border-[#2a2a2e] shadow-[0_-4px_10px_rgba(0,0,0,0.2)] z-10'
                            : 'bg-transparent text-slate-500 hover:text-slate-300 hover:bg-white/5 border-t border-l border-r border-transparent'
                    )}
                >
                    推垛模式
                    {activeTab === 'push' && (
                        <span className="absolute bottom-[-1px] left-0 right-0 h-[3px] bg-[#1c1c1e]" />
                    )}
                </button>
            </div>
        </Card>
    );
}
