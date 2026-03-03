/**
 * Behavior Tree Visualization
 * Final Strict Version: Absolute Centered Header, Pill-Only Nodes, No Lines.
 */

import { Card } from './ui/card';
import { cn } from '../lib/utils';
import { Network } from 'lucide-react';

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
        {
            name: 'FullHarvestCycle',
            type: 'sequence',
            children: [
                { name: 'NavigateToSugarPoint', type: 'action' },
                { name: 'AnalyzeSugarDistance', type: 'action' },
                {
                    name: 'HeightCheck',
                    type: 'selector',
                    children: [
                        {
                            name: 'ContinueHarvest',
                            type: 'sequence',
                            children: [
                                { name: 'MoveForwardToScoop', type: 'action' },
                                { name: 'ReverseToNavPoint', type: 'action' },
                                { name: 'NavigateToDumpPoint', type: 'action' },
                                { name: 'DumpAction', type: 'action' },
                            ],
                        },
                        { name: 'SwitchToPushMode', type: 'action' },
                    ],
                },
            ],
        },
    ],
};

const STEP_NAMES: Record<string, string> = {
    'NavigateToSugarPoint': '导航到取糖点',
    'AnalyzeSugarDistance': '分析糖堆距离和高度',
    'MoveForwardToScoop': '前进铲糖',
    'ReverseToNavPoint': '原路倒退',
    'NavigateToDumpPoint': '导航到卸载点',
    'DumpAction': '翻斗卸载',
    'SwitchToPushMode': '切换到推垛模式',
    'SugarHarvestMainLoop': '铲糖主循环',
    'FullHarvestCycle': '完整铲糖流程',
    'HeightCheck': '检查糖堆高度',
    'ContinueHarvest': '继续铲糖',
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
                    'inline-flex items-center gap-2.5 px-4 py-1.5 mb-3 rounded-full text-[11px] font-bold tracking-wider transition-all duration-300 w-auto border-none select-none',
                    isSuccess
                        ? 'bg-[#FD802E] text-white shadow-[0_4px_12px_rgba(253,128,46,0.35)]'
                        : isCurrent
                            ? 'bg-[#FD802E]/10 text-[#FD802E] ring-1 ring-[#FD802E]/40 shadow-[0_0_15px_rgba(253,128,46,0.25)]'
                            : 'bg-white/5 text-slate-500 opacity-60'
                )}
            >
                <span className="opacity-70 text-[9px] font-mono">
                    {node.type === 'sequence' && '→'}
                    {node.type === 'selector' && '?'}
                    {node.type === 'repeat' && '↻'}
                    {node.type === 'action' && '⚡'}
                </span>
                <span className="flex items-center gap-1.5">
                    <span className="text-[13px] filter drop-shadow-sm">{STEP_ICONS[node.name] || '⚡'}</span>
                    <span className="whitespace-nowrap">{STEP_NAMES[node.name] || node.name}</span>
                </span>
                {isCurrent && (
                    <span className="ml-1 w-1.5 h-1.5 rounded-full bg-[#FD802E] animate-pulse" />
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
    const needsPushMode = sugarHeight < heightThreshold;

    return (
        <Card
            data-vibe="bt-panel-v3"
            className={cn('relative p-6 bg-[#1A1A1E] border-[#2a2a2e] shadow-2xl rounded-[10px] flex flex-col h-full overflow-hidden', className)}
        >

            {/* ABSOLUTE TOP CENTERED CAPSULE - FORCE CENTERING WITH LEFT-1/2 */}
            <div
                className="absolute top-[10px] left-1/2 -translate-x-1/2 z-[100] flex items-center justify-center gap-2 bg-[#1c1c1e] px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_20px_rgba(253,128,46,0.3)] whitespace-nowrap"
            >
                <div className={cn('w-2.5 h-2.5 rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]',
                    status === 'running' ? 'bg-[#FD802E]' : 'bg-green-500')} />
                <Network className="w-3.5 h-3.5 text-[#FD802E]" />
                <span className="text-[10px] text-[#FD802E] font-bold tracking-[0.2em] uppercase font-mono">
                    行为树状态 | {status === 'running' ? '运行中' : '空闲'}
                </span>
            </div>

            {/* Scrollable Content */}
            <div className="flex-1 space-y-8 overflow-y-auto no-scrollbar pt-14 px-1">

                {/* Metrics Grid */}
                <div className="grid grid-cols-2 gap-5">
                    <div className="bg-white/5 p-4 rounded-2xl border border-transparent">
                        <div className="text-[9px] font-black text-slate-500 uppercase tracking-[0.2em] mb-1">系统循环</div>
                        <div className="flex items-baseline gap-1.5">
                            <span className="text-2xl font-black text-[#FD802E] tabular-nums tracking-tighter">{cycleCount}</span>
                            <span className="text-xs text-slate-600 font-bold italic">/ {maxCycles}</span>
                        </div>
                    </div>
                    <div className={cn('p-4 rounded-2xl transition-all', needsPushMode ? 'bg-[#FD802E]/10 ring-1 ring-[#FD802E]/30' : 'bg-white/5')}>
                        <div className="text-[9px] font-black text-slate-500 uppercase tracking-[0.2em] mb-1">堆体高度</div>
                        <div className="flex items-baseline gap-1.5">
                            <span className={cn('text-2xl font-black tabular-nums tracking-tighter', needsPushMode ? 'text-[#FD802E]' : 'text-slate-200')}>
                                {(sugarHeight * 100).toFixed(1)}
                            </span>
                            <span className="text-xs text-slate-600 font-bold italic">cm</span>
                        </div>
                    </div>
                </div>

                {/* Tree Visual Area */}
                <div className="space-y-4">
                    <div className="text-[10px] font-black text-slate-600 uppercase tracking-[0.4em] px-1 opacity-80">BEHAVIOR FLOW</div>
                    <div className="max-h-[500px] overflow-y-auto no-scrollbar pb-10">
                        <TreeNode node={SUGAR_HARVEST_TREE} currentNode={currentNode} />
                    </div>
                </div>

                {/* Footnotes / History */}
                {stepHistory.length > 0 && (
                    <div className="absolute bottom-6 left-6 right-6 pt-4 border-t border-white/5 bg-[#1A1A1E]">
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
        </Card>
    );
}
