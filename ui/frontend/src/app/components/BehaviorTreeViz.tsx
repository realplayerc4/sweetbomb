/**
 * Behavior Tree Visualization
 * Real-time visualization of the behavior tree execution with current node highlighting.
 */

import { Card } from './ui/card';
import { cn } from '../lib/utils';

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

// Sugar harvest behavior tree structure
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

// Step name mapping for display
const STEP_NAMES: Record<string, string> = {
    'NavigateToSugarPoint': '📍 导航到取糖点',
    'AnalyzeSugarDistance': '📏 分析糖堆距离和高度',
    'MoveForwardToScoop': '⬆️ 前进铲糖',
    'ReverseToNavPoint': '⬇️ 原路倒退',
    'NavigateToDumpPoint': '📍 导航到卸载点',
    'DumpAction': '🗑️ 翻斗卸载',
    'SwitchToPushMode': '🔄 切换到推垛模式',
    'SugarHarvestMainLoop': '🔄 铲糖主循环',
    'FullHarvestCycle': '⚙️ 完整铲糖流程',
    'HeightCheck': '📏 检查糖堆高度',
    'ContinueHarvest': '✅ 继续铲糖',
};

function getStepDisplayName(nodeName: string): string {
    return STEP_NAMES[nodeName] || nodeName;
}

function getNodeStatusColor(status: string | undefined): string {
    switch (status) {
        case 'running':
            return 'text-blue-400 bg-blue-900/30 border-blue-500/50';
        case 'success':
            return 'text-green-400 bg-green-900/30 border-green-500/50';
        case 'failure':
            return 'text-red-400 bg-red-900/30 border-red-500/50';
        default:
            return 'text-slate-400 bg-slate-800/50 border-slate-700/50';
    }
}

function TreeNode({ node, currentNode, depth = 0 }: { node: BTNodeConfig; currentNode: string; depth?: number }) {
    const isCurrent = node.name === currentNode;
    const nodeColor = isCurrent ? 'text-primary font-bold' : 'text-slate-400';
    const borderColor = isCurrent ? 'border-primary' : 'border-slate-700';

    return (
        <div className={cn('flex flex-col', depth > 0 && 'ml-4')}>
            <div
                className={cn(
                    'flex items-center gap-2 px-2 py-1 rounded border text-sm',
                    borderColor,
                    getNodeStatusColor(isCurrent ? 'running' : node.status)
                )}
            >
                {/* Node type indicator */}
                <span className="text-xs opacity-50">
                    {node.type === 'sequence' && '→'}
                    {node.type === 'selector' && '?'}
                    {node.type === 'repeat' && '↻'}
                    {node.type === 'action' && '⚡'}
                    {node.type === 'condition' && '?='}
                </span>

                {/* Node name */}
                <span className={nodeColor}>{getStepDisplayName(node.name)}</span>

                {/* Current indicator */}
                {isCurrent && (
                    <span className="ml-auto w-2 h-2 rounded-full bg-primary animate-pulse" />
                )}
            </div>

            {/* Children */}
            {node.children && node.children.length > 0 && (
                <div className="flex flex-col gap-1 mt-1">
                    {node.children.map((child, idx) => (
                        <TreeNode
                            key={`${child.name}-${idx}`}
                            node={child}
                            currentNode={currentNode}
                            depth={depth + 1}
                        />
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
    const currentStepName = getStepDisplayName(currentNode);

    return (
        <Card className={cn('p-4 space-y-4', className)}>
            {/* Header */}
            <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">行为树状态</h3>
                <div className={cn(
                    'px-2 py-1 rounded text-xs font-medium',
                    status === 'running' && 'text-blue-400 bg-blue-900/30',
                    status === 'success' && 'text-green-400 bg-green-900/30',
                    status === 'failure' && 'text-red-400 bg-red-900/30',
                    status === 'idle' && 'text-slate-400 bg-slate-800/50',
                )}>
                    {status === 'running' && '运行中'}
                    {status === 'success' && '已完成'}
                    {status === 'failure' && '失败'}
                    {status === 'idle' && '空闲'}
                </div>
            </div>

            {/* Current Step */}
            {currentNode && (
                <div className="bg-slate-800 rounded-lg p-4 text-center">
                    <div className="text-sm text-slate-400 mb-1">当前步骤</div>
                    <div className="text-xl font-bold text-primary">{currentStepName}</div>
                </div>
            )}

            {/* Cycle Progress */}
            <div className="bg-slate-800 rounded-lg p-3 space-y-2">
                <div className="flex items-center justify-between text-sm">
                    <span className="text-slate-400">循环进度</span>
                    <span className="font-medium">{cycleCount} / {maxCycles}</span>
                </div>
                <div className="h-2 bg-slate-700 rounded-full overflow-hidden">
                    <div
                        className="h-full bg-primary transition-all duration-300"
                        style={{ width: `${(cycleCount / maxCycles) * 100}%` }}
                    />
                </div>
            </div>

            {/* Sugar Height Indicator */}
            <div className={cn(
                'rounded-lg p-3',
                needsPushMode ? 'bg-yellow-900/30' : 'bg-green-900/30'
            )}>
                <div className="flex items-center justify-between text-sm">
                    <span className={cn(
                        'font-medium',
                        needsPushMode ? 'text-yellow-400' : 'text-green-400'
                    )}>
                        糖堆高度
                    </span>
                    <span className={cn(
                        'font-bold text-lg',
                        needsPushMode ? 'text-yellow-400' : 'text-green-400'
                    )}>
                        {(sugarHeight * 100).toFixed(1)} cm
                    </span>
                </div>
                {needsPushMode && (
                    <div className="text-xs text-yellow-400 mt-1 text-center">
                        ⚠️ 即将切换到推垛模式
                    </div>
                )}
            </div>

            {/* Behavior Tree Visualization */}
            <div className="space-y-1">
                <div className="text-xs text-slate-400 mb-2">行为树结构</div>
                <div className="max-h-64 overflow-y-auto custom-scrollbar">
                    <TreeNode node={SUGAR_HARVEST_TREE} currentNode={currentNode} />
                </div>
            </div>

            {/* Step History */}
            {stepHistory.length > 0 && (
                <div className="space-y-2 pt-2 border-t border-slate-700">
                    <div className="text-xs text-slate-400">最近步骤</div>
                    <div className="flex flex-wrap gap-1">
                        {stepHistory.slice(-5).reverse().map((step, idx) => (
                            <span
                                key={idx}
                                className="px-2 py-1 rounded bg-slate-800 text-xs text-slate-300"
                            >
                                {getStepDisplayName(step)}
                            </span>
                        ))}
                    </div>
                </div>
            )}
        </Card>
    );
}
