import { useState } from 'react';
import { Button, message } from 'antd';
import { TechModal } from './TechModal';
import { useTaskStore } from '@/stores/useTaskStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { robotApi } from '@/services/robotApi';
import { Hand, AlertTriangle, Play, Pause, Square, Navigation, Package } from 'lucide-react';
import { cn } from '@/lib/utils';

interface ManualInterveneModalProps {
  open: boolean;
  onClose: () => void;
}

export function ManualInterveneModal({ open, onClose }: ManualInterveneModalProps) {
  const [loading, setLoading] = useState(false);

  const phase = useTaskStore((s) => s.phase);
  const isRunning = useTaskStore((s) => s.isRunning);
  const start = useTaskStore((s) => s.start);
  const pause = useTaskStore((s) => s.pause);
  const stop = useTaskStore((s) => s.stop);

  const isConnected = useRobotStore((s) => s.isConnected);
  const robotStatus = useRobotStore((s) => s.status);

  const handleManualNavPick = async () => {
    setLoading(true);
    try {
      await robotApi.navToPick();
      message.success('已发送导航取货指令');
    } catch {
      message.error('发送失败');
    } finally {
      setLoading(false);
    }
  };

  const handleManualNavDrop = async () => {
    setLoading(true);
    try {
      await robotApi.navToDrop();
      message.success('已发送导航卸货指令');
    } catch {
      message.error('发送失败');
    } finally {
      setLoading(false);
    }
  };

  const handleManualScoop = async () => {
    setLoading(true);
    try {
      await robotApi.scoop();
      message.success('已发送铲取指令');
    } catch {
      message.error('发送失败');
    } finally {
      setLoading(false);
    }
  };

  const handleManualDump = async () => {
    setLoading(true);
    try {
      await robotApi.dump();
      message.success('已发送倾倒指令');
    } catch {
      message.error('发送失败');
    } finally {
      setLoading(false);
    }
  };

  const handleManualPause = async () => {
    setLoading(true);
    try {
      await robotApi.pause();
      message.success('已发送暂停指令');
    } catch {
      message.error('发送失败');
    } finally {
      setLoading(false);
    }
  };

  const handleManualStop = async () => {
    setLoading(true);
    try {
      await robotApi.stop();
      message.success('已发送停止指令');
    } catch {
      message.error('发送失败');
    } finally {
      setLoading(false);
    }
  };

  return (
    <TechModal
      title="手动控制"
      open={open}
      onCancel={onClose}
      footer={null}
      width={480}
    >
      <div className="space-y-4 py-2">
        <div className="p-3 rounded-md border border-tech-orange/20 bg-tech-orange/5">
          <div className="flex items-center gap-1.5 mb-1">
            <AlertTriangle className="w-3.5 h-3.5 text-tech-orange" />
            <span className="text-[12px] font-bold text-tech-orange">手动干预模式</span>
          </div>
          <span className="text-[11px] text-text-secondary">手动控制机器人执行单个动作，不依赖任务循环。</span>
        </div>

        <div className="tech-card p-3">
          <div className="flex items-center justify-between mb-2">
            <span className="text-[12px] text-text-muted">机器人状态</span>
            <span className={cn(
              'text-[12px] font-bold',
              isConnected ? 'text-tech-green' : 'text-tech-red'
            )}>
              {isConnected ? '已连接' : '未连接'}
            </span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-[12px] text-text-muted">当前状态</span>
            <span className="text-[12px] text-cyan-400 font-bold">{robotStatus}</span>
          </div>
        </div>

        <div>
          <div className="text-[12px] text-text-muted font-semibold mb-2">导航指令</div>
          <div className="grid grid-cols-2 gap-2">
            <Button
              onClick={handleManualNavPick}
              disabled={!isConnected || loading}
              className="h-9 flex items-center gap-2"
            >
              <Navigation className="w-3.5 h-3.5" />
              导航取货点
            </Button>
            <Button
              onClick={handleManualNavDrop}
              disabled={!isConnected || loading}
              className="h-9 flex items-center gap-2"
            >
              <Navigation className="w-3.5 h-3.5" />
              导航卸货点
            </Button>
          </div>
        </div>

        <div>
          <div className="text-[12px] text-text-muted font-semibold mb-2">动作指令</div>
          <div className="grid grid-cols-2 gap-2">
            <Button
              onClick={handleManualScoop}
              disabled={!isConnected || loading}
              className="h-9 flex items-center gap-2"
            >
              <Package className="w-3.5 h-3.5" />
              铲取
            </Button>
            <Button
              onClick={handleManualDump}
              disabled={!isConnected || loading}
              className="h-9 flex items-center gap-2"
            >
              <Package className="w-3.5 h-3.5" />
              倾倒
            </Button>
          </div>
        </div>

        <div>
          <div className="text-[12px] text-text-muted font-semibold mb-2">控制指令</div>
          <div className="grid grid-cols-2 gap-2">
            <Button
              onClick={handleManualPause}
              disabled={!isConnected || loading}
              className="h-9 flex items-center gap-2"
            >
              <Pause className="w-3.5 h-3.5" />
              暂停
            </Button>
            <Button
              onClick={handleManualStop}
              disabled={!isConnected || loading}
              danger
              className="h-9 flex items-center gap-2"
            >
              <Square className="w-3.5 h-3.5" />
              停止
            </Button>
          </div>
        </div>
      </div>
    </TechModal>
  );
}