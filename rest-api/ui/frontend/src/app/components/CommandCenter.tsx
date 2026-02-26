import { useState } from 'react';
import { Button } from './ui/button';
import {
  Play,
  Pause,
  RotateCcw,
  Home,
  MapPin,
  Camera,
  Shield,
  AlertCircle,
  Loader2,
} from 'lucide-react';
import { toast } from 'sonner';
import { useTaskManager } from '../hooks/useTaskManager';

interface CommandCenterProps {
  isRunning: boolean;
  onToggleRunning: () => void;
  onReset: () => void;
  deviceId?: string | null;
}

export function CommandCenter({
  isRunning,
  onToggleRunning,
  onReset,
  deviceId,
}: CommandCenterProps) {
  const { createTask, startTask, stopTask, tasks, addWaypoint, canStartMore } =
    useTaskManager({ deviceId });
  const [isProcessing, setIsProcessing] = useState(false);

  const handleCommand = async (command: string) => {
    setIsProcessing(true);
    try {
      if (command === '返回原点') {
        const task = await createTask({
          task_type: 'return_to_origin',
          device_id: deviceId,
        });
        await startTask(task.task_id);
        toast.success('已启动“返回原点”任务');
      } else if (command === '设置路径点') {
        const name = window.prompt('请输入路径点名称:', `WP_${new Date().getTime()}`);
        if (name) {
          await addWaypoint({
            name,
            pos: [0, 0, 0], // In real case, this would be current robot pos
            created_at: new Date().toISOString(),
          });
          toast.success(`路径点 “${name}” 已保存`);
        }
      } else if (command === '紧急停止') {
        const runningTasks = tasks.filter((t) => t.status === 'running');
        await Promise.all(runningTasks.map((t) => stopTask(t.task_id)));
        toast.error('已紧急停止所有运行中任务');
      } else {
        toast.info(`模拟执行命令: ${command}`);
      }
    } catch (e: any) {
      toast.error(`执行失败: ${e.message}`);
    } finally {
      setIsProcessing(false);
    }
  };

  return (
    <div className="flex items-center gap-[3px]">
      {/* Primary Controls */}
      <Button
        onClick={onToggleRunning}
        size="lg"
        className="h-12 px-6 bg-[#FD802E] hover:bg-orange-600 text-white border-none shadow-[0_0_15px_rgba(253,128,46,0.2)]"
        disabled={isProcessing}
      >
        {isRunning ? (
          <>
            <Pause className="w-5 h-5 mr-2" />
            暂停视频
          </>
        ) : (
          <>
            <Play className="w-5 h-5 mr-2" />
            启动视频
          </>
        )}
      </Button>

      <Button
        onClick={onReset}
        size="lg"
        className="h-12 px-6 bg-[#FD802E] hover:bg-orange-600 text-white border-none shadow-[0_0_15px_rgba(253,128,46,0.2)]"
        disabled={isProcessing}
      >
        <RotateCcw className="w-5 h-5 mr-2" />
        重置
      </Button>

      {/* Navigation Commands */}
      <Button
        onClick={() => handleCommand('返回原点')}
        size="lg"
        className="h-12 px-6 bg-[#FD802E] hover:bg-orange-600 text-white border-none shadow-[0_0_15px_rgba(253,128,46,0.2)]"
        disabled={isProcessing || !canStartMore}
      >
        {isProcessing ? (
          <Loader2 className="w-5 h-5 animate-spin mr-2" />
        ) : (
          <Home className="w-5 h-5 mr-2" />
        )}
        原点
      </Button>

      <Button
        onClick={() => handleCommand('设置路径点')}
        size="lg"
        className="h-12 px-6 bg-[#FD802E] hover:bg-orange-600 text-white border-none shadow-[0_0_15px_rgba(253,128,46,0.2)]"
        disabled={isProcessing}
      >
        <MapPin className="w-5 h-5 mr-2" />
        路径点
      </Button>

      {/* Action Commands */}
      <Button
        onClick={() => handleCommand('拍摄图像')}
        className="h-12 px-4 bg-[#FD802E] hover:bg-orange-600 text-white border-none shadow-[0_0_15px_rgba(253,128,46,0.2)]"
        size="lg"
        disabled={isProcessing}
      >
        <Camera className="w-5 h-5" />
      </Button>

      <Button
        onClick={() => handleCommand('启用防护')}
        className="h-12 px-4 bg-[#FD802E] hover:bg-orange-600 text-white border-none shadow-[0_0_15px_rgba(253,128,46,0.2)]"
        size="lg"
        disabled={isProcessing}
      >
        <Shield className="w-5 h-5" />
      </Button>

      <Button
        onClick={() => handleCommand('紧急停止')}
        className="h-12 px-4 bg-[#FF4500] hover:bg-[#E63E00] text-white border-none shadow-[0_0_15px_rgba(255,69,0,0.2)]"
        size="lg"
        disabled={isProcessing}
      >
        <AlertCircle className="w-5 h-5" />
      </Button>
    </div>
  );
}
