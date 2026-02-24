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
} from 'lucide-react';
import { toast } from 'sonner';

interface CommandCenterProps {
  isRunning: boolean;
  onToggleRunning: () => void;
  onReset: () => void;
}

export function CommandCenter({ isRunning, onToggleRunning, onReset }: CommandCenterProps) {
  const handleCommand = (command: string) => {
    toast.success(`命令已执行: ${command}`);
  };

  return (
    <div className="flex items-center gap-3">
      {/* Primary Controls */}
      <Button
        onClick={onToggleRunning}
        size="lg"
        className={`h-12 px-6 ${isRunning ? 'bg-red-600 hover:bg-red-700' : 'bg-orange-500 hover:bg-orange-600'}`}
      >
        {isRunning ? (
          <>
            <Pause className="w-5 h-5 mr-2" />
            暂停
          </>
        ) : (
          <>
            <Play className="w-5 h-5 mr-2" />
            启动
          </>
        )}
      </Button>
      
      <Button onClick={onReset} variant="outline" size="lg" className="h-12 px-6">
        <RotateCcw className="w-5 h-5 mr-2" />
        重置
      </Button>

      {/* Navigation Commands */}
      <Button
        onClick={() => handleCommand('返回原点')}
        variant="outline"
        size="lg"
        className="h-12 px-6"
      >
        <Home className="w-5 h-5 mr-2" />
        原点
      </Button>
      
      <Button
        onClick={() => handleCommand('设置路径点')}
        variant="outline"
        size="lg"
        className="h-12 px-6"
      >
        <MapPin className="w-5 h-5 mr-2" />
        路径点
      </Button>

      {/* Action Commands */}
      <Button
        onClick={() => handleCommand('拍摄图像')}
        variant="outline"
        size="lg"
        className="h-12 px-4"
      >
        <Camera className="w-5 h-5" />
      </Button>
      
      <Button
        onClick={() => handleCommand('启用防护')}
        variant="outline"
        size="lg"
        className="h-12 px-4"
      >
        <Shield className="w-5 h-5" />
      </Button>
      
      <Button
        onClick={() => handleCommand('紧急停止')}
        variant="outline"
        size="lg"
        className="h-12 px-4 border-red-500 text-red-500 hover:bg-red-500 hover:text-white"
      >
        <AlertCircle className="w-5 h-5" />
      </Button>
    </div>
  );
}
