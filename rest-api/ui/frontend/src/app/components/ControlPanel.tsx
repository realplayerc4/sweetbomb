import { Slider } from './ui/slider';
import { Switch } from './ui/switch';
import { Label } from './ui/label';
import { Zap, Volume2, Gauge } from 'lucide-react';

interface ControlPanelProps {
  power: number;
  speed: number;
  volume: number;
  onPowerChange: (value: number[]) => void;
  onSpeedChange: (value: number[]) => void;
  onVolumeChange: (value: number[]) => void;
  sensorsEnabled: boolean;
  onSensorsToggle: (checked: boolean) => void;
}

export function ControlPanel({
  power,
  speed,
  volume,
  onPowerChange,
  onSpeedChange,
  onVolumeChange,
  sensorsEnabled,
  onSensorsToggle,
}: ControlPanelProps) {
  return (
    <div className="space-y-6">
      {/* Power Control */}
      <div className="space-y-3">
        <div className="flex items-center justify-between">
          <Label className="flex items-center gap-2 text-sm">
            <Zap className="w-4 h-4 text-yellow-500" />
            Power Level
          </Label>
          <span className="text-sm font-mono text-blue-400">{power}%</span>
        </div>
        <Slider
          value={[power]}
          onValueChange={onPowerChange}
          max={100}
          step={1}
          className="w-full"
        />
      </div>

      {/* Speed Control */}
      <div className="space-y-3">
        <div className="flex items-center justify-between">
          <Label className="flex items-center gap-2 text-sm">
            <Gauge className="w-4 h-4 text-green-500" />
            Movement Speed
          </Label>
          <span className="text-sm font-mono text-blue-400">{speed}%</span>
        </div>
        <Slider
          value={[speed]}
          onValueChange={onSpeedChange}
          max={100}
          step={1}
          className="w-full"
        />
      </div>

      {/* Volume Control */}
      <div className="space-y-3">
        <div className="flex items-center justify-between">
          <Label className="flex items-center gap-2 text-sm">
            <Volume2 className="w-4 h-4 text-purple-500" />
            Audio Volume
          </Label>
          <span className="text-sm font-mono text-blue-400">{volume}%</span>
        </div>
        <Slider
          value={[volume]}
          onValueChange={onVolumeChange}
          max={100}
          step={1}
          className="w-full"
        />
      </div>

      {/* Sensors Toggle */}
      <div className="flex items-center justify-between pt-4 border-t border-slate-700">
        <Label className="text-sm">Environmental Sensors</Label>
        <Switch checked={sensorsEnabled} onCheckedChange={onSensorsToggle} />
      </div>
    </div>
  );
}
