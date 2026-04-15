import { useEffect } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { Header } from './Header';
import { FixedModule } from './FixedModule';
import { MapMonitor } from '@/components/map/MapMonitor';
import { StatsCards } from '@/components/overview/StatsCards';
import { AlertPanel } from '@/components/overview/AlertPanel';
import { HarvestStats } from '@/components/overview/HarvestStats';
import { WorkingDevices } from '@/components/monitor/WorkingDevices';
import { SensorMonitor } from '@/components/monitor/SensorMonitor';
import { TaskPanel } from '@/components/task/TaskPanel';
import { DispatchPanel } from '@/components/dispatch/DispatchPanel';
import { ZoneManageModal } from '@/components/modals/ZoneManageModal';
import { ReportModal } from '@/components/modals/ReportModal';
import { LayoutDashboard, Monitor, AlertTriangle, BarChart3, Truck, Camera, Zap, MapPin, FileText } from 'lucide-react';

export function DashboardLayout() {
  const updateTime = useSystemStore((s) => s.updateTime);
  const robotConnect = useRobotStore((s) => s.connect);
  const robotDisconnect = useRobotStore((s) => s.disconnect);
  const taskConnect = useTaskStore((s) => s.connect);
  const taskDisconnect = useTaskStore((s) => s.disconnect);
  const refreshTasks = useTaskStore((s) => s.refreshTasks);

  useEffect(() => {
    robotConnect();
    taskConnect();
    refreshTasks();

    const timer = setInterval(updateTime, 1000);
    return () => {
      clearInterval(timer);
      robotDisconnect();
      taskDisconnect();
    };
  }, [robotConnect, robotDisconnect, taskConnect, taskDisconnect, refreshTasks, updateTime]);

  return (
    <div className="w-full h-full flex flex-col overflow-hidden">
      <Header />

      <div className="flex-1 px-5 py-3">
        <div className="grid grid-cols-18 gap-2.5 h-full">
          {/* 左侧模块 */}
          <div className="col-span-5 flex flex-col gap-2.5">
            <FixedModule id="tasks" title="任务管理" icon={<Monitor className="w-4 h-4" />} height={240}>
              <TaskPanel />
            </FixedModule>
            <FixedModule id="working-devices" title="工作中设备" icon={<Truck className="w-4 h-4" />} height={380}>
              <WorkingDevices />
            </FixedModule>
            <FixedModule id="dispatch" title="智能调度" icon={<Zap className="w-4 h-4" />} className="flex-1">
              <DispatchPanel />
            </FixedModule>
          </div>

          {/* 中央区域 */}
          <div className="col-span-8 flex flex-col gap-2.5">
            <div className="flex-1 map-container relative">
              <MapMonitor />
              <div className="absolute top-2 right-2 z-[1000] flex gap-1.5">
                <ZoneManageModal />
                <ReportModal />
              </div>
            </div>
            <FixedModule id="alerts" title="" icon={<AlertTriangle className="w-4 h-4" />} height={200}>
              {/* <AlertPanel /> */}
              <img src="/history.png" alt="alerts" className="w-full h-full" />
            </FixedModule>
          </div>

          {/* 右侧模块 */}
          <div className="col-span-5 flex flex-col gap-2.5">
            <FixedModule id="rgb-sensor" title="实时监控 (RGB)" icon={<Camera className="w-4 h-4" />} height={280}>
              <SensorMonitor type="rgb" />
            </FixedModule>
            <FixedModule id="depth-sensor" title="实时监控 (深度)" icon={<Camera className="w-4 h-4" />} height={280}>
              <SensorMonitor type="depth" />
            </FixedModule>
            
            <FixedModule id="harvest-stats" title="铲糖数据统计" icon={<BarChart3 className="w-4 h-4" />} className="flex-1">
              <HarvestStats />
            </FixedModule>
            <FixedModule id="stats" title="系统概览" icon={<LayoutDashboard className="w-4 h-4" />} height={180}>
              <StatsCards />
            </FixedModule>
          </div>
        </div>
      </div>
    </div>
  );
}
