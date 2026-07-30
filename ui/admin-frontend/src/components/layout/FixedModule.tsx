import { cn } from '@/lib/utils';

interface FixedModuleProps {
  id: string;
  title: string;
  icon?: React.ReactNode;
  height?: number;
  className?: string;
  children: React.ReactNode;
}

export function FixedModule({ id, title, icon, height, className, children }: FixedModuleProps) {
  return (
    <div
      className={cn(
        'fixed-module relative overflow-hidden',
        className
      )}
      style={{
        height: height || 'auto',
      }}
    >
      <div className="h-full flex flex-col">
        <div className="module-header px-4 py-2.5 flex items-center gap-2">
          {icon && <span className="text-cyan-400">{icon}</span>}
          <h3 className="text-[14px] font-semibold text-cyan-400 tracking-[0.15em] uppercase">{title}</h3>
          <div className="flex-1" />
          <div className="h-px flex-1 max-w-[60px] bg-gradient-to-r from-cyan-500/40 to-transparent" />
        </div>
        <div className="flex-1 px-4 pb-4 overflow-auto">
          {children}
        </div>
      </div>
    </div>
  );
}
