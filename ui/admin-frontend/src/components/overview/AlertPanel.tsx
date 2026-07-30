import { useRef, useState, useEffect } from 'react';
import { X, ChevronLeft, ChevronRight } from 'lucide-react';
import { useSystemStore } from '@/stores/useSystemStore';
import { getStatusColor, formatTime, getStatusLabel, cn } from '@/lib/utils';

export function AlertPanel() {
  const alerts = useSystemStore((s) => s.alerts);
  const acknowledgeAlert = useSystemStore((s) => s.acknowledgeAlert);
  const scrollRef = useRef<HTMLDivElement>(null);
  const [canScrollLeft, setCanScrollLeft] = useState(false);
  const [canScrollRight, setCanScrollRight] = useState(false);

  const unacknowledged = alerts.filter((a) => !a.acknowledged);

  const checkScroll = () => {
    const el = scrollRef.current;
    if (!el) return;
    setCanScrollLeft(el.scrollLeft > 4);
    setCanScrollRight(el.scrollLeft < el.scrollWidth - el.clientWidth - 4);
  };

  useEffect(() => {
    checkScroll();
    const el = scrollRef.current;
    if (el) el.addEventListener('scroll', checkScroll);
    return () => { if (el) el.removeEventListener('scroll', checkScroll); };
  }, [alerts]);

  const scroll = (dir: 'left' | 'right') => {
    const el = scrollRef.current;
    if (!el) return;
    el.scrollBy({ left: dir === 'left' ? -200 : 200, behavior: 'smooth' });
  };

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-2">
          
        </div>
        <div className="flex items-center gap-2">
          {unacknowledged.length > 0 && (
            <span className="text-[12px] px-2 py-0.5 rounded-full bg-tech-red/20 text-tech-red font-bold">
              {unacknowledged.length} 未处理
            </span>
          )}
          <div className="flex items-center gap-0.5">
            <button
              onClick={() => scroll('left')}
              className={cn('p-1 rounded transition-colors', canScrollLeft ? 'text-cyan-400 hover:bg-white/10' : 'text-text-muted/30 cursor-default')}
              disabled={!canScrollLeft}
            >
              <ChevronLeft className="w-4 h-4" />
            </button>
            <button
              onClick={() => scroll('right')}
              className={cn('p-1 rounded transition-colors', canScrollRight ? 'text-cyan-400 hover:bg-white/10' : 'text-text-muted/30 cursor-default')}
              disabled={!canScrollRight}
            >
              <ChevronRight className="w-4 h-4" />
            </button>
          </div>
        </div>
      </div>

      {alerts.length === 0 ? (
        <div className="flex-1 flex items-center justify-center">
          <span className="text-[14px] text-text-muted">暂无告警</span>
        </div>
      ) : (
        <div
          ref={scrollRef}
          className="flex-1 flex gap-2.5 overflow-x-auto overflow-y-hidden scroll-smooth snap-x snap-mandatory"
          style={{ scrollbarWidth: 'none', msOverflowStyle: 'none' }}
        >
          {alerts.map((alert) => (
            <div
              key={alert.id}
              className={cn(
                'flex-shrink-0 w-[200px] snap-start flex flex-col gap-1.5 p-3 rounded-md border transition-all',
                alert.acknowledged
                  ? 'bg-transparent border-cyan-500/10 opacity-50'
                  : 'bg-bg-primary/40 border-cyan-500/20'
              )}
            >
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-1.5">
                  <div
                    className="status-dot shrink-0"
                    style={{
                      background: getStatusColor(alert.level),
                      boxShadow: `0 0 6px ${getStatusColor(alert.level)}60`,
                    }}
                  />
                  <span className="text-[12px] font-bold" style={{ color: getStatusColor(alert.level) }}>
                    {getStatusLabel(alert.level)}
                  </span>
                </div>
                {!alert.acknowledged && (
                  <button
                    onClick={() => acknowledgeAlert(alert.id)}
                    className="p-0.5 rounded hover:bg-white/10 text-text-muted hover:text-text-secondary transition-colors"
                  >
                    <X className="w-3.5 h-3.5" />
                  </button>
                )}
              </div>
              <span className="text-[12px] text-text-muted font-mono">{alert.source}</span>
              <p className="text-[13px] text-text-secondary leading-snug line-clamp-2">{alert.message}</p>
              <span className="text-[12px] text-text-muted font-mono mt-auto">{formatTime(alert.timestamp)}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
