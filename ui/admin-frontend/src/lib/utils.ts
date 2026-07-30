import { clsx } from 'clsx';
import { twMerge } from 'tailwind-merge';
import type { ClassValue } from 'clsx';

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

export function formatTime(timestamp: string): string {
  const d = new Date(timestamp);
  return d.toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

export function formatDateTime(timestamp: string): string {
  const d = new Date(timestamp);
  return d.toLocaleString('zh-CN', {
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
  });
}

export function formatDuration(seconds: number): string {
  const days = Math.floor(seconds / 86400);
  const hours = Math.floor((seconds % 86400) / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  
  const parts = [];
  if (days > 0) parts.push(`${days}天`);
  if (hours > 0) parts.push(`${hours}时`);
  parts.push(`${minutes}分`);
  
  return parts.join('/');
}

export function getStatusColor(status: string): string {
  const map: Record<string, string> = {
    online: '#52c41a',
    running: '#1677ff',
    idle: '#1677ff',
    pending: '#fa8c16',
    warning: '#fa8c16',
    paused: '#fa8c16',
    error: '#f5222d',
    offline: '#64748b',
    completed: '#52c41a',
    failed: '#f5222d',
    stopped: '#64748b',
    cancelled: '#64748b',
    emergency_stop: '#f5222d',
    moving: '#00d4ff',
    scooping: '#fa8c16',
    dumping: '#52c41a',
    critical: '#f5222d',
    info: '#1677ff',
  };
  return map[status] || '#64748b';
}

export function getStatusLabel(status: string): string {
  const map: Record<string, string> = {
    online: '在线',
    offline: '离线',
    running: '运行中',
    idle: '空闲',
    pending: '待执行',
    paused: '已暂停',
    completed: '已完成',
    failed: '失败',
    stopped: '已停止',
    cancelled: '已取消',
    error: '错误',
    warning: '告警',
    emergency_stop: '紧急停止',
    moving: '移动中',
    scooping: '铲取中',
    dumping: '倾倒中',
    critical: '严重',
    info: '信息',
  };
  return map[status] || status;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}
