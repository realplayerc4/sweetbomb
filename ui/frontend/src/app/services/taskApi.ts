/**
 * 任务 API 客户端
 * 与后端任务系统通信，支持任务的创建、控制和查询
 */

import { API_BASE } from '../config';

// --- 类型定义 ---

/** 任务状态枚举 */
export type TaskStatus =
    | 'pending'
    | 'running'
    | 'paused'
    | 'completed'
    | 'failed'
    | 'stopped'
    | 'cancelled';

/** 任务优先级 */
export type TaskPriority = 1 | 5 | 10 | 20;

/** 任务配置 */
export interface TaskConfig {
    device_id?: string;
    priority?: TaskPriority;
    max_retries?: number;
    timeout_seconds?: number | null;
    params?: Record<string, unknown>;
}

/** 任务进度信息 */
export interface TaskProgress {
    current_step: number;
    total_steps: number;
    percentage: number;
    message: string;
    elapsed_seconds: number;
    estimated_remaining_seconds?: number | null;
}

/** 任务执行结果 */
export interface TaskResult {
    success: boolean;
    data?: Record<string, unknown>;
    message?: string;
    error?: string | null;
    metrics?: Record<string, unknown>;
}

/** 任务信息 */
export interface TaskInfo {
    task_id: string;
    task_type: string;
    status: TaskStatus;
    priority: TaskPriority;
    device_id?: string | null;
    config: TaskConfig;
    params: Record<string, unknown>;
    created_at: string;
    started_at?: string | null;
    completed_at?: string | null;
    progress: TaskProgress;
    result?: TaskResult | null;
}

/** 任务类型信息 */
export interface TaskTypeInfo {
    task_type: string;
    name: string;
    description: string;
    category: string;
    requires_device: boolean;
    params_schema: Record<string, unknown>;
}

/** 任务列表响应 */
export interface TaskListResponse {
    tasks: TaskInfo[];
    total: number;
    running_count: number;
    pending_count: number;
;
}

/** 任务创建请求 */
export interface TaskCreateRequest {
    task_type: string;
    device_id?: string | null;
    config?: TaskConfig | null;
    params?: Record<string, unknown>;
}

/** 任务事件（Socket.IO 广播） */
export interface TaskEvent {
    event_type: string;
    task_id: string;
    task_type: string;
    status: TaskStatus;
    progress?: TaskProgress | null;
    result?: TaskResult | null;
    timestamp: string;
}

// --- 航点类型 ---

/** 航点 */
export interface Waypoint {
    name: string;
    pos: [number, number, number];
    created_at?: string;
}

// --- API 客户端 ---

/** 任务 API 客户端 */
class TaskApiClient {
    private baseUrl: string;

    constructor() {
        this.baseUrl = `${API_BASE}/tasks`;
    }

    /** 通用请求方法 */
    private async request<T>(
        endpoint: string,
        options?: RequestInit
    ): Promise<T> {
        const url = `${this.baseUrl}${endpoint}`;
        const response = await fetch(url, {
            headers: {
                'Content-Type': 'application/json',
                ...options?.headers,
            },
            ...options,
        });

        if (!response.ok) {
            const error = await response.json().catch(() => ({ detail: 'Unknown error' }));
            throw new Error(error.detail || `HTTP ${response.status}`);
        }

        if (response.status === 204) {
            return undefined as T;
        }

        return response.json();
    }

    // --- 任务类型相关 ---

    /** 获取所有任务类型 */
    async getTaskTypes(): Promise<TaskTypeInfo[]> {
        return this.request<TaskTypeInfo[]>('/types');
    }

    /** 获取指定任务类型信息 */
    async getTaskTypeInfo(taskType: string): Promise<TaskTypeInfo> {
        return this.request<TaskTypeInfo>(`/types/${taskType}`);
    }

    // --- 任务 CRUD 操作 ---

    /** 创建任务 */
    async createTask(request: TaskCreateRequest): Promise<TaskInfo> {
        return this.request<TaskInfo>('/', {
            method: 'POST',
            body: JSON.stringify(request),
        });
    }

    /** 列表任务（支持过滤） */
    async listTasks(filters?: {
        status?: TaskStatus;
        device_id?: string;
        task_type?: string;
    }): Promise<TaskListResponse> {
        const params = new URLSearchParams();
        if (filters?.status) params.append('status', filters.status);
        if (filters?.device_id) params.append('device_id', filters.device_id);
        if (filters?.task_type) params.append('task_type', filters.task_type);

        const query = params.toString();
        return this.request<TaskListResponse>(`/${query ? `?${query}` : ''}`);
    }

    /** 获取任务详情 */
    async getTask(taskId: string): Promise<TaskInfo> {
        return this.request<TaskInfo>(`/${taskId}`);
    }

    /** 删除任务 */
    async deleteTask(taskId: string): Promise<void> {
        return this.request<void>(`/${taskId}`, { method: 'DELETE' });
    }

    // --- 任务控制 ---

    /** 启动任务 */
    async startTask(taskId: string): Promise<TaskInfo> {
        return this.request<TaskInfo>(`/${taskId}/start`, { method: 'POST' });
    }

    /** 暂停任务 */
    async pauseTask(taskId: string): Promise<TaskInfo> {
        return this.request<TaskInfo>(`/${taskId}/pause`, { method: 'POST' });
    }

    /** 恢复任务 */
    async resumeTask(taskId: string): Promise<TaskInfo> {
        return this.request<TaskInfo>(`/${taskId}/resume`, { method: 'POST' });
    }

    /** 停止任务 */
    async stopTask(taskId: string): Promise<TaskInfo> {
        return this.request<TaskInfo>(`/${taskId}/stop`, { method: 'POST' });
    }
}

// --- 航点 API 客户端 ---

/** 航点 API 客户端 */
class WaypointApiClient {
    private baseUrl: string;

    constructor() {
        this.baseUrl = `${API_BASE}/waypoints`;
    }

    /** 通用请求方法 */
    private async request<T>(
        endpoint: string,
        options?: RequestInit
    ): Promise<T> {
        const url = `${this.baseUrl}${endpoint}`;
        const response = await fetch(url, {
            headers: {
                'Content-Type': 'application/json',
                ...options?.headers,
            },
            ...options,
        });

        if (!response.ok) {
            const error = await response.json().catch(() => ({ detail: 'Unknown error' }));
            throw new Error(error.detail || `HTTP ${response.status}`);
        }

        if (response.status === 204) {
            return undefined as T;
        }

        return response.json();
    }

    /** 获取所有航点 */
    async listWaypoints(): Promise<Waypoint[]> {
        return this.request<Waypoint[]>('/');
    }

    /** 创建航点 */
    async createWaypoint(waypoint: Waypoint): Promise<Waypoint> {
        return this.request<Waypoint>('/', {
            method: 'POST',
            body: JSON.stringify(waypoint),
        });
    }

    /** 删除航点 */
    async deleteWaypoint(name: string): Promise<void> {
        return this.request<void>(`/${name}`, { method: 'DELETE' });
    }
}

// --- 导出单例实例 ---

export const taskApi = new TaskApiClient();
export const waypointApi = new WaypointApiClient();
