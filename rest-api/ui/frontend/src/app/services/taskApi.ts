/**
 * Task API client for communicating with the backend task system.
 */

import { API_BASE } from '../config';

// --- Types ---

export type TaskStatus =
  | 'pending'
  | 'running'
  | 'paused'
  | 'completed'
  | 'failed'
  | 'stopped'
  | 'cancelled';

export type TaskPriority = 1 | 5 | 10 | 20;

export interface TaskConfig {
  device_id?: string;
  priority?: TaskPriority;
  max_retries?: number;
  timeout_seconds?: number | null;
  params?: Record<string, unknown>;
}

export interface TaskProgress {
  current_step: number;
  total_steps: number;
  percentage: number;
  message: string;
  elapsed_seconds: number;
  estimated_remaining_seconds?: number | null;
}

export interface TaskResult {
  success: boolean;
  data?: Record<string, unknown>;
  message?: string;
  error?: string | null;
  metrics?: Record<string, unknown>;
}

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

export interface TaskTypeInfo {
  task_type: string;
  name: string;
  description: string;
  category: string;
  requires_device: boolean;
  params_schema: Record<string, unknown>;
}

export interface TaskListResponse {
  tasks: TaskInfo[];
  total: number;
  running_count: number;
  pending_count: number;
}

export interface TaskCreateRequest {
  task_type: string;
  device_id?: string | null;
  config?: TaskConfig | null;
  params?: Record<string, unknown>;
}

export interface TaskEvent {
  event_type: string;
  task_id: string;
  task_type: string;
  status: TaskStatus;
  progress?: TaskProgress | null;
  result?: TaskResult | null;
  timestamp: string;
}

// --- API Client ---

class TaskApiClient {
  private baseUrl: string;

  constructor() {
    this.baseUrl = `${API_BASE}/tasks`;
  }

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

  // --- Task Types ---

  async getTaskTypes(): Promise<TaskTypeInfo[]> {
    return this.request<TaskTypeInfo[]>('/types');
  }

  async getTaskTypeInfo(taskType: string): Promise<TaskTypeInfo> {
    return this.request<TaskTypeInfo>(`/types/${taskType}`);
  }

  // --- Task CRUD ---

  async createTask(request: TaskCreateRequest): Promise<TaskInfo> {
    return this.request<TaskInfo>('/', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

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

  async getTask(taskId: string): Promise<TaskInfo> {
    return this.request<TaskInfo>(`/${taskId}`);
  }

  async deleteTask(taskId: string): Promise<void> {
    return this.request<void>(`/${taskId}`, { method: 'DELETE' });
  }

  // --- Task Control ---

  async startTask(taskId: string): Promise<TaskInfo> {
    return this.request<TaskInfo>(`/${taskId}/start`, { method: 'POST' });
  }

  async pauseTask(taskId: string): Promise<TaskInfo> {
    return this.request<TaskInfo>(`/${taskId}/pause`, { method: 'POST' });
  }

  async resumeTask(taskId: string): Promise<TaskInfo> {
    return this.request<TaskInfo>(`/${taskId}/resume`, { method: 'POST' });
  }

  async stopTask(taskId: string): Promise<TaskInfo> {
    return this.request<TaskInfo>(`/${taskId}/stop`, { method: 'POST' });
  }
}

// Export singleton instance
export const taskApi = new TaskApiClient();
