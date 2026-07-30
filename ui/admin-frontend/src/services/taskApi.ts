import apiClient from './apiClient';
import type {
  TaskInfo,
  TaskTypeInfo,
  TaskCreateRequest,
  TaskStatus,
  Waypoint,
} from '../types';

interface TaskListResponse {
  tasks: TaskInfo[];
  total: number;
  running_count: number;
  pending_count: number;
}

export const taskApi = {
  async getTaskTypes(): Promise<TaskTypeInfo[]> {
    const res = await apiClient.get<TaskTypeInfo[]>('/tasks/types');
    return res.data;
  },

  async listTasks(filters?: { status?: TaskStatus; device_id?: string }): Promise<TaskListResponse> {
    const res = await apiClient.get<TaskListResponse>('/tasks/', { params: filters });
    return res.data;
  },

  async createTask(request: TaskCreateRequest): Promise<TaskInfo> {
    const res = await apiClient.post<TaskInfo>('/tasks/', request);
    return res.data;
  },

  async startTask(taskId: string): Promise<TaskInfo> {
    const res = await apiClient.post<TaskInfo>(`/tasks/${taskId}/start`);
    return res.data;
  },

  async pauseTask(taskId: string): Promise<TaskInfo> {
    const res = await apiClient.post<TaskInfo>(`/tasks/${taskId}/pause`);
    return res.data;
  },

  async resumeTask(taskId: string): Promise<TaskInfo> {
    const res = await apiClient.post<TaskInfo>(`/tasks/${taskId}/resume`);
    return res.data;
  },

  async stopTask(taskId: string): Promise<TaskInfo> {
    const res = await apiClient.post<TaskInfo>(`/tasks/${taskId}/stop`);
    return res.data;
  },

  async deleteTask(taskId: string): Promise<void> {
    await apiClient.delete(`/tasks/${taskId}`);
  },
};

export const waypointApi = {
  async listWaypoints(): Promise<Waypoint[]> {
    const res = await apiClient.get<Waypoint[]>('/waypoints/');
    return res.data;
  },

  async createWaypoint(waypoint: Waypoint): Promise<Waypoint> {
    const res = await apiClient.post<Waypoint>('/waypoints/', waypoint);
    return res.data;
  },

  async deleteWaypoint(name: string): Promise<void> {
    await apiClient.delete(`/waypoints/${name}`);
  },
};
