/**
 * Robot control API service / 机器人控制 API 服务
 * Provides API calls for robot motion control, servo control, and automatic sugar scooping cycle / 提供机器人运动控制、伺服控制和自动循环铲糖功能的 API 调用
 */

import { API_BASE } from '../config';

// ==================== Type definitions / 类型定义 ====================

/** Robot state enum / 机器人状态枚举 */
export const RobotState = {
    IDLE: 'idle',
    MOVING: 'moving',
    SCOOPING: 'scooping',
    DUMPING: 'dumping',
    ERROR: 'error',
    EMERGENCY_STOP: 'emergency_stop',
} as const;

export type RobotState = (typeof RobotState)[keyof typeof RobotState];

/** Move direction enum / 移动方向枚举 */
export const MoveDirection = {
    FORWARD: 'forward',
    BACKWARD: 'backward',
    LEFT: 'left',
    RIGHT: 'right',
    ROTATE_LEFT: 'rotate_left',
    ROTATE_RIGHT: 'rotate_right',
    STOP: 'stop',
} as const;

export type MoveDirection = (typeof MoveDirection)[keyof typeof MoveDirection];

/** Servo state / 伺服状态 */
export interface ServoState {
    name: string;
    current_angle: number;
    target_angle: number;
    is_moving: boolean;
}

/** Robot status - matches backend RobotStatusResponse / 机器人状态 - 匹配后端 RobotStatusResponse */
export interface RobotStatus {
    connected: boolean;
    mode: string;
    status: string;  // 对应前端的 state
    charge: number;  // 对应前端的 battery_level
    speed: number;   // 对应前端的 track_speed
    fault: string;
    fault_level: string;
    task_id: string;
    station: string;
    map_name: string;
    x: number;
    y: number;
    z: number;
    a: number;  // orientation angle
    boom: number;  // lift angle (°)
    bucket: number;  // bucket angle (°)
    last_update?: string;
}

/** Move request / 移动请求 */
export interface MoveRequest {
    direction: MoveDirection;
    speed?: number;
    duration?: number;
}

/** Servo control request / 伺服控制请求 */
export interface ServoRequest {
    servo_id: string;
    angle: number;
}

/** Distance analysis request / 距离分析请求 */
export interface DistanceAnalysisRequest {
    region_of_interest?: Record<string, unknown>;
}

/** Distance analysis result / 距离分析结果 */
export interface DistanceAnalysisResult {
    distance_m: number;
    sugar_height_m: number;
    point_count: number;
    deepest_point: [number, number, number];
    bucket_width_m: number;
    should_switch_to_push: boolean;
    timestamp?: string;
}

/** Sugar harvest config / 铲糖配置 */
export interface SugarHarvestConfig {
    navigation_point: [number, number];
    dump_point: [number, number];
    bucket_width_m?: number;
    scoop_position?: number;
    dump_position?: number;
    max_cycles?: number;
    height_threshold_m?: number;
}

/** Sugar harvest start request / 铲糖启动请求 */
export interface SugarHarvestStartRequest {
    config: SugarHarvestConfig;
}

/** Sugar harvest status / 铲糖状态 */
export interface SugarHarvestStatus {
    is_running: boolean;
    current_cycle?: number;
    max_cycles?: number;
    sugar_height?: number;
    height_threshold?: number;
    blackboard?: Record<string, unknown>;
}

// ==================== API / 接口 ====================

// Point cloud analysis parameters / 点云分析参数
export interface PointCloudSettings {
    teethHeight: number;    // Z1: 铲齿高度 (m)
    cameraToTeeth: number;  // 相机到铲齿距离 (m)
    bucketDepth: number;    // 铲斗深度 (m)
    bucketVolume: number;   // 目标体积 (L)
}

export const robotApi = {
    // --- Motion control / 运动控制 ---

    /** Move robot / 移动机器人 */
    async move(request: MoveRequest): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/move`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request),
        });
        if (!res.ok) throw new Error('Failed to move robot');
        return res.json();
    },

    /** Emergency stop robot / 紧急停止机器人 */
    async stop(): Promise<{ success: boolean; message: string }> {
        const res = await fetch(`${API_BASE}/robot/stop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to stop robot');
        return res.json();
    },

    /** Pause task (pauseTask) / 暂停任务 (pauseTask) */
    async pause(): Promise<{ success: boolean; message: string }> {
        const res = await fetch(`${API_BASE}/robot/pause`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to pause robot');
        return res.json();
    },

    /** Resume from pause (pauseCancel) / 取消暂停 (pauseCancel) */
    async resume(): Promise<{ success: boolean; message: string }> {
        const res = await fetch(`${API_BASE}/robot/resume`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to resume robot');
        return res.json();
    },

    /** Navigate to pick point (allPick) / 导航到取货点 (allPick) */
    async navToPick(): Promise<{ success: boolean; task_id: string; message: string }> {
        const res = await fetch(`${API_BASE}/robot/nav-pick`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to navigate to pick point');
        return res.json();
    },

    /** Navigate to drop point (allDrop) / 导航到卸货点 (allDrop) */
    async navToDrop(): Promise<{ success: boolean; task_id: string; message: string }> {
        const res = await fetch(`${API_BASE}/robot/nav-drop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to navigate to drop point');
        return res.json();
    },

    /** Reset emergency stop status / 重置紧急停止状态 */
    async reset(): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/reset`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to reset robot');
        return res.json();
    },

    // --- Servo control / 伺服控制 ---

    /** Set servo angle / 设置伺服角度 */
    async setServo(request: ServoRequest): Promise<{
        success: boolean;
        message: string;
        servo_status?: { current_angle: number; is_moving: boolean };
    }> {
        const res = await fetch(`${API_BASE}/robot/servo`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request),
        });
        if (!res.ok) throw new Error('Failed to control servo');
        return res.json();
    },

    /** Execute scoop action / 执行铲取动作 */
    async scoop(): Promise<{ success: boolean; task_id: string; message: string }> {
        const res = await fetch(`${API_BASE}/robot/scoop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to execute scoop action');
        return res.json();
    },

    /** Execute dump action / 执行倾倒动作 */
    async dump(): Promise<{ success: boolean; task_id: string; message: string }> {
        const res = await fetch(`${API_BASE}/robot/dump`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to execute dump action');
        return res.json();
    },
    /** Execute dock action / 执行回桩动作 */
    async dock(): Promise<{ success: boolean; task_id: string; message: string }> {
        const res = await fetch(`${API_BASE}/robot/dock`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to execute dock action');
        return res.json();
    },


    // --- Status query / 状态查询 ---

    /** Get robot status / 获取机器人状态 */
    async getStatus(): Promise<RobotStatus> {
        const res = await fetch(`${API_BASE}/robot/status`);
        if (!res.ok) throw new Error('Failed to get robot status');
        return res.json();
    },

    // --- Distance analysis / 距离分析 ---

    /** Analyze sugar pile distance / 分析糖堆距离 */
    async analyzeDistance(request?: DistanceAnalysisRequest): Promise<DistanceAnalysisResult> {
        const res = await fetch(`${API_BASE}/robot/distance_analyze`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request || {}),
        });
        if (!res.ok) throw new Error('Failed to analyze distance');
        return res.json();
    },

    // --- Auto sugar harvest cycle / 自动铲糖循环 ---

    /** Start auto sugar harvest cycle / 启动自动铲糖循环 */
    async startSugarHarvest(request: SugarHarvestStartRequest): Promise<{
        success: boolean;
        message: string;
        config: SugarHarvestConfig;
    }> {
        const res = await fetch(`${API_BASE}/robot/auto_cycle/start`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request),
        });
        if (!res.ok) throw new Error('Failed to start sugar harvest cycle');
        return res.json();
    },

    /** Stop auto sugar harvest cycle / 停止自动铲糖循环 */
    async stopSugarHarvest(): Promise<{ success: boolean; message: string }> {
        const res = await fetch(`${API_BASE}/robot/auto_cycle/stop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to stop sugar harvest cycle');
        return res.json();
    },

    /** Get auto sugar harvest cycle status / 获取自动铲糖循环状态 */
    async getSugarHarvestStatus(): Promise<SugarHarvestStatus> {
        const res = await fetch(`${API_BASE}/robot/auto_cycle/status`);
        if (!res.ok) throw new Error('Failed to get sugar harvest status');
        return res.json();
    },
};
