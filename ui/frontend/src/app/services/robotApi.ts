/**
 * Robot API service
 */

import { API_BASE } from '../config';

// ==================== Types ====================

export const RobotState = {
    IDLE: 'idle',
    MOVING: 'moving',
    SCOOPING: 'scooping',
    DUMPING: 'dumping',
    ERROR: 'error',
    EMERGENCY_STOP: 'emergency_stop',
} as const;

export type RobotState = (typeof RobotState)[keyof typeof RobotState];

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

export interface ServoState {
    name: string;
    current_angle: number;
    target_angle: number;
    is_moving: boolean;
}

export interface RobotStatus {
    state: RobotState;
    battery_level: number;
    current_position: [number, number, number];
    orientation: [number, number, number]; // [Roll, Pitch, Yaw]
    left_track_speed: number;
    right_track_speed: number;
    servos: Record<string, ServoState>;
    timestamp: string;
}

export interface MoveRequest {
    direction: MoveDirection;
    speed?: number;
    duration?: number;
}

export interface ServoRequest {
    servo_id: string;
    angle: number;
}

export interface DistanceAnalysisRequest {
    region_of_interest?: Record<string, unknown>;
}

export interface DistanceAnalysisResult {
    distance_m: number;
    sugar_height_m: number;
    point_count: number;
    deepest_point: [number, number, number];
    bucket_width_m: number;
    should_switch_to_push: boolean;
    timestamp?: string;
}

export interface SugarHarvestConfig {
    navigation_point: [number, number];
    dump_point: [number, number];
    bucket_width_m?: number;
    approach_offset_m?: number;
    scoop_position?: number;
    dump_position?: number;
    max_cycles?: number;
    height_threshold_m?: number;
}

export interface SugarHarvestStartRequest {
    config: SugarHarvestConfig;
}

export interface SugarHarvestStatus {
    is_running: boolean;
    current_cycle?: number;
    max_cycles?: number;
    sugar_height?: number;
    height_threshold?: number;
    blackboard?: Record<string, unknown>;
}

// ==================== API ====================

export const robotApi = {
    // --- Movement Control ---

    async move(request: MoveRequest): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/move`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request),
        });
        if (!res.ok) throw new Error('Failed to move robot');
        return res.json();
    },

    async stop(): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/stop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to stop robot');
        return res.json();
    },

    async reset(): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/reset`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to reset robot');
        return res.json();
    },

    // --- Servo Control ---

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

    async scoop(): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/scoop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to execute scoop action');
        return res.json();
    },

    async dump(): Promise<{ success: boolean; message: string; status: RobotState }> {
        const res = await fetch(`${API_BASE}/robot/dump`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to execute dump action');
        return res.json();
    },

    // --- Status ---

    async getStatus(): Promise<RobotStatus> {
        const res = await fetch(`${API_BASE}/robot/status`);
        if (!res.ok) throw new Error('Failed to get robot status');
        return res.json();
    },

    // --- Distance Analysis ---

    async analyzeDistance(request?: DistanceAnalysisRequest): Promise<DistanceAnalysisResult> {
        const res = await fetch(`${API_BASE}/robot/distance_analyze`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request || {}),
        });
        if (!res.ok) throw new Error('Failed to analyze distance');
        return res.json();
    },

    // --- Auto Cycle ---

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

    async stopSugarHarvest(): Promise<{ success: boolean; message: string }> {
        const res = await fetch(`${API_BASE}/robot/auto_cycle/stop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to stop sugar harvest cycle');
        return res.json();
    },

    async getSugarHarvestStatus(): Promise<SugarHarvestStatus> {
        const res = await fetch(`${API_BASE}/robot/auto_cycle/status`);
        if (!res.ok) throw new Error('Failed to get sugar harvest status');
        return res.json();
    },
};
