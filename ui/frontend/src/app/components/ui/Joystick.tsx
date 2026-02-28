/**
 * Joystick component for robot control.
 * Supports both mouse/touch and keyboard control.
 */

import React, { useState, useRef, useCallback, useEffect } from 'react';
import { cn } from '../../lib/utils';

interface JoystickProps {
    className?: string;
    onMove: (direction: 'forward' | 'backward' | 'left' | 'right' | null, intensity: number) => void;
    onStop?: () => void;
    disabled?: boolean;
    size?: number;
}

export function Joystick({ className, onMove, onStop, disabled = false, size = 150 }: JoystickProps) {
    const [isDragging, setIsDragging] = useState(false);
    const [position, setPosition] = useState({ x: 0, y: 0 });
    const containerRef = useRef<HTMLDivElement>(null);
    const knobRef = useRef<HTMLDivElement>(null);

    const radius = size / 2;
    const maxDistance = radius * 0.6; // Maximum knob movement

    // Calculate direction and intensity from position
    const calculateDirection = useCallback((x: number, y: number) => {
        const distance = Math.sqrt(x * x + y * y);
        const intensity = Math.min(distance / maxDistance, 1);

        if (distance < 10) {
            return { direction: null, intensity: 0 };
        }

        const angle = Math.atan2(y, x) * (180 / Math.PI);

        // Determine direction based on angle
        let direction: 'forward' | 'backward' | 'left' | 'right' | null = null;

        if (angle >= -45 && angle < 45) {
            direction = 'right'; // X positive
        } else if (angle >= 45 && angle < 135) {
            direction = 'backward'; // Y positive (down)
        } else if (angle >= -135 && angle < -45) {
            direction = 'forward'; // Y negative (up)
        } else {
            direction = 'left'; // X negative
        }

        return { direction, intensity };
    }, [maxDistance]);

    // Handle move events
    const handleMove = useCallback((clientX: number, clientY: number) => {
        if (!containerRef.current) return;

        const rect = containerRef.current.getBoundingClientRect();
        const centerX = rect.left + radius;
        const centerY = rect.top + radius;

        let x = clientX - centerX;
        let y = clientY - centerY;

        // Clamp to max distance
        const distance = Math.sqrt(x * x + y * y);
        if (distance > maxDistance) {
            const ratio = maxDistance / distance;
            x *= ratio;
            y *= ratio;
        }

        setPosition({ x, y });

        const { direction, intensity } = calculateDirection(x, y);
        onMove(direction, intensity);
    }, [radius, maxDistance, calculateDirection, onMove]);

    // Mouse events
    const handleMouseDown = useCallback((e: React.MouseEvent) => {
        if (disabled) return;
        setIsDragging(true);
        handleMove(e.clientX, e.clientY);
    }, [disabled, handleMove]);

    // Touch events
    const handleTouchStart = useCallback((e: React.TouchEvent) => {
        if (disabled) return;
        e.preventDefault();
        setIsDragging(true);
        const touch = e.touches[0];
        handleMove(touch.clientX, touch.clientY);
    }, [disabled, handleMove]);

    // Global move and stop handlers
    useEffect(() => {
        if (!isDragging) return;

        const handleMouseMove = (e: MouseEvent) => {
            handleMove(e.clientX, e.clientY);
        };

        const handleTouchMove = (e: TouchEvent) => {
            e.preventDefault();
            const touch = e.touches[0];
            handleMove(touch.clientX, touch.clientY);
        };

        const handleStop = () => {
            setIsDragging(false);
            setPosition({ x: 0, y: 0 });
            onMove(null, 0);
            onStop?.();
        };

        document.addEventListener('mousemove', handleMouseMove);
        document.addEventListener('touchmove', handleTouchMove, { passive: false });
        document.addEventListener('mouseup', handleStop);
        document.addEventListener('touchend', handleStop);

        return () => {
            document.removeEventListener('mousemove', handleMouseMove);
            document.removeEventListener('touchmove', handleTouchMove);
            document.removeEventListener('mouseup', handleStop);
            document.removeEventListener('touchend', handleStop);
        };
    }, [isDragging, handleMove, onMove, onStop]);

    // Keyboard controls
    useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (disabled) return;

            const keyMap: Record<string, 'forward' | 'backward' | 'left' | 'right'> = {
                'ArrowUp': 'forward',
                'w': 'forward',
                'W': 'forward',
                'ArrowDown': 'backward',
                's': 'backward',
                'S': 'backward',
                'ArrowLeft': 'left',
                'a': 'left',
                'A': 'left',
                'ArrowRight': 'right',
                'd': 'right',
                'D': 'right',
            };

            const direction = keyMap[e.key];
            if (direction) {
                e.preventDefault();
                const newPos = { x: 0, y: 0 };
                if (direction === 'forward') newPos.y = -maxDistance;
                if (direction === 'backward') newPos.y = maxDistance;
                if (direction === 'left') newPos.x = -maxDistance;
                if (direction === 'right') newPos.x = maxDistance;

                setPosition(newPos);
                onMove(direction, 1);
            }
        };

        const handleKeyUp = (e: KeyboardEvent) => {
            const keys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', 'W', 'A', 'S', 'D'];
            if (keys.includes(e.key)) {
                setPosition({ x: 0, y: 0 });
                onMove(null, 0);
                onStop?.();
            }
        };

        document.addEventListener('keydown', handleKeyDown);
        document.addEventListener('keyup', handleKeyUp);

        return () => {
            document.removeEventListener('keydown', handleKeyDown);
            document.removeEventListener('keyup', handleKeyUp);
        };
    }, [disabled, maxDistance, onMove, onStop]);

    return (
        <div
            ref={containerRef}
            className={cn(
                'relative rounded-full bg-slate-800 border-2 border-slate-600',
                'select-none touch-none',
                disabled && 'opacity-50 cursor-not-allowed',
                className
            )}
            style={{ width: size, height: size }}
            onMouseDown={handleMouseDown}
            onTouchStart={handleTouchStart}
        >
            {/* Direction indicators */}
            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                <div className="w-full h-full relative">
                    {/* Arrow indicators */}
                    <div className="absolute top-2 left-1/2 -translate-x-1/2 text-slate-500 text-xs">↑</div>
                    <div className="absolute bottom-2 left-1/2 -translate-x-1/2 text-slate-500 text-xs">↓</div>
                    <div className="absolute left-2 top-1/2 -translate-y-1/2 text-slate-500 text-xs">←</div>
                    <div className="absolute right-2 top-1/2 -translate-y-1/2 text-slate-500 text-xs">→</div>
                </div>
            </div>

            {/* Center indicator */}
            <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-4 h-4 rounded-full bg-slate-700" />

            {/* Draggable knob */}
            <div
                ref={knobRef}
                className={cn(
                    'absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2',
                    'w-12 h-12 rounded-full shadow-lg transition-colors',
                    isDragging || (position.x !== 0 || position.y !== 0)
                        ? 'bg-primary shadow-primary/50'
                        : 'bg-slate-400'
                )}
                style={{
                    transform: `translate(calc(-50% + ${position.x}px), calc(-50% + ${position.y}px))`,
                }}
            />

            {/* Direction label */}
            <div className="absolute -bottom-6 left-1/2 -translate-x-1/2 text-xs text-slate-400 whitespace-nowrap">
                {isDragging ? '拖动控制' : '拖动或 WASD'}
            </div>
        </div>
    );
}
