import { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import * as THREE from 'three';
import { Cloud } from 'lucide-react';

interface PointCloudViewProps {
  isActive: boolean;
  points: Float32Array | null;
}

const STORAGE_KEY = 'pointcloud-settings';
const DEFAULT_RANGE = { min: -1, max: 1 };

interface Range {
  min: number;
  max: number;
}

interface RangeSettings {
  x: Range;
  y: Range;
  z: Range;
  downsample: number;
}

const loadSettings = (): RangeSettings => {
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      return JSON.parse(saved);
    }
  } catch (e) {
    console.warn('Failed to load settings:', e);
  }
  return {
    x: DEFAULT_RANGE,
    y: DEFAULT_RANGE,
    z: DEFAULT_RANGE,
    downsample: 1
  };
};

export function PointCloudView({ isActive, points }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const pointsRef = useRef<THREE.Points | null>(null);
  const rotationRef = useRef({ x: 0.3, y: 0 });
  const panRef = useRef({ x: 0, y: 0 });
  const zoomRef = useRef(1.0);
  const animationFrameRef = useRef<number>(0);
  const [isHovered, setIsHovered] = useState(false);

  const savedSettings = useMemo(() => loadSettings(), []);
  const [rangeX, setRangeX] = useState<Range>(savedSettings.x);
  const [rangeY, setRangeY] = useState<Range>(savedSettings.y);
  const [rangeZ, setRangeZ] = useState<Range>(savedSettings.z);
  const [downsample, setDownsample] = useState<number>(savedSettings.downsample);
  const [filteredPointCount, setFilteredPointCount] = useState(0);

  useEffect(() => {
    const timer = setTimeout(() => {
      const settings: RangeSettings = {
        x: rangeX,
        y: rangeY,
        z: rangeZ,
        downsample
      };
      localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
    }, 500);
    return () => clearTimeout(timer);
  }, [rangeX, rangeY, rangeZ, downsample]);

  const handleRangeChange = useCallback(
    (axis: 'x' | 'y' | 'z', field: 'min' | 'max', value: number) => {
      if (axis === 'x') setRangeX((prev) => ({ ...prev, [field]: value }));
      if (axis === 'y') setRangeY((prev) => ({ ...prev, [field]: value }));
      if (axis === 'z') setRangeZ((prev) => ({ ...prev, [field]: value }));
    },
    []
  );

  useEffect(() => {
    if (!containerRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1c1c1e);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(
      60,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.01,
      100
    );
    camera.up.set(0, 0, 1);
    camera.position.set(0, -6, 4);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({
      antialias: false,
      alpha: true,
      preserveDrawingBuffer: false,
    });
    renderer.setPixelRatio(1);
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const pointCloudGroup = new THREE.Group();
    scene.add(pointCloudGroup);

    const axesHelper = new THREE.AxesHelper(1.5);
    pointCloudGroup.add(axesHelper);

    const gridHelper = new THREE.GridHelper(6, 12, 0x444444, 0x333333);
    gridHelper.rotation.x = Math.PI / 2;
    pointCloudGroup.add(gridHelper);

    pointCloudGroup.rotation.x = rotationRef.current.x;
    pointCloudGroup.rotation.y = rotationRef.current.y;

    let isDragging = false;
    let draggedButton: number | null = null;
    let previousMousePosition = { x: 0, y: 0 };

    const onContextMenu = (e: Event) => e.preventDefault();

    const onMouseDown = (e: MouseEvent) => {
      if (e.button === 2) e.preventDefault();
      isDragging = true;
      draggedButton = e.button;
      previousMousePosition = { x: e.clientX, y: e.clientY };
    };

    const onMouseUp = () => {
      isDragging = false;
      draggedButton = null;
    };

    const onMouseMove = (e: MouseEvent) => {
      if (!isDragging || draggedButton === null) return;

      const deltaX = e.clientX - previousMousePosition.x;
      const deltaY = e.clientY - previousMousePosition.y;

      if (draggedButton === 0) {
        rotationRef.current.y -= deltaX * 0.005;
        rotationRef.current.x -= deltaY * 0.005;
        rotationRef.current.x = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, rotationRef.current.x));
        pointCloudGroup.rotation.x = rotationRef.current.x;
        pointCloudGroup.rotation.y = rotationRef.current.y;
      } else if (draggedButton === 2) {
        panRef.current.x += deltaX * 0.01 * zoomRef.current;
        panRef.current.y -= deltaY * 0.01 * zoomRef.current;
        camera.position.set(panRef.current.x, -6 + panRef.current.y, 4);
      }

      previousMousePosition = { x: e.clientX, y: e.clientY };
    };

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const delta = e.deltaY > 0 ? 1.1 : 0.9;
      zoomRef.current *= delta;
      zoomRef.current = Math.max(0.1, Math.min(5.0, zoomRef.current));
      camera.fov = 60 / zoomRef.current;
      camera.updateProjectionMatrix();
    };

    containerRef.current.addEventListener('mousedown', onMouseDown);
    containerRef.current.addEventListener('contextmenu', onContextMenu);
    containerRef.current.addEventListener('wheel', onWheel, { passive: false });
    window.addEventListener('mouseup', onMouseUp);
    window.addEventListener('mousemove', onMouseMove);

    const animate = () => {
      animationFrameRef.current = requestAnimationFrame(animate);
      if (rendererRef.current && sceneRef.current && cameraRef.current) {
        rendererRef.current.render(sceneRef.current, cameraRef.current);
      }
    };
    animate();

    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (cameraRef.current && rendererRef.current) {
          cameraRef.current.aspect = width / height;
          cameraRef.current.updateProjectionMatrix();
          rendererRef.current.setSize(width, height);
        }
      }
    });
    resizeObserver.observe(containerRef.current);

    return () => {
      resizeObserver.disconnect();
      cancelAnimationFrame(animationFrameRef.current);
      if (rendererRef.current) rendererRef.current.dispose();
      if (containerRef.current && rendererRef.current && rendererRef.current.domElement.parentNode === containerRef.current) {
        containerRef.current.removeChild(rendererRef.current.domElement);
      }
      containerRef.current?.removeEventListener('mousedown', onMouseDown);
      containerRef.current?.removeEventListener('contextmenu', onContextMenu);
      containerRef.current?.removeEventListener('wheel', onWheel);
      window.removeEventListener('mouseup', onMouseUp);
      window.removeEventListener('mousemove', onMouseMove);
    };
  }, []);

  useEffect(() => {
    if (!sceneRef.current || !points || points.length === 0) {
      setFilteredPointCount(0);
      return;
    }

    const pointCloudGroup = sceneRef.current.children.find(
      (child) => child instanceof THREE.Group
    ) as THREE.Group | undefined;

    if (!pointCloudGroup) return;

    const oldPoints = pointCloudGroup.children.find(
      (child) => child instanceof THREE.Points
    ) as THREE.Points | undefined;

    if (oldPoints) {
      pointCloudGroup.remove(oldPoints);
      if (oldPoints.geometry) oldPoints.geometry.dispose();
      if (oldPoints.material) (oldPoints.material as THREE.Material).dispose();
    }

    const filteredPositions: number[] = [];
    const { min: minX, max: maxX } = rangeX;
    const { min: minY, max: maxY } = rangeY;
    const { min: minZ, max: maxZ } = rangeZ;

    const step = downsample;
    for (let i = 0; i < points.length; i += 3 * step) {
      const rosX = points[i];
      const rosY = points[i + 1];
      const rosZ = points[i + 2];

      if (rosX >= minX && rosX <= maxX &&
          rosY >= minY && rosY <= maxY &&
          rosZ >= minZ && rosZ <= maxZ) {
        filteredPositions.push(rosX, rosY, rosZ);
      }
    }

    const count = filteredPositions.length / 3;
    setFilteredPointCount(count);

    if (count === 0) return;

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(filteredPositions), 3));

    const material = new THREE.PointsMaterial({
      size: 0.05,
      color: 0xFD802E,
      transparent: false,
      depthWrite: true,
      blending: THREE.NormalBlending,
    });

    const pointsObj = new THREE.Points(geometry, material);
    pointsObj.frustumCulled = false;

    pointCloudGroup.add(pointsObj);
    pointsRef.current = pointsObj;
  }, [points, rangeX, rangeY, rangeZ, downsample]);

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md cursor-grab active:cursor-grabbing"
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        {isActive ? (
          <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
        ) : (
          <div className="w-3 h-3 bg-slate-500 rounded-full" />
        )}
        <Cloud className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> Point Cloud </span>
        <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
          | {filteredPointCount.toLocaleString()} POINTS
        </span>
      </div>

      <div className="absolute bottom-[10px] left-[10px] z-10 bg-black/80 backdrop-blur-md p-3 rounded-lg border border-slate-700">
        <div className="text-[9px] text-slate-400 font-mono mb-2">ROS 坐标范围 (米)</div>
        <div className="space-y-1.5 mb-3">
          {['x', 'y', 'z'].map((axis) => {
            const axisColor = axis === 'x' ? 'text-[#FD802E]' : axis === 'y' ? 'text-[#00BFFF]' : 'text-[#32CD32]';
            const range = axis === 'x' ? rangeX : axis === 'y' ? rangeY : rangeZ;
            return (
              <div key={axis} className="flex items-center gap-2">
                <span className={`text-[9px] ${axisColor} font-mono w-6 uppercase`}>{axis}</span>
                <input
                  type="number"
                  step="0.1"
                  value={range.min}
                  onChange={(e) => handleRangeChange(axis as any, 'min', parseFloat(e.target.value) || 0)}
                  className="w-14 h-5 bg-slate-800 border border-slate-600 rounded text-[9px] text-white px-1 font-mono"
                />
                <span className="text-slate-600">~</span>
                <input
                  type="number"
                  step="0.1"
                  value={range.max}
                  onChange={(e) => handleRangeChange(axis as any, 'max', parseFloat(e.target.value) || 0)}
                  className="w-14 h-5 bg-slate-800 border border-slate-600 rounded text-[9px] text-white px-1 font-mono"
                />
              </div>
            );
          })}
        </div>
        <div className="border-t border-slate-600 pt-2">
          <div className="flex items-center justify-between mb-1">
            <span className="text-[9px] text-slate-400 font-m-mono">降采样</span>
            <span className="text-[9px] text-[#FD802E] font-mono">1/{downsample}</span>
          </div>
          <div className="flex items-center gap-2">
            <input
              type="range"
              min="1"
              max="16"
              step="1"
              value={downsample}
              onChange={(e) => setDownsample(parseInt(e.target.value))}
              className="flex-1 h-5 bg-slate-800 rounded appearance-none cursor-pointer"
            />
          </div>
        </div>
      </div>

      {isHovered && isActive && (
        <div className="absolute bottom-[10px] right-[10px] z-10 flex items-center justify-center gap-3 bg-black/80 backdrop-blur-md px-4 py-2 rounded-full text-[10px] text-slate-400 font-mono">
          <span className="flex items-center gap-1"><span className="text-[#FD802E]">LMB</span> 旋转</span>
          <span className="text-slate-600">|</span>
          <span className="flex items-center gap-1"><span className="text-[#FD802E]">RMB</span> 平移</span>
          <span className="text-slate-600">|</span>
          <span className="flex items-center gap-1"><span className="text-[#FD802E]">Wheel</span> 缩放</span>
        </div>
      )}

      {!isActive && (
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="text-center">
            <div className="w-16 h-16 rounded-full bg-slate-700/50 flex items-center justify-center mx-auto mb-3">
              <Cloud className="w-8 h-8 text-slate-500" />
            </div>
            <p className="text-slate-500 text-sm">等待流启动...</p>
            <p className="text-slate-600 text-xs mt-1">ROS 坐标系</p>
          </div>
        </div>
      )}
    </div>
  );
}
