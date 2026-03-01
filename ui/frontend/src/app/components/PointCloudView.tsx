import { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import * as THREE from 'three';
import { Cloud } from 'lucide-react';

interface PointCloudViewProps {
  isActive: boolean;
  points: Float32Array | null;
}

const STORAGE_KEY = 'pointcloud-settings';
const DEFAULT_RANGE = { min: -5, max: 5 };  // Expanded default range

interface Range {
  min: number;
  max: number;
}

interface RangeSettings {
  x: Range;
  y: Range;
  z: Range;
  downsample: number;
  autoRange: boolean;
}

const loadSettings = (): RangeSettings => {
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      const parsed = JSON.parse(saved);
      return {
        x: parsed.x ?? DEFAULT_RANGE,
        y: parsed.y ?? DEFAULT_RANGE,
        z: parsed.z ?? { min: -1, max: 2 },
        downsample: parsed.downsample ?? 1,
        autoRange: parsed.autoRange ?? true
      };
    }
  } catch (e) {
    console.warn('Failed to load settings:', e);
  }
  return {
    x: DEFAULT_RANGE,
    y: DEFAULT_RANGE,
    z: { min: -1, max: 2 },
    downsample: 1,
    autoRange: true
  };
};

export function PointCloudView({ isActive, points }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const pointCloudGroupRef = useRef<THREE.Group | null>(null);  // Direct ref to point cloud group
  const pointsRef = useRef<THREE.Points | null>(null);
  const animationFrameRef = useRef<number>(0);
  const [isHovered, setIsHovered] = useState(false);
  const [autoRange, setAutoRange] = useState(true);

  // Camera orbit state (spherical coordinates)
  const orbitRef = useRef({
    target: new THREE.Vector3(0, 0, 0),  // Point camera is looking at
    radius: 7,     // Distance from target
    theta: Math.PI / 2,   // Horizontal angle (around Z axis)
    phi: Math.PI / 4      // Vertical angle (from horizontal plane)
  });

  const savedSettings = useMemo(() => loadSettings(), []);
  const [rangeX, setRangeX] = useState<Range>(savedSettings.x);
  const [rangeY, setRangeY] = useState<Range>(savedSettings.y);
  const [rangeZ, setRangeZ] = useState<Range>(savedSettings.z);
  const [downsample, setDownsample] = useState<number>(savedSettings.downsample);
  const [filteredPointCount, setFilteredPointCount] = useState(0);
  const [particleSize, setParticleSize] = useState(0.5);  // Good default size for visibility

  // Generate test point cloud data when no device is connected (for debugging)
  // TEMPORARILY DISABLED to see real data state
  const displayPoints = useMemo(() => {
    return points;
  }, [points]);

  // Auto-calculate bounds from point cloud
  const pointBounds = useMemo(() => {
    const data = displayPoints;
    if (!data || data.length === 0) return null;

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    for (let i = 0; i < data.length; i += 3) {
      const x = data[i];
      const y = data[i + 1];
      const z = data[i + 2];

      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
      minZ = Math.min(minZ, z);
      maxZ = Math.max(maxZ, z);
    }

    return {
      x: { min: minX, max: maxX },
      y: { min: minY, max: maxY },
      z: { min: minZ, max: maxZ }
    };
  }, [displayPoints]);

  // Debug logging
  useEffect(() => {
    if (filteredPointCount > 0) {
      console.log('[PointCloud]', filteredPointCount, 'points rendered');
    }
  }, [filteredPointCount]);

  // Auto-focus camera on point cloud center (runs once)
  const autoFocusDoneRef = useRef(false);
  useEffect(() => {
    if (!pointBounds || !cameraRef.current || autoFocusDoneRef.current) return;

    const centerX = (pointBounds.x.min + pointBounds.x.max) / 2;
    const centerY = (pointBounds.y.min + pointBounds.y.max) / 2;
    const centerZ = (pointBounds.z.min + pointBounds.z.max) / 2;

    const sizeX = pointBounds.x.max - pointBounds.x.min;
    const sizeY = pointBounds.y.max - pointBounds.y.min;
    const sizeZ = pointBounds.z.max - pointBounds.z.min;
    const maxSize = Math.max(sizeX, sizeY, sizeZ, 1);

    // Update orbit target and radius
    orbitRef.current.target.set(centerX, centerY, centerZ);
    orbitRef.current.radius = maxSize * 2;
    orbitRef.current.theta = Math.PI / 4;  // 45 degrees
    orbitRef.current.phi = Math.PI / 3;    // 60 degrees from vertical

    autoFocusDoneRef.current = true;

    // Update camera position
    const { target, radius, theta, phi } = orbitRef.current;
    cameraRef.current.position.set(
      target.x + radius * Math.sin(phi) * Math.cos(theta),
      target.y + radius * Math.sin(phi) * Math.sin(theta),
      target.z + radius * Math.cos(phi)
    );
    cameraRef.current.lookAt(target);
    cameraRef.current.updateProjectionMatrix();
  }, [pointBounds]);

  // Auto-adjust ranges when bounds change
  useEffect(() => {
    if (autoRange && pointBounds) {
      const padding = 0.5;
      setRangeX({
        min: pointBounds.x.min - padding,
        max: pointBounds.x.max + padding
      });
      setRangeY({
        min: pointBounds.y.min - padding,
        max: pointBounds.y.max + padding
      });
      setRangeZ({
        min: pointBounds.z.min - padding,
        max: pointBounds.z.max + padding
      });
    }
  }, [pointBounds, autoRange]);

  useEffect(() => {
    const timer = setTimeout(() => {
      const settings: RangeSettings = {
        x: rangeX,
        y: rangeY,
        z: rangeZ,
        downsample,
        autoRange
      };
      localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
    }, 500);
    return () => clearTimeout(timer);
  }, [rangeX, rangeY, rangeZ, downsample, autoRange]);

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

    // Initialize camera from orbit state
    const updateCameraFromOrbit = () => {
      const { target, radius, theta, phi } = orbitRef.current;
      const x = target.x + radius * Math.sin(phi) * Math.cos(theta);
      const y = target.y + radius * Math.sin(phi) * Math.sin(theta);
      const z = target.z + radius * Math.cos(phi);
      camera.position.set(x, y, z);
      camera.lookAt(target);
    };

    const camera = new THREE.PerspectiveCamera(
      60,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.01,
      100
    );
    camera.up.set(0, 0, 1);  // Z is up in ROS coordinate system
    cameraRef.current = camera;
    updateCameraFromOrbit();

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
    pointCloudGroupRef.current = pointCloudGroup;  // Save reference

    const axesHelper = new THREE.AxesHelper(1.5);
    pointCloudGroup.add(axesHelper);

    const gridHelper = new THREE.GridHelper(6, 12, 0x444444, 0x333333);
    gridHelper.rotation.x = Math.PI / 2;
    pointCloudGroup.add(gridHelper);

    // No scene rotation - we use camera orbit controls instead

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
        // Left click: Orbit rotation (spherical coordinates)
        const sensitivity = 0.005;
        const orbit = orbitRef.current;

        // Theta: horizontal rotation around Z axis
        orbit.theta -= deltaX * sensitivity;

        // Phi: vertical rotation, clamped to avoid gimbal lock
        orbit.phi -= deltaY * sensitivity;
        orbit.phi = Math.max(0.1, Math.min(Math.PI - 0.1, orbit.phi));

        updateCameraFromOrbit();
      } else if (draggedButton === 2) {
        // Right click: Pan in screen plane
        const panSpeed = 0.005 * orbitRef.current.radius;
        const orbit = orbitRef.current;

        // Get camera right and up vectors
        const direction = new THREE.Vector3();
        camera.getWorldDirection(direction);
        const right = new THREE.Vector3();
        right.crossVectors(direction, camera.up).normalize();
        const up = new THREE.Vector3();
        up.crossVectors(right, direction).normalize();

        // Move target in pan direction
        orbit.target.addScaledVector(right, -deltaX * panSpeed);
        orbit.target.addScaledVector(up, deltaY * panSpeed);

        updateCameraFromOrbit();
      }

      previousMousePosition = { x: e.clientX, y: e.clientY };
    };

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const delta = e.deltaY > 0 ? 1.1 : 0.9;
      const orbit = orbitRef.current;
      orbit.radius *= delta;
      orbit.radius = Math.max(1, Math.min(50, orbit.radius));
      updateCameraFromOrbit();
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
      if (pointsRef.current) {
        if (pointCloudGroupRef.current) {
          pointCloudGroupRef.current.remove(pointsRef.current);
        }
        if (pointsRef.current.geometry) pointsRef.current.geometry.dispose();
        if (pointsRef.current.material) (pointsRef.current.material as THREE.Material).dispose();
        pointsRef.current = null;
      }
      pointCloudGroupRef.current = null;  // Clear ref on cleanup
    };
  }, []);

  // Use ref to track previous data for comparison
  const prevPointsRef = useRef<Float32Array | null>(null);

  useEffect(() => {
    if (!sceneRef.current || !displayPoints || displayPoints.length === 0) {
      setFilteredPointCount(0);
      prevPointsRef.current = null;
      return;
    }

    const pointCloudGroup = pointCloudGroupRef.current;
    if (!pointCloudGroup) return;

    const filteredPositions: number[] = [];
    const { min: minX, max: maxX } = rangeX;
    const { min: minY, max: maxY } = rangeY;
    const { min: minZ, max: maxZ } = rangeZ;

    const step = downsample;
    for (let i = 0; i < displayPoints.length; i += 3 * step) {
      const rosX = displayPoints[i];
      const rosY = displayPoints[i + 1];
      const rosZ = displayPoints[i + 2];

      if (rosX >= minX && rosX <= maxX &&
        rosY >= minY && rosY <= maxY &&
        rosZ >= minZ && rosZ <= maxZ) {
        filteredPositions.push(rosX, rosY, rosZ);
      }
    }

    const count = filteredPositions.length / 3;
    setFilteredPointCount(count);

    if (count === 0) {
      prevPointsRef.current = displayPoints;
      return;
    }

    try {
      // Reuse existing points object or create new one
      let pointsObj = pointsRef.current;

      if (!pointsObj || !pointsObj.geometry) {
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(filteredPositions), 3));

        const material = new THREE.PointsMaterial({
          size: particleSize,
          color: 0xFD802E,
          transparent: false,
          depthWrite: true,
          blending: THREE.NormalBlending,
          sizeAttenuation: true,
        });

        pointsObj = new THREE.Points(geometry, material);
        pointsObj.frustumCulled = false;
        geometry.computeBoundingSphere();
        pointCloudGroup.add(pointsObj);
        pointsRef.current = pointsObj;
      } else {
        // Update existing geometry - much more efficient
        const geometry = pointsObj.geometry;
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(filteredPositions), 3));
        geometry.attributes.position.needsUpdate = true;
        geometry.computeBoundingSphere();

        // Update material if particle size changed
        if ('size' in pointsObj.material) {
          (pointsObj.material as THREE.PointsMaterial).size = particleSize;
        }
      }

      prevPointsRef.current = displayPoints;
    } catch (e) {
      console.error('[PointCloud] Error updating points:', e);
    }
  }, [displayPoints, rangeX, rangeY, rangeZ, downsample, particleSize]);

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

      <div className="absolute bottom-[10px] left-[10px] z-10 p-2 rounded-lg border-2 border-[#FD802E] bg-black/90">
        <div className="flex items-center justify-between mb-1.5">
          <div className="text-[8px] text-[#FD802E] font-bold font-mono">ROS 范围 (米)</div>
          <button
            onClick={() => setAutoRange(!autoRange)}
            className={`text-[7px] px-1.5 py-0.5 rounded font-mono transition-colors ${autoRange
              ? 'bg-[#FD802E] text-black'
              : 'bg-slate-700 text-slate-300'
              }`}
          >
            {autoRange ? '自动' : '手动'}
          </button>
        </div>
        <div className="space-y-1 mb-2">
          {['x', 'y', 'z'].map((axis) => {
            const axisColor = axis === 'x' ? 'text-[#FD802E]' : axis === 'y' ? 'text-[#00BFFF]' : 'text-[#32CD32]';
            const range = axis === 'x' ? rangeX : axis === 'y' ? rangeY : rangeZ;
            return (
              <div key={axis} className="flex items-center gap-1">
                <span className={`text-[8px] ${axisColor} font-mono w-4 uppercase`}>{axis}</span>
                <input
                  type="number"
                  step="0.1"
                  value={range.min.toFixed(1)}
                  onChange={(e) => {
                    setAutoRange(false);
                    handleRangeChange(axis as any, 'min', parseFloat(e.target.value) || 0);
                  }}
                  disabled={autoRange}
                  className="w-10 h-4 bg-slate-800 border border-slate-600 rounded text-[8px] text-white px-1 font-mono disabled:opacity-50"
                />
                <span className="text-slate-600 text-[8px]">~</span>
                <input
                  type="number"
                  step="0.1"
                  value={range.max.toFixed(1)}
                  onChange={(e) => {
                    setAutoRange(false);
                    handleRangeChange(axis as any, 'max', parseFloat(e.target.value) || 0);
                  }}
                  disabled={autoRange}
                  className="w-10 h-4 bg-slate-800 border border-slate-600 rounded text-[8px] text-white px-1 font-mono disabled:opacity-50"
                />
              </div>
            );
          })}
        </div>
        <div className="border-t border-[#FD802E]/30 pt-1.5 mb-1.5">
          <div className="flex items-center justify-between mb-0.5">
            <span className="text-[8px] text-[#FD802E] font-mono">降采样</span>
            <span className="text-[8px] text-[#FD802E] font-mono">1/{downsample}</span>
          </div>
          <input
            type="range"
            min="1"
            max="16"
            step="1"
            value={downsample}
            onChange={(e) => setDownsample(parseInt(e.target.value))}
            className="w-full h-1 bg-slate-700 rounded appearance-none cursor-pointer accent-[#FD802E]"
          />
        </div>
        <div className="border-t border-[#FD802E]/30 pt-1.5">
          <div className="flex items-center justify-between mb-0.5">
            <span className="text-[8px] text-[#FD802E] font-mono">粒子</span>
            <span className="text-[8px] text-[#FD802E] font-mono">{particleSize.toFixed(2)}</span>
          </div>
          <input
            type="range"
            min="0.01"
            max="0.3"
            step="0.01"
            value={particleSize}
            onChange={(e) => setParticleSize(parseFloat(e.target.value))}
            className="w-full h-1 bg-slate-700 rounded appearance-none cursor-pointer accent-[#FD802E]"
          />
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

      {/* Debug info panel - top right */}
      {isHovered && pointBounds && (
        <div className="absolute top-[10px] right-[10px] z-10 bg-black/90 backdrop-blur-md p-3 rounded-lg border border-slate-700 text-[9px] font-mono max-w-[200px]">
          <div className="text-[#FD802E] font-bold mb-2 border-b border-slate-700 pb-1 flex items-center justify-between">
            <span>DEBUG INFO</span>
            {isActive && (!points || points.length === 0) && (
              <span className="text-[8px] bg-yellow-600 text-black px-1 rounded">TEST MODE</span>
            )}
          </div>
          <div className="space-y-1 text-slate-300">
            <div className="flex justify-between">
              <span>原始点数:</span>
              <span className="text-white">{points?.length || 0}</span>
            </div>
            <div className="flex justify-between">
              <span>显示点数:</span>
              <span className="text-[#FD802E]">{displayPoints?.length || 0}</span>
            </div>
            <div className="flex justify-between">
              <span>过滤后:</span>
              <span className="text-[#FD802E]">{filteredPointCount.toLocaleString()}</span>
            </div>
            <div className="border-t border-slate-700 pt-1 mt-1">
              <div className="text-slate-400 mb-1">边界 (米):</div>
              <div className="grid grid-cols-2 gap-x-2 gap-y-0.5">
                <span>X: {pointBounds.x.min.toFixed(2)}~{pointBounds.x.max.toFixed(2)}</span>
                <span>Y: {pointBounds.y.min.toFixed(2)}~{pointBounds.y.max.toFixed(2)}</span>
                <span className="col-span-2">Z: {pointBounds.z.min.toFixed(2)}~{pointBounds.z.max.toFixed(2)}</span>
              </div>
            </div>
            <div className="border-t border-slate-700 pt-1 mt-1">
              <div className="flex justify-between">
                <span>相机:</span>
                <span className="text-[#00BFFF]">
                  {cameraRef.current?.position.x.toFixed(1)}, {cameraRef.current?.position.y.toFixed(1)}, {cameraRef.current?.position.z.toFixed(1)}
                </span>
              </div>
            </div>
          </div>
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
