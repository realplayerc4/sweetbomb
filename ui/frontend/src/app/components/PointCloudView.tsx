import { useEffect, useRef, useState, useMemo } from 'react';
import * as THREE from 'three';
import { Cloud } from 'lucide-react';

interface PointCloudViewProps {
  isActive: boolean;
  points: Float32Array | null;
}

const STORAGE_KEY = 'pointcloud-settings';

interface RangeSettings {
  downsample: number;
  particleSize: number;
}

const loadSettings = (): RangeSettings => {
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      const parsed = JSON.parse(saved);
      return {
        downsample: parsed.downsample ?? 1,
        particleSize: parsed.particleSize ?? 4.5,
      };
    }
  } catch (e) {
    console.warn('Failed to load settings:', e);
  }
  return {
    downsample: 1,
    particleSize: 4.5,
  };
};

export function PointCloudView({ isActive, points }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const pointCloudGroupRef = useRef<THREE.Group | null>(null);
  const pointsRef = useRef<THREE.Points | null>(null);
  const animationFrameRef = useRef<number>(0);
  const [isHovered, setIsHovered] = useState(false);
  const [filteredPointCount, setFilteredPointCount] = useState(0);

  // Camera orbit state (spherical coordinates)
  const orbitRef = useRef({
    target: new THREE.Vector3(0, 0, 0),
    radius: 4,
    theta: Math.PI / 4,
    phi: Math.PI / 3,
  });

  const savedSettings = useMemo(() => loadSettings(), []);
  const [downsample, setDownsample] = useState<number>(savedSettings.downsample);
  const [particleSize, setParticleSize] = useState<number>(savedSettings.particleSize);

  // Debug logging
  useEffect(() => {
    if (filteredPointCount > 0) {
      console.log('[PointCloud]', filteredPointCount, 'points rendered');
    }
  }, [filteredPointCount]);

  // Save settings when changed
  useEffect(() => {
    const timer = setTimeout(() => {
      const settings: RangeSettings = {
        downsample,
        particleSize,
      };
      localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
    }, 500);
    return () => clearTimeout(timer);
  }, [downsample, particleSize]);

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
    pointCloudGroupRef.current = pointCloudGroup;

    const axesHelper = new THREE.AxesHelper(1.5);
    pointCloudGroup.add(axesHelper);

    const gridHelper = new THREE.GridHelper(6, 12, 0x444444, 0x333333);
    gridHelper.rotation.x = Math.PI / 2;
    pointCloudGroup.add(gridHelper);

    // Mouse controls
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

        orbit.theta -= deltaX * sensitivity;
        orbit.phi -= deltaY * sensitivity;
        orbit.phi = Math.max(0.1, Math.min(Math.PI - 0.1, orbit.phi));

        updateCameraFromOrbit();
      } else if (draggedButton === 2) {
        // Right click: Pan in screen plane
        const panSpeed = 0.005 * orbitRef.current.radius;
        const orbit = orbitRef.current;

        const direction = new THREE.Vector3();
        camera.getWorldDirection(direction);
        const right = new THREE.Vector3();
        right.crossVectors(direction, camera.up).normalize();
        const up = new THREE.Vector3();
        up.crossVectors(right, direction).normalize();

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

    const resizeObserver = new ResizeObserver((entries: ResizeObserverEntry[]) => {
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
      pointCloudGroupRef.current = null;
    };
  }, []);

  // Point cloud rendering with simplified filtering
  useEffect(() => {
    if (!sceneRef.current || !points || points.length === 0) {
      setFilteredPointCount(0);
      return;
    }

    const pointCloudGroup = pointCloudGroupRef.current;
    if (!pointCloudGroup) return;

    // Fixed filter ranges: X: 1~6m, Y: no limit, Z: -0.6~2m
    const minX = 1.0;
    const maxX = 6.0;
    const minY = -Infinity;  // No Y limit
    const maxY = Infinity;
    const minZ = -0.6;
    const maxZ = 2.0;

    const filteredPositions: number[] = [];
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

    try {
      let pointsObj = pointsRef.current;

      if (!pointsObj || !pointsObj.geometry) {
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(filteredPositions), 3));

        const material = new THREE.PointsMaterial({
          size: particleSize,
          color: 0xFD802E,
          transparent: false,
          depthWrite: false, // 修复粒子由于深度测试冲突导致的不显示问题
          blending: THREE.NormalBlending,
          sizeAttenuation: false, // 设置为 false，使得 size (4.5) 以像素为单位
        });

        pointsObj = new THREE.Points(geometry, material);
        pointsObj.frustumCulled = false;
        geometry.computeBoundingSphere();
        pointCloudGroup.add(pointsObj);
        pointsRef.current = pointsObj;
      } else {
        const geometry = pointsObj.geometry;
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(filteredPositions), 3));
        geometry.attributes.position.needsUpdate = true;
        geometry.computeBoundingSphere();

        if ('size' in pointsObj.material) {
          (pointsObj.material as THREE.PointsMaterial).size = particleSize;
        }
      }
    } catch (e) {
      console.error('[PointCloud] Error updating points:', e);
    }
  }, [points, downsample, particleSize]);

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md cursor-grab active:cursor-grabbing"
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      {/* Top Status Capsule */}
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

      {/* Controls Panel - Downsample & Particle Size */}
      <div className="absolute bottom-[10px] right-[10px] z-10 p-3 rounded-lg border-2 border-[#FD802E] bg-black/90">
        {/* Downsample Control */}
        <div className="mb-3">
          <div className="flex items-center justify-between mb-1">
            <span className="text-[10px] text-[#FD802E] font-mono">降采样</span>
            <span className="text-[10px] text-[#FD802E] font-mono">1/{downsample}</span>
          </div>
          <input
            type="range"
            min="1"
            max="16"
            step="1"
            value={downsample}
            onChange={(e) => setDownsample(parseInt(e.target.value))}
            className="w-full h-1 bg-transparent rounded appearance-none cursor-pointer accent-[#FD802E]"
          />
        </div>

        {/* Particle Size Control */}
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-[10px] text-[#FD802E] font-mono">粒子大小</span>
            <span className="text-[10px] text-[#FD802E] font-mono">{particleSize.toFixed(2)}</span>
          </div>
          <input
            type="range"
            min="1.0"
            max="10.0"
            step="0.1"
            value={particleSize}
            onChange={(e) => setParticleSize(parseFloat(e.target.value))}
            className="w-full h-1 bg-transparent rounded appearance-none cursor-pointer accent-[#FD802E]"
          />
        </div>
      </div>

      {/* Mouse Controls Hint */}
      {isHovered && (
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
