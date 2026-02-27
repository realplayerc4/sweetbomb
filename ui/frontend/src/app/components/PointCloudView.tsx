import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Box } from 'lucide-react';

interface PointCloudViewProps {
  isActive: boolean;
  points: Float32Array | null;
  metrics?: { pointCount: number } | null;
  camZ?: number;
  camX?: number;
}

// Custom shader for circular particles with border
const vertexShader = `
  attribute float aSize;
  varying vec2 vUv;

  void main() {
    vUv = uv;
    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mvPosition;
    gl_PointSize = aSize * (3.0 / -mvPosition.z);
  }
`;

const fragmentShader = `
  varying vec2 vUv;

  void main() {
    // Calculate distance from center of point
    vec2 center = gl_PointCoord - vec2(0.5);
    float dist = length(center);

    // Discard pixels outside circle
    if (dist > 0.5) discard;

    // Graphite Orange fill, dark border
    vec3 fillColor = vec3(0.99, 0.50, 0.18); // 0xFD802E
    vec3 borderColor = vec3(0.3, 0.1, 0.0);

    // Border region (outer ring)
    float t = smoothstep(0.3, 0.45, dist);
    vec3 color = mix(fillColor, borderColor, t);

    // Soft edge
    float alpha = 0.9 - smoothstep(0.45, 0.5, dist);

    gl_FragColor = vec4(color, alpha);
  }
`;

export function PointCloudView({ isActive, points, metrics, camZ = 3.0, camX = -5.0 }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const pointsRef = useRef<THREE.Points | null>(null);
  const geometryRef = useRef<THREE.BufferGeometry | null>(null);
  const animationIdRef = useRef<number | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    const width = containerRef.current.clientWidth;
    const height = containerRef.current.clientHeight;

    // Scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#1c1c1e');
    sceneRef.current = scene;

    // Camera - 适应73系原版对齐
    const camera = new THREE.PerspectiveCamera(60, width / height, 0.01, 1000);
    camera.up.set(0, 0, 1);        // 设Z轴向上
    camera.position.set(-5, 0, 3); // 固定观测点: 从X轴负向后方往回看
    camera.lookAt(0, 0, 0);        // 看向原点
    cameraRef.current = camera;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Grid Helper - XY 平面作为地面 (Z-up 坐标系)
    const grid = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
    grid.rotation.x = Math.PI / 2; // 将默认的 XZ 平面翻转至 XY 平面
    scene.add(grid);

    // Axes Helper - 添加坐标轴以排查方位与原点
    // X轴为红色, Y轴为绿色, Z轴为蓝色
    const axesHelper = new THREE.AxesHelper(3);
    scene.add(axesHelper);

    // Points Geometry & Custom Shader Material
    const geometry = new THREE.BufferGeometry();
    const maxPoints = 1280 * 720;
    const positions = new Float32Array(maxPoints * 3);
    const sizes = new Float32Array(maxPoints);

    // Initialize all sizes
    for (let i = 0; i < maxPoints; i++) {
      sizes[i] = 1.5; // 对齐73粒子大小（原本为4.5，缩减带来更精细画面）
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('aSize', new THREE.BufferAttribute(sizes, 1));
    geometry.setDrawRange(0, 0);
    geometryRef.current = geometry;

    const material = new THREE.ShaderMaterial({
      uniforms: {},
      vertexShader,
      fragmentShader,
      transparent: true,
      depthWrite: false,
      blending: THREE.AdditiveBlending,
    });

    const pointsObj = new THREE.Points(geometry, material);
    pointsObj.frustumCulled = false; // CRITICAL FIX: Don't cull points even if bounds are weird
    scene.add(pointsObj);
    pointsRef.current = pointsObj;

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.enableZoom = true;
    controls.enableRotate = true;
    controls.enablePan = true;
    controls.target.set(0, 0, 0); // 轨道旋转中心对应 lookAt原点

    const animate = () => {
      animationIdRef.current = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      if (!containerRef.current || !camera || !renderer) return;
      const w = containerRef.current.clientWidth;
      const h = containerRef.current.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
      if (animationIdRef.current) cancelAnimationFrame(animationIdRef.current);
      if (containerRef.current && renderer.domElement) {
        containerRef.current.removeChild(renderer.domElement);
      }
      geometry.dispose();
      material.dispose();
      renderer.dispose();
    };
  }, []);

  // Update Camera View dynamically
  useEffect(() => {
    if (cameraRef.current && (camX !== -5.0 || camZ !== 3.0)) {
      cameraRef.current.position.set(camX, 0, camZ);
      cameraRef.current.lookAt(0, 0, 0);
    }
  }, [camX, camZ]);

  // Update points - optimized for performance
  useEffect(() => {
    if (points && geometryRef.current) {
      const positions = geometryRef.current.attributes.position.array as Float32Array;

      if (positions.length < points.length) {
        geometryRef.current.setAttribute('position', new THREE.BufferAttribute(points, 3));
      } else {
        positions.set(points);
        geometryRef.current.attributes.position.needsUpdate = true;
      }
      const drawCount = points.length / 3;
      geometryRef.current.setDrawRange(0, drawCount);
      // Update bounding sphere manually because frustumCulled is false anyway, 
      // but good practice if we ever turn it back on
      geometryRef.current.computeBoundingSphere();
    }
  }, [points]);

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-2xl overflow-hidden border border-white/5 shadow-md group">
      {/* 悬浮状态胶囊 */}
      <div className="absolute top-4 left-4 z-10 flex items-center gap-2 bg-black/40 backdrop-blur-md px-3 py-1.5 rounded-full border border-white/10">
        <Box className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-slate-100 font-bold tracking-widest uppercase">Point Cloud</span>
        {metrics && isActive && (
          <span className="text-[10px] text-slate-400 border-l border-white/20 pl-2 ml-1 font-medium">
            {metrics.pointCount.toLocaleString()} POINTS
          </span>
        )}
      </div>

      <div ref={containerRef} className="w-full h-full" />
    </div>
  );
}
