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

export function PointCloudView({ isActive, points, metrics, camZ = 3.0, camX = -5.0 }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const pointsRef = useRef<THREE.Points | null>(null);
  const animationIdRef = useRef<number | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    const width = containerRef.current.clientWidth;
    const height = containerRef.current.clientHeight;

    // Scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#1c1c1e');
    sceneRef.current = scene;

    // Camera - 适应机器人Z-up坐标系
    const camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
    camera.up.set(0, 0, 1);        // 设Z轴向上
    camera.position.set(camX, 0, camZ);
    camera.lookAt(2, 0, 0);        // 看向机器人前方
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

    // Points Geometry
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({
      size: 0.1,
      vertexColors: false,
      color: 0xFD802E, // 石墨橙品牌色
      transparent: true,
      opacity: 0.9,
      depthWrite: false,
      blending: THREE.NormalBlending
    });

    const pointsObj = new THREE.Points(geometry, material);
    pointsObj.frustumCulled = false; // CRITICAL FIX: Don't cull points even if bounds are weird
    scene.add(pointsObj);
    pointsRef.current = pointsObj;

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(2, 0, 0);

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
    if (cameraRef.current) {
      cameraRef.current.position.set(camX, 0, camZ);
      cameraRef.current.lookAt(2, 0, 0);
    }
  }, [camX, camZ]);

  // Update Points content
  useEffect(() => {
    if (points && pointsRef.current) {
      const geometry = pointsRef.current.geometry;
      geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
      geometry.attributes.position.needsUpdate = true;
      geometry.computeBoundingSphere();
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
