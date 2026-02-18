
import { useEffect, useRef } from 'react';
import { Box } from 'lucide-react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

interface PointCloudViewProps {
  isActive: boolean;
  points: Float32Array | null;
}

export function PointCloudView({ isActive, points }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const geometryRef = useRef<THREE.BufferGeometry | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f);
    scene.fog = new THREE.Fog(0x0a0a0f, 2, 12);

    const camera = new THREE.PerspectiveCamera(60, containerRef.current.clientWidth / containerRef.current.clientHeight, 0.1, 100);
    camera.up.set(0, 0, 1); // Set Z-up
    camera.position.set(-2.4, -2.4, 1.6); // Slightly closer to match 80% ROI feel
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.update();

    // Geometry
    const geometry = new THREE.BufferGeometry();
    const maxPoints = 1280 * 720; // Match higher resolution
    const positions = new Float32Array(maxPoints * 3);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setDrawRange(0, 0);
    geometryRef.current = geometry;

    const material = new THREE.PointsMaterial({
      color: 0x00d2ff,
      size: 0.008,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.8
    });

    const pointsObj = new THREE.Points(geometry, material);
    scene.add(pointsObj);

    // Visual Helpers (Mimicking OctoMap Viewer)
    const gridHelper = new THREE.GridHelper(10, 20, 0x303040, 0x202030);
    gridHelper.rotation.x = Math.PI / 2; // Rotate to X-Y plane (Z-up)
    scene.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);

    // Ambient Light to help see axes if needed (though they are emissive)
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    // Animation loop
    let animationId: number;
    const animate = () => {
      animationId = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      if (!containerRef.current) return;
      camera.aspect = containerRef.current.clientWidth / containerRef.current.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    };
    window.addEventListener('resize', handleResize);

    return () => {
      cancelAnimationFrame(animationId);
      window.removeEventListener('resize', handleResize);
      if (rendererRef.current && containerRef.current) {
        containerRef.current.removeChild(rendererRef.current.domElement);
      }
      geometry.dispose();
      material.dispose();
    };
  }, []);

  // Update points
  useEffect(() => {
    if (points && geometryRef.current) {
      const positions = geometryRef.current.attributes.position.array as Float32Array;

      if (positions.length < points.length) {
        // Resize needed - simple recreation for now, or just warn
        console.warn("Buffer too small for points");
        geometryRef.current.setAttribute('position', new THREE.BufferAttribute(points, 3));
      } else {
        positions.set(points);
        geometryRef.current.attributes.position.needsUpdate = true;
      }
      geometryRef.current.setDrawRange(0, points.length / 3);
    }
  }, [points]);

  return (
    <div className="relative w-full h-full bg-slate-950 rounded-lg overflow-hidden border border-slate-700">
      <div className="absolute top-3 left-3 z-10 flex items-center gap-2 bg-slate-900/80 backdrop-blur-sm px-3 py-1.5 rounded-md">
        <Box className="w-4 h-4 text-green-400" />
        <span className="text-xs text-white font-mono">POINT CLOUD</span>
        {isActive && (
          <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse" />
        )}
      </div>
      <div ref={containerRef} className="w-full h-full" />
    </div>
  );
}
