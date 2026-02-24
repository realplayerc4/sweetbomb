
import { useEffect, useRef } from 'react';
import { Box } from 'lucide-react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

interface PointCloudViewProps {
  isActive: boolean;
  points: Float32Array | null;
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

    // Orange fill, dark border
    vec3 fillColor = vec3(1.0, 0.4, 0.0);
    vec3 borderColor = vec3(0.2, 0.08, 0.0);

    // Border region (outer ring)
    float t = smoothstep(0.3, 0.45, dist);
    vec3 color = mix(fillColor, borderColor, t);

    // Soft edge
    float alpha = 1.0 - smoothstep(0.45, 0.5, dist);

    gl_FragColor = vec4(color, alpha);
  }
`;

export function PointCloudView({ isActive, points }: PointCloudViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const geometryRef = useRef<THREE.BufferGeometry | null>(null);
  const materialRef = useRef<THREE.ShaderMaterial | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const animationIdRef = useRef<number | null>(null);
  const initAttemptedRef = useRef(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    // Wait for container to have dimensions
    const width = container.clientWidth;
    const height = container.clientHeight;
    if (width === 0 || height === 0) {
      if (!initAttemptedRef.current) {
        initAttemptedRef.current = true;
        const retryId = setTimeout(() => {
          initAttemptedRef.current = false;
        }, 100);
        return () => clearTimeout(retryId);
      }
      return;
    }

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(60, width / height, 0.01, 1000);
    camera.up.set(0, 0, 1); // Set Z-up (height)
    camera.position.set(-5, 0, 3); // Behind (negative X), slightly above
    camera.lookAt(0, 0, 0); // Look at origin
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: false }); // Disabled antialias for performance
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5)); // Limit pixel ratio
    renderer.setSize(width, height);
    renderer.domElement.style.display = 'block';
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.enableZoom = true;
    controls.enableRotate = true;
    controls.enablePan = true;
    controlsRef.current = controls;

    // Geometry - match source resolution
    const geometry = new THREE.BufferGeometry();
    const maxPoints = 1280 * 720;
    const positions = new Float32Array(maxPoints * 3);
    const sizes = new Float32Array(maxPoints);

    // Initialize all sizes
    for (let i = 0; i < maxPoints; i++) {
      sizes[i] = 1.5; // Point size (smaller for cleaner look)
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('aSize', new THREE.BufferAttribute(sizes, 1));
    geometry.setDrawRange(0, 0);
    geometryRef.current = geometry;

    // Custom shader material for better-looking particles
    const material = new THREE.ShaderMaterial({
      uniforms: {},
      vertexShader,
      fragmentShader,
      transparent: true,
      depthWrite: false,
      blending: THREE.AdditiveBlending,
    });
    materialRef.current = material;

    const pointsObj = new THREE.Points(geometry, material);
    scene.add(pointsObj);

    // Visual Helpers
    const gridHelper = new THREE.GridHelper(10, 20, 0x303040, 0x202030);
    gridHelper.rotation.x = Math.PI / 2; // Rotate to X-Y plane (Z-up)
    scene.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);

    // Animation loop
    const animate = () => {
      animationIdRef.current = requestAnimationFrame(animate);
      if (controlsRef.current) {
        controlsRef.current.update();
      }
      if (rendererRef.current && sceneRef.current && cameraRef.current) {
        rendererRef.current.render(sceneRef.current, cameraRef.current);
      }
    };
    animate();

    const handleResize = () => {
      if (!containerRef.current || !cameraRef.current || !rendererRef.current) return;
      const w = containerRef.current.clientWidth;
      const h = containerRef.current.clientHeight;
      cameraRef.current.aspect = w / h;
      cameraRef.current.updateProjectionMatrix();
      rendererRef.current.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    // Cleanup function
    return () => {
      if (animationIdRef.current !== null) {
        cancelAnimationFrame(animationIdRef.current);
        animationIdRef.current = null;
      }
      window.removeEventListener('resize', handleResize);
      if (controlsRef.current) {
        controlsRef.current.dispose();
        controlsRef.current = null;
      }
      if (rendererRef.current) {
        if (container && container.contains(rendererRef.current.domElement)) {
          container.removeChild(rendererRef.current.domElement);
        }
        rendererRef.current.dispose();
        rendererRef.current = null;
      }
      if (geometryRef.current) {
        geometryRef.current.dispose();
        geometryRef.current = null;
      }
      if (materialRef.current) {
        materialRef.current.dispose();
        materialRef.current = null;
      }
      sceneRef.current = null;
      cameraRef.current = null;
      initAttemptedRef.current = false;
    };
  }, []);

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
    }
  }, [points]);

  return (
    <div className="relative w-full h-full bg-slate-950 rounded-lg overflow-hidden border-2 border-orange-500/50">
      <div className="absolute top-3 left-3 z-10 flex items-center gap-2 bg-slate-900/80 backdrop-blur-sm px-3 py-1.5 rounded-md">
        <Box className="w-4 h-4 text-orange-400" />
        <span className="text-xs text-white font-mono">POINT CLOUD</span>
        {isActive && (
          <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse" />
        )}
      </div>
      <div ref={containerRef} className="w-full h-full" style={{ pointerEvents: 'auto' }} />
    </div>
  );
}
