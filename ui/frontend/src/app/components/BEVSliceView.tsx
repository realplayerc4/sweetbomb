import { useEffect, useRef } from 'react';
import { Layers } from 'lucide-react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

interface BEVSliceViewProps {
    isActive: boolean;
    points: Float32Array | null;
    targetHeight: number;
    tolerance: number;
    cameraHeight: number;
}

// Shader for circular particles with background context mapping
const vertexShader = `
  attribute float aSize;
  uniform float uTargetHeight;
  uniform float uTolerance;
  uniform float uCameraHeight;

  varying vec2 vUv;
  varying float vOpacity;
  varying float vPointType; // 0.0: Target(Green), 1.0: Camera(Blue), 2.0: Background

  void main() {
    vUv = uv;
    
    // In 3D Slice View, we don't clip forward ranges to allow zooming in/out freely
    // Just clip camera noise at the origin (radius < 0.03m)
    if (length(position.xy) < 0.03) {
      gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
      vOpacity = 0.0;
      return;
    }

    float realHeight = position.z;
    float diffTarget = abs(realHeight - uTargetHeight);
    float diffCamera = abs(realHeight - uCameraHeight);
    
    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mvPosition;

    // Use absolute realHeight (post-compensation) to determine slicing coloring
    if (diffTarget > uTolerance && diffCamera > uTolerance) {
      // Points outside of target slice height: dim background markers
      vPointType = 2.0; 
      vOpacity = 0.20;
      gl_PointSize = aSize * 4.0;
    } else {
      // Highlighted specific slices inside Tolerance range
      gl_PointSize = aSize * 10.0;
      vOpacity = 1.0;
      
      if (diffCamera <= uTolerance) {
         vPointType = 1.0; // blue
      } else {
         vPointType = 0.0; // green
      }
    }

    // Perspective point size attenuation (matches PointCloudView)
    gl_PointSize = gl_PointSize * (3.0 / -mvPosition.z);
  }
`;

const fragmentShader = `
  varying vec2 vUv;
  varying float vOpacity;
  varying float vPointType;

  void main() {
    if(vOpacity < 0.05) discard;

    // Calculate distance from center of point
    vec2 center = gl_PointCoord - vec2(0.5);
    float dist = length(center);

    // Discard pixels outside circle
    if (dist > 0.5) discard;

    vec3 fillColor;
    vec3 borderColor;
    
    if (vPointType > 1.5) {
       // Background points
       fillColor = vec3(0.4, 0.4, 0.4);
       borderColor = vec3(0.2, 0.2, 0.2);
    } else if (vPointType > 0.5) {
       // Camera height slice -> Blue
       fillColor = vec3(0.0, 0.5, 1.0);
       borderColor = vec3(0.0, 0.1, 0.5);
    } else {
       // Target height slice -> Green
       fillColor = vec3(0.0, 1.0, 0.0);
       borderColor = vec3(0.0, 0.3, 0.0);
    }

    // Border region (outer ring)
    float t = smoothstep(0.3, 0.45, dist);
    vec3 color = mix(fillColor, borderColor, t);

    // Soft edge combined with base opacity
    float alpha = vOpacity * (1.0 - smoothstep(0.45, 0.5, dist));

    gl_FragColor = vec4(color, alpha);
  }
`;

export function BEVSliceView({ points, targetHeight, tolerance, cameraHeight }: BEVSliceViewProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
    const geometryRef = useRef<THREE.BufferGeometry | null>(null);
    const materialRef = useRef<THREE.ShaderMaterial | null>(null);
    const sceneRef = useRef<THREE.Scene | null>(null);
    const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
    const animationIdRef = useRef<number | null>(null);
    const initAttemptedRef = useRef(false);

    useEffect(() => {
        const container = containerRef.current;
        if (!container) return;

        let width = container.clientWidth || 300;
        let height = container.clientHeight || 300;

        // Scene setup - Transparent to let Cyberpunk parent CSS shine through
        const scene = new THREE.Scene();
        sceneRef.current = scene;

        // 3D Perspective Camera (Matching PointCloudView coordinates)
        const camera = new THREE.PerspectiveCamera(60, width / height, 0.01, 1000);
        camera.up.set(0, 0, 1);        // Z axis points up
        camera.position.set(-5, 0, 3); // Start slightly behind and above, looking forward over slice
        camera.lookAt(0, 0, 0);

        cameraRef.current = camera;

        const renderer = new THREE.WebGLRenderer({ antialias: false, alpha: true });
        renderer.setClearColor(0x000000, 0); // Allow glassmorphism
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
        renderer.setSize(width, height);
        renderer.domElement.style.display = 'block';
        container.appendChild(renderer.domElement);
        rendererRef.current = renderer;

        // Geometry - match source resolution
        const geometry = new THREE.BufferGeometry();
        const maxPoints = 1280 * 720;
        const positions = new Float32Array(maxPoints * 3);
        const sizes = new Float32Array(maxPoints);

        for (let i = 0; i < maxPoints; i++) {
            sizes[i] = 4.5; // Match PointCloud particle size for consistency in 3D
        }

        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('aSize', new THREE.BufferAttribute(sizes, 1));
        geometry.setDrawRange(0, 0);
        geometryRef.current = geometry;

        // Custom shader material with Uniforms
        const material = new THREE.ShaderMaterial({
            uniforms: {
                uTargetHeight: { value: targetHeight },
                uTolerance: { value: tolerance },
                uCameraHeight: { value: cameraHeight }
            },
            vertexShader,
            fragmentShader,
            transparent: true,
            depthWrite: false,
            blending: THREE.NormalBlending,
        });
        materialRef.current = material;

        const pointsObj = new THREE.Points(geometry, material);
        pointsObj.frustumCulled = false; // Need this since points teleport out of frame in shader
        scene.add(pointsObj);

        // Visual Helpers - Reduced opacity to fit cyberpunk neon grid vibe
        const gridHelper = new THREE.GridHelper(10, 20, 0x444455, 0x222233);
        gridHelper.material.transparent = true;
        gridHelper.material.opacity = 0.5;
        gridHelper.rotation.x = Math.PI / 2; // Rotate to X-Y plane (Z-up)
        scene.add(gridHelper);

        const axesHelper = new THREE.AxesHelper(1);
        scene.add(axesHelper);

        // Controls
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.enableZoom = true;
        controls.enableRotate = true;
        controls.enablePan = true;
        controls.target.set(0, 0, 0); // Orbit center

        // Animation loop
        const animate = () => {
            animationIdRef.current = requestAnimationFrame(animate);
            controls.update();
            if (rendererRef.current && sceneRef.current && cameraRef.current) {
                rendererRef.current.render(sceneRef.current, cameraRef.current);
            }
        };
        animate();

        const handleResize = () => {
            if (!containerRef.current || !cameraRef.current || !rendererRef.current) return;
            const w = containerRef.current.clientWidth;
            const h = containerRef.current.clientHeight;
            if (w === 0 || h === 0) return;

            const cam = cameraRef.current as THREE.PerspectiveCamera;
            cam.aspect = w / h;
            cam.updateProjectionMatrix();
            rendererRef.current.setSize(w, h);
        };
        // Use ResizeObserver for more robust resizing over simple window 'resize' event
        const resizeObserver = new ResizeObserver(handleResize);
        resizeObserver.observe(container);

        // Cleanup
        return () => {
            if (animationIdRef.current !== null) {
                cancelAnimationFrame(animationIdRef.current);
                animationIdRef.current = null;
            }
            resizeObserver.disconnect();
            if (rendererRef.current && containerRef.current) {
                containerRef.current.innerHTML = '';
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

    // Update points
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

    // Update Shader Uniforms
    useEffect(() => {
        if (materialRef.current) {
            materialRef.current.uniforms.uTargetHeight.value = targetHeight;
            materialRef.current.uniforms.uTolerance.value = tolerance;
            materialRef.current.uniforms.uCameraHeight.value = cameraHeight;
        }
    }, [targetHeight, tolerance, cameraHeight]);


    return (
        <div className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden shadow-md group">
            {/* 悬浮状态胶囊 */}
            <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex gap-4">
                <div className="flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
                    <Layers className="w-3.5 h-3.5 text-[#FD802E]" />
                    <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">
                        TGT SLICE
                    </span>
                    <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-3 ml-2 mr-6 font-mono font-bold">
                        | {targetHeight.toFixed(2)}m
                    </span>
                    <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">
                        CAM SLICE
                    </span>
                    <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
                        | {cameraHeight.toFixed(2)}m
                    </span>
                </div>
            </div>

            {/* Canvas Container */}
            <div ref={containerRef} className="absolute inset-0 z-0" style={{ pointerEvents: 'auto' }} />
        </div>
    );
}
