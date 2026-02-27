import { useEffect, useRef } from 'react';
import { Layers } from 'lucide-react';
import * as THREE from 'three';

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
    
    // Discard points completely outside 1-3m forward range (X is forward in 73 mapping)
    if (position.x < 1.0 || position.x > 3.0) {
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
    const cameraRef = useRef<THREE.OrthographicCamera | null>(null);
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

        // Strictly Locked Orthographic Bird's Eye View
        const aspect = width / height;
        const viewSizeX = 2.0; // Show a 2m window (from X=1m to X=3m)

        // We want Z (Upwards) to be squashed orthographically into view.
        // So we just look top-down. 
        // Setting Near to negative and Far to positive so cutting planes don't clip the cloud
        const camera = new THREE.OrthographicCamera(
            -viewSizeX * aspect / 2, // Left
            viewSizeX * aspect / 2, // Right
            viewSizeX / 2,          // Top
            -viewSizeX / 2,          // Bottom
            -20, // Negative near plane to capture everything under camera unconditionally
            20
        );

        // Set X+ direction as screen "North" (Forward/Car Head). 
        // This requires `up` vector to be (1, 0, 0) since Z is vertical up.
        camera.up.set(1, 0, 0);

        // Pointing straight down from Z=10 to origin, with North facing +X.
        camera.position.set(2.0, 0, 10);
        camera.lookAt(2.0, 0, 0);

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
            sizes[i] = 1.5; // same size as PointCloud
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

        // Animation loop
        const animate = () => {
            animationIdRef.current = requestAnimationFrame(animate);
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
            const currentAspect = w / h;

            const cam = cameraRef.current as THREE.OrthographicCamera;
            cam.left = -viewSizeX * currentAspect / 2;
            cam.right = viewSizeX * currentAspect / 2;
            cam.top = viewSizeX / 2;
            cam.bottom = -viewSizeX / 2;
            cam.updateProjectionMatrix();
            rendererRef.current.setSize(w, h);
        };

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
        <div className="relative w-full h-full bg-[#1c1c1e] rounded-2xl overflow-hidden border border-white/5 shadow-md group">
            <div className="absolute top-4 left-4 z-10 flex flex-col gap-2">
                <div className="flex items-center gap-1.5 bg-[#2c2c2e]/90 border border-white/10 backdrop-blur-md px-2 py-1 rounded shadow-sm">
                    <Layers className="w-3.5 h-3.5 text-green-500" />
                    <span className="text-[10px] text-green-400 font-semibold tracking-wider uppercase font-mono">
                        TGT SLICE ({targetHeight.toFixed(2)}m)
                    </span>
                </div>
                <div className="flex items-center gap-1.5 bg-[#2c2c2e]/90 border border-white/10 backdrop-blur-md px-2 py-1 rounded shadow-sm">
                    <Layers className="w-3.5 h-3.5 text-blue-500" />
                    <span className="text-[10px] text-blue-400 font-semibold tracking-wider uppercase font-mono">
                        CAM SLICE ({cameraHeight.toFixed(2)}m)
                    </span>
                </div>
            </div>

            {/* Canvas Container */}
            <div ref={containerRef} className="absolute inset-0 z-0" style={{ pointerEvents: 'auto' }} />

            {/* Grid Distance Rulers */}
            <div className="absolute right-0 top-0 bottom-0 w-12 pointer-events-none z-10 opacity-70">
                <div className="absolute top-[0%] right-2 text-[11px] font-mono font-bold text-slate-400 translate-y-1">3.0m -</div>
                <div className="absolute top-[25%] right-2 text-[11px] font-mono font-bold text-slate-400 -translate-y-1/2">2.5m -</div>
                <div className="absolute top-[50%] right-2 text-[11px] font-mono font-bold text-slate-400 -translate-y-1/2">2.0m -</div>
                <div className="absolute top-[75%] right-2 text-[11px] font-mono font-bold text-slate-400 -translate-y-1/2">1.5m -</div>
                <div className="absolute top-[100%] right-2 text-[11px] font-mono font-bold text-slate-400 -translate-y-[calc(100%+4px)]">1.0m -</div>
            </div>
        </div>
    );
}
