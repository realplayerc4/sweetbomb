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

// Shader for circular particles with green BEV slice highlight
const vertexShader = `
  attribute float aSize;
  uniform float uTargetHeight;
  uniform float uTolerance;
  uniform float uCameraHeight;

  varying vec2 vUv;
  varying float vOpacity;
  varying float vPointType; // 0.0 for target (green), 1.0 for camera (blue)

  void main() {
    vUv = uv;
    
    // Discard points outside 0-3m forward range (car front bounds)
    if (position.x < 0.0 || position.x > 3.0) {
      gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
      vOpacity = 0.0;
      return;
    }

    float realHeight = position.z;
    float diffTarget = abs(realHeight - uTargetHeight);
    float diffCamera = abs(realHeight - uCameraHeight);
    
    if (diffTarget > uTolerance && diffCamera > uTolerance) {
      gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // Move out of view
      vOpacity = 0.0;
    } else {
      vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
      gl_Position = projectionMatrix * mvPosition;
      // Fixed size for orthographic lock
      gl_PointSize = aSize * 8.0;
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
    if(vOpacity < 0.5) discard;

    // Calculate distance from center of point
    vec2 center = gl_PointCoord - vec2(0.5);
    float dist = length(center);

    // Discard pixels outside circle
    if (dist > 0.5) discard;

    vec3 fillColor;
    vec3 borderColor;
    
    if (vPointType > 0.5) {
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

    // Soft edge
    float alpha = 1.0 - smoothstep(0.45, 0.5, dist);

    gl_FragColor = vec4(color, alpha);
  }
`;

export function BEVSliceView({ isActive, points, targetHeight, tolerance, cameraHeight }: BEVSliceViewProps) {
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
        scene.background = new THREE.Color('#403D39');
        sceneRef.current = scene;

        // Strictly Locked Orthographic Bird's Eye View
        const aspect = width / height;
        const viewSizeX = 3.0; // The screen height will exactly map to 3 meters in X

        const camera = new THREE.OrthographicCamera(
            -viewSizeX * aspect / 2, // Left
            viewSizeX * aspect / 2, // Right
            viewSizeX / 2,          // Top
            -viewSizeX / 2,          // Bottom
            0.1,
            100
        );

        // Set X+ direction as screen "North" (Forward/Car Head). 
        // This requires `up` vector to be (1, 0, 0) since Z is vertical up.
        camera.up.set(1, 0, 0);

        // We only show X from 0 to 3m. Absolute center of our X field is 1.5.
        camera.position.set(1.5, 0, 10);
        camera.lookAt(1.5, 0, 0);

        cameraRef.current = camera;

        const renderer = new THREE.WebGLRenderer({ antialias: false });
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

        // Visual Helpers (same as PointCloud)
        const gridHelper = new THREE.GridHelper(10, 20, 0x303040, 0x202030);
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
            const currentAspect = w / h;

            const cam = cameraRef.current as THREE.OrthographicCamera;
            cam.left = -viewSizeX * currentAspect / 2;
            cam.right = viewSizeX * currentAspect / 2;
            cam.top = viewSizeX / 2;
            cam.bottom = -viewSizeX / 2;
            cam.updateProjectionMatrix();
            rendererRef.current.setSize(w, h);
        };
        window.addEventListener('resize', handleResize);

        // Cleanup
        return () => {
            if (animationIdRef.current !== null) {
                cancelAnimationFrame(animationIdRef.current);
                animationIdRef.current = null;
            }
            window.removeEventListener('resize', handleResize);
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
        <div className="relative w-full h-full bg-[#403D39] rounded-lg overflow-hidden border-2 border-[#FD802E]">
            <div className="absolute top-3 left-3 z-10 flex flex-col gap-1">
                <div className="flex items-center gap-2 bg-slate-900/80 backdrop-blur-sm px-3 py-1.5 rounded-md">
                    <Layers className="w-4 h-4 text-green-400" />
                    <span className="text-xs font-mono font-bold text-green-400">
                        TGT SLICE ({targetHeight.toFixed(2)}m)
                    </span>
                    {isActive && (
                        <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
                    )}
                </div>
                <div className="flex items-center gap-2 bg-slate-900/80 backdrop-blur-sm px-3 py-1.5 rounded-md">
                    <Layers className="w-4 h-4 text-blue-400" />
                    <span className="text-xs font-mono font-bold text-blue-400">
                        CAM SLICE ({cameraHeight.toFixed(2)}m)
                    </span>
                </div>
            </div>
            <div ref={containerRef} className="w-full h-full" style={{ pointerEvents: 'auto' }} />
        </div>
    );
}
