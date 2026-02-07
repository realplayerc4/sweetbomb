/**
 * OctoMap 3D Viewer - Three.js Renderer
 * Renders MarkerArray messages as 3D voxels
 */

class OctomapViewer {
    constructor(container) {
        this.container = container;
        this.voxels = new THREE.Group();
        this.voxelCount = 0;
        this.showAxes = true;
        this.autoFocus = true;

        this.init();
    }

    init() {
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a0f);

        // Camera
        this.camera = new THREE.PerspectiveCamera(
            60,
            window.innerWidth / window.innerHeight,
            0.1,
            100
        );
        this.camera.up.set(0, 0, 1); // Set Z-up BEFORE lookAt
        this.camera.position.set(3, 2, 3);
        this.camera.lookAt(0, 0, 0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.container.appendChild(this.renderer.domElement);

        // Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.minDistance = 0.5;
        this.controls.maxDistance = 20;
        this.controls.update();

        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 5);
        this.scene.add(directionalLight);

        // Grid (on X-Y plane)
        const gridHelper = new THREE.GridHelper(10, 20, 0x303040, 0x202030);
        // Rotate grid to be on X-Y plane (perpendicular to Z)
        gridHelper.rotation.x = Math.PI / 2;
        this.scene.add(gridHelper);

        // Axes
        this.axesHelper = new THREE.AxesHelper(0.5);
        this.scene.add(this.axesHelper);

        // Voxels group
        this.scene.add(this.voxels);

        // Handle resize
        window.addEventListener('resize', () => this.onResize());

        // Start animation
        this.animate();
    }

    /**
     * Update voxels from MarkerArray message
     * @param {Object} markerArray - ROS MarkerArray message
     */
    updateFromMarkerArray(markerArray) {
        // Clear existing voxels
        this.clearVoxels();

        if (!markerArray.markers || markerArray.markers.length === 0) {
            console.log('Received empty MarkerArray');
            return;
        }

        console.log(`Received ${markerArray.markers.length} markers`);
        let totalVoxels = 0;
        let bounds = new THREE.Box3();

        for (const marker of markerArray.markers) {
            // Handle CUBE_LIST markers (type 6)
            if (marker.type === 6 && marker.points) {
                console.log(`Processing CUBE_LIST: ${marker.points.length} points, Scale: ${marker.scale.x}, ${marker.scale.y}, ${marker.scale.z}`);

                const geometry = new THREE.BoxGeometry(
                    marker.scale.x * 0.95,
                    marker.scale.y * 0.95,
                    marker.scale.z * 0.95
                );

                const instancedMesh = new THREE.InstancedMesh(
                    geometry,
                    new THREE.MeshBasicMaterial(),
                    marker.points.length
                );

                const matrix = new THREE.Matrix4();
                const color = new THREE.Color();

                for (let i = 0; i < marker.points.length; i++) {
                    const p = marker.points[i];
                    matrix.setPosition(p.x, p.y, p.z);
                    instancedMesh.setMatrixAt(i, matrix);

                    // Update bounds
                    bounds.expandByPoint(new THREE.Vector3(p.x, p.y, p.z));

                    // Always force Orange color, ignoring marker colors
                    color.setHex(0xFFA500);
                    instancedMesh.setColorAt(i, color);
                }

                instancedMesh.instanceMatrix.needsUpdate = true;
                if (instancedMesh.instanceColor) {
                    instancedMesh.instanceColor.needsUpdate = true;
                }

                // Add wireframe edges (merged geometry for performance)
                const pointCount = marker.points.length;
                const edgeVertices = [];
                const sx = marker.scale.x * 0.95 / 2;
                const sy = marker.scale.y * 0.95 / 2;
                const sz = marker.scale.z * 0.95 / 2;

                // Relative coordinates for a cube's 12 edges (24 vertices)
                const relC = [
                    [-sx, -sy, -sz], [sx, -sy, -sz],
                    [sx, -sy, -sz], [sx, sy, -sz],
                    [sx, sy, -sz], [-sx, sy, -sz],
                    [-sx, sy, -sz], [-sx, -sy, -sz],
                    [-sx, -sy, sz], [sx, -sy, sz],
                    [sx, -sy, sz], [sx, sy, sz],
                    [sx, sy, sz], [-sx, sy, sz],
                    [-sx, sy, sz], [-sx, -sy, sz],
                    [-sx, -sy, -sz], [-sx, -sy, sz],
                    [sx, -sy, -sz], [sx, -sy, sz],
                    [sx, sy, -sz], [sx, sy, sz],
                    [-sx, sy, -sz], [-sx, sy, sz]
                ];

                for (let i = 0; i < pointCount; i++) {
                    const p = marker.points[i];
                    for (let j = 0; j < 24; j++) {
                        edgeVertices.push(p.x + relC[j][0]);
                        edgeVertices.push(p.y + relC[j][1]);
                        edgeVertices.push(p.z + relC[j][2]);
                    }
                }

                const edgesGeo = new THREE.BufferGeometry();
                edgesGeo.setAttribute('position', new THREE.Float32BufferAttribute(edgeVertices, 3));
                const edgesMat = new THREE.LineBasicMaterial({ color: 0x000000 });
                const edgesObj = new THREE.LineSegments(edgesGeo, edgesMat);

                this.voxels.add(instancedMesh);
                this.voxels.add(edgesObj);
                totalVoxels += pointCount;
            }
            // Handle single CUBE markers (type 1)
            else if (marker.type === 1) {
                const geometry = new THREE.BoxGeometry(
                    marker.scale.x * 0.95,
                    marker.scale.y * 0.95,
                    marker.scale.z * 0.95
                );
                const material = new THREE.MeshBasicMaterial({
                    color: 0xFFA500,
                    opacity: marker.color.a,
                    transparent: marker.color.a < 1
                });
                const mesh = new THREE.Mesh(geometry, material);
                mesh.position.set(
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                );
                this.voxels.add(mesh);

                // Add wireframe edge for single cube
                const edges = new THREE.EdgesGeometry(geometry);
                const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x000000 }));
                line.position.copy(mesh.position);
                this.voxels.add(line);

                totalVoxels++;
                bounds.expandByPoint(new THREE.Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
            }
        }

        this.voxelCount = totalVoxels;
        console.log(`Total voxels rendered: ${totalVoxels}`);

        // Auto-center camera if bounds are valid and first real load
        if (!bounds.isEmpty() && this.voxelCount > 0 && this.autoFocus) {
            const center = new THREE.Vector3();
            bounds.getCenter(center);
            // Zoom out to fit
            const size = new THREE.Vector3();
            bounds.getSize(size);
            const maxDim = Math.max(size.x, size.y, size.z);
            const fov = this.camera.fov * (Math.PI / 180);
            let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
            cameraZ *= 1.5; // Zoom out a bit
            if (cameraZ < 2) cameraZ = 2; // Min distance

            this.camera.position.set(center.x + cameraZ, center.y + cameraZ, center.z + cameraZ);
            this.camera.lookAt(center);
            this.controls.target.copy(center);
            this.controls.update();

            this.autoFocus = false; // Only once
            console.log('Auto-focused camera on:', center, 'Size:', size);
        }
    }

    clearVoxels() {
        while (this.voxels.children.length > 0) {
            const child = this.voxels.children[0];
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
                if (Array.isArray(child.material)) {
                    child.material.forEach(m => m.dispose());
                } else {
                    child.material.dispose();
                }
            }
            this.voxels.remove(child);
        }
        this.voxelCount = 0;
    }

    resetView() {
        // Look towards positive X-axis (from behind and above)
        // Position: X=-3 (closer), Y=0 (centered), Z=2 (lower height)
        this.camera.position.set(-3, 0, 2);
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt(5, 0, 0);
        this.controls.target.set(3, 0, 0);
        this.controls.update();
    }

    setGridHeight(z) {
        // Grid is on X-Y plane, so we just change its position.z (which is actually up/down in ROS coords?)
        // Wait, Three.js Y-up vs Z-up. 
        // We rotated grid by 90 deg around X.
        // The original Y axis became Z axis.
        // So we should translate along the NEW Z axis?
        // gridHelper.position is in world coordinates.
        // Since scene is Y-up by default but we treat Z-up...
        // Camera up is (0,0,1).
        // So we should move gridHelper.position.z

        // Find gridHelper in scene
        const gridHelper = this.scene.children.find(c => c.type === 'GridHelper');
        if (gridHelper) {
            gridHelper.position.z = parseFloat(z);
        }
    }

    toggleAxes() {
        this.showAxes = !this.showAxes;
        this.axesHelper.visible = this.showAxes;
    }

    onResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    getVoxelCount() {
        return this.voxelCount;
    }
}
