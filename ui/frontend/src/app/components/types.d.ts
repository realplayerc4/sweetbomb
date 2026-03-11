declare module 'three/examples/jsm/controls/OrbitControls' {
  import { Camera, EventListener, Object3D } from 'three';

  export default class OrbitControls {
    constructor(camera: Camera, domElement: HTMLElement);

    enableDamping: boolean;
    dampingFactor: number;
    enableZoom: boolean;
    enableRotate: boolean;
    enablePan: boolean;

    update(): void;
    dispose(): void;
  }
}
