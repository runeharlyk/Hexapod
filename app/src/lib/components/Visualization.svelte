<script lang="ts">
    import { onMount } from 'svelte';
    import * as THREE from 'three';
    import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

    let container: HTMLDivElement;
    let camera: THREE.PerspectiveCamera,
        scene: THREE.Scene,
        renderer: THREE.WebGLRenderer,
        controls: {
            enableDamping: boolean;
            dampingFactor: number;
            update: () => void;
            dispose: () => void;
        };
    let hexapod: { group: any; legs: any };

    // Joint angles for each leg (coxa, femur, tibia)
    let legAngles = Array(6)
        .fill()
        .map(() => ({ coxa: 0, femur: 0, tibia: 0 }));

    // Leg segment lengths
    const COXA_OFFSET = 20.75 / 10;
    const COXA_LENGTH = 28.0 / 10;
    const FEMUR_LENGTH = 42.6 / 10;
    const TIBIA_LENGTH = 89.07 / 10;

    const DEFAULT_COXA_ANGLE = [
        0,
        Math.PI / 3 + Math.PI,
        (2 * Math.PI) / 3,
        0,
        (4 * Math.PI) / 3,
        (5 * Math.PI) / 3 + Math.PI
    ];

    // Leg positions relative to body (radians)
    const LEG_POSITIONS = Array(6)
        .fill()
        .map((_, index) => ({
            angle: (index * Math.PI) / 3, // Divide circle into 6 equal parts (2π/6 = π/3)
            side: index < 3 ? 1 : -1 // Right side for first 3, left side for last 3
        }));

    function createLeg(
        position: { angle: any; side?: number },
        angles: { coxa: any; femur: any; tibia: any },
        index: number
    ) {
        const legGroup = new THREE.Group();

        // Coxa (first segment)
        const coxa = new THREE.Mesh(
            new THREE.BoxGeometry(COXA_LENGTH, 1, 1),
            new THREE.MeshPhongMaterial({ color: 0xff0000 })
        );
        coxa.position.x = COXA_LENGTH / 2;

        // Femur (second segment)
        const femur = new THREE.Mesh(
            new THREE.BoxGeometry(FEMUR_LENGTH, 1, 1),
            new THREE.MeshPhongMaterial({ color: 0x00ff00 })
        );
        femur.position.x = FEMUR_LENGTH / 2;

        // Tibia (third segment)
        const tibia = new THREE.Mesh(
            new THREE.BoxGeometry(TIBIA_LENGTH, 1, 1),
            new THREE.MeshPhongMaterial({ color: 0x0000ff })
        );
        tibia.position.x = TIBIA_LENGTH / 2;

        // Create joints with visible markers
        const coxaJoint = new THREE.Group();
        const femurJoint = new THREE.Group();
        const tibiaJoint = new THREE.Group();

        // Add joint markers
        const jointGeometry = new THREE.SphereGeometry(0.5);
        const jointMaterial = new THREE.MeshPhongMaterial({ color: 0xffff00 });

        const coxaMarker = new THREE.Mesh(jointGeometry, jointMaterial);
        const femurMarker = new THREE.Mesh(jointGeometry, jointMaterial);
        const tibiaMarker = new THREE.Mesh(jointGeometry, jointMaterial);

        coxaJoint.add(coxaMarker);
        femurJoint.add(femurMarker);
        tibiaJoint.add(tibiaMarker);

        // Build leg hierarchy
        coxaJoint.add(coxa);
        coxa.add(femurJoint);
        femurJoint.position.x = COXA_LENGTH;
        femurJoint.add(femur);
        femur.add(tibiaJoint);
        tibiaJoint.position.x = FEMUR_LENGTH;
        tibiaJoint.add(tibia);

        legGroup.add(coxaJoint);

        // Set initial joint angles
        coxaJoint.rotation.y = angles.coxa + DEFAULT_COXA_ANGLE[index];
        femurJoint.rotation.z = angles.femur;
        tibiaJoint.rotation.z = angles.tibia;

        // Position leg relative to body
        legGroup.position.x = Math.cos(position.angle) * 5;
        legGroup.position.z = Math.sin(position.angle) * 5;
        legGroup.rotation.y = position.angle;

        return {
            group: legGroup,
            joints: { coxaJoint, femurJoint, tibiaJoint }
        };
    }

    function createHexapod() {
        const body = new THREE.Mesh(
            new THREE.BoxGeometry(8, 2, 6),
            new THREE.MeshPhongMaterial({ color: 0x444444 })
        );

        const hexapodGroup = new THREE.Group();
        hexapodGroup.add(body);

        const legs = LEG_POSITIONS.map((pos, index) => {
            const leg = createLeg(pos, legAngles[index], index);
            hexapodGroup.add(leg.group);
            console.log(`Created leg ${index} at angle ${pos.angle}`);
            return leg;
        });

        // Move the hexapod up so it's not intersecting with the grid
        hexapodGroup.position.y = 4;

        return { group: hexapodGroup, legs };
    }

    function init() {
        scene = new THREE.Scene();
        console.log('Initializing Three.js scene');
        camera = new THREE.PerspectiveCamera(
            75,
            container.clientWidth / container.clientHeight,
            0.1,
            1000
        );
        camera.position.set(30, 20, 30);
        camera.lookAt(0, 0, 0);

        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setClearColor(0xf0f0f0);
        container.appendChild(renderer.domElement);

        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 10);
        scene.add(directionalLight);

        // Create grid
        const gridHelper = new THREE.GridHelper(20, 20);
        scene.add(gridHelper);

        // Create hexapod
        hexapod = createHexapod();
        scene.add(hexapod.group);

        animate();
    }

    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }

    function updateAllLegAngles(joint: string, value: number) {
        for (let i = 0; i < 6; i++) {
            updateLegAngles(i, joint, value);
        }
    }

    function updateLegAngles(legIndex: number, joint: string, value: number) {
        legAngles[legIndex][joint] = value;
        // Update the corresponding joint in the 3D model
        const leg = hexapod.legs[legIndex];
        switch (joint) {
            case 'coxa':
                leg.joints.coxaJoint.rotation.y = value + DEFAULT_COXA_ANGLE[legIndex];
                break;
            case 'femur':
                leg.joints.femurJoint.rotation.z = value;
                break;
            case 'tibia':
                leg.joints.tibiaJoint.rotation.z = value;
                break;
        }
    }

    onMount(() => {
        init();

        return () => {
            renderer?.dispose();
            controls?.dispose();
        };
    });
</script>

<div class="flex h-screen">
    <div class="w-1/4 p-4 bg-gray-100 overflow-y-auto">
        <div class="mb-6">
            <h3 class="font-bold mb-2">Angles</h3>
            {#each ['coxa', 'femur', 'tibia'] as joint}
                <div class="mb-2">
                    <label class="block text-sm font-medium text-gray-700">
                        <!-- {joint} ({((angle * 180) / Math.PI).toFixed(1)}°) -->
                    </label>
                    <input
                        type="range"
                        min={-Math.PI}
                        max={Math.PI}
                        step={0.01}
                        on:input={e => updateAllLegAngles(joint, parseFloat(e.target.value))}
                        class="w-full"
                    />
                </div>
            {/each}
        </div>

        <!-- {#each legAngles as leg, i}
            <div class="mb-6">
                <h3 class="font-bold mb-2">Leg {i + 1}</h3>
                {#each Object.entries(leg) as [joint, angle]}
                    <div class="mb-2">
                        <label class="block text-sm font-medium text-gray-700">
                            {joint} ({((angle * 180) / Math.PI).toFixed(1)}°)
                        </label>
                        <input
                            type="range"
                            min={-Math.PI}
                            max={Math.PI}
                            step={0.01}
                            value={angle}
                            on:input={e => updateLegAngles(i, joint, parseFloat(e.target.value))}
                            class="w-full"
                        />
                    </div>
                {/each}
            </div>
        {/each} -->
    </div>
    <div class="w-3/4" bind:this={container}></div>
</div>

<style>
    :global(body) {
        margin: 0;
        overflow: hidden;
    }
</style>
