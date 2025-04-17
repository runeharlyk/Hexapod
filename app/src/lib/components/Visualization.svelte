<script lang="ts">
    import { onDestroy, onMount } from 'svelte';
    import { jointNames, model } from '$lib/stores';
    import { populateModelCache } from '$lib/utilities';
    import SceneBuilder from '$lib/sceneBuilder';
    import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
    import type { URDFRobot } from 'urdf-loader';
    import { degToRad, lerp } from 'three/src/math/MathUtils';
    import Kinematics, { gen_posture, type body_state_t, type HexapodConfig } from '$lib/kinematic';
    import { config } from './config';

    interface Props {
        sky?: boolean;
        orbit?: boolean;
        panel?: boolean;
        debug?: boolean;
        ground?: boolean;
    }

    let { sky = true, orbit = false, panel = true, debug = false, ground = true }: Props = $props();

    let sceneManager = $state(new SceneBuilder());
    let canvas: HTMLCanvasElement | null = $state(null);

    const angles = new Array(18).fill(0);
    const targetAngles = new Array(18).fill(0);

    const modelOffset = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    // [90, 90, 220, 90, 90, 220, 90, 90, 90, 90, 90, 90, 90, 90, 220, 90, 90, 220];
    const dir = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];

    const posture = gen_posture(60, 75, config);

    const kinematics = new Kinematics(config);

    let gui_panel: GUI;

    let settings = {
        'Internal kinematic': true,
        'Robot transform controls': false,
        'Auto orient robot': true,
        'Trace feet': debug,
        'Target position': false,
        'Trace points': 30,
        'Fix camera on robot': true,
        'Smooth motion': true,
        Background: 'black'
    };

    const _jointNames = [
        'front_left_shoulder',
        'front_left_leg',
        'front_left_foot',
        'middle_left_shoulder',
        'middle_left_leg',
        'middle_left_foot',
        'back_left_shoulder',
        'back_left_leg',
        'back_left_foot',
        'front_right_shoulder',
        'front_right_leg',
        'front_right_foot',
        'middle_right_shoulder',
        'middle_right_leg',
        'middle_right_foot',
        'back_right_shoulder',
        'back_right_leg',
        'back_right_foot'
    ];

    const jointAngles: Record<string, number> = _jointNames.reduce(
        (prev, cur) => ({ ...prev, [cur]: 0 }),
        {}
    );

    const kinematics_state: body_state_t = {
        omega: 0,
        phi: 0,
        psi: 0,
        xm: 0,
        ym: 0.7,
        zm: 0,
        feet: posture
    };

    onMount(async () => {
        await populateModelCache();
        await createScene();
        if (panel) createPanel();
    });

    onDestroy(() => {
        canvas?.remove();
        gui_panel?.destroy();
    });

    const createPanel = () => {
        gui_panel = new GUI({ width: 310 });
        gui_panel.close();
        gui_panel.domElement.id = 'three-gui-panel';

        const general = gui_panel.addFolder('General');
        general.add(settings, 'Internal kinematic');
        general.add(settings, 'Robot transform controls');
        general.add(settings, 'Auto orient robot');

        const kin = gui_panel.addFolder('Kinematics');
        for (const name of Object.keys(kinematics_state)) {
            if (name === 'feet') continue;
            kin.add(kinematics_state, name as any, -100, 100, 1)
                .onChange(updateInverseKinematics)
                .listen();
        }

        const limbs = gui_panel.addFolder('limbs');
        limbs.add({ center: resetLimbs }, 'center');
        for (const name of $jointNames) {
            limbs.add(jointAngles, name, -150, 150).onChange(updateLimbs).listen();
        }

        const visibility = gui_panel.addFolder('Visualization');
        visibility.add(settings, 'Trace feet');
        visibility.add(settings, 'Trace points', 1, 1000, 1);
        visibility.add(settings, 'Target position');
        visibility.add(settings, 'Smooth motion');
        visibility.addColor(settings, 'Background');
    };

    const resetLimbs = () => {
        for (const name of $jointNames) {
            jointAngles[name] = 0;
        }
        updateLimbs();
    };

    const updateLimbs = () => {
        for (let i = 0; i < $jointNames.length; i++) {
            targetAngles[i] = jointAngles[$jointNames[i]];
        }
    };

    const setTargetAngles = (angles: number[]) => {
        for (let i = 0; i < angles.length; i++) {
            targetAngles[i] = (angles[i] - modelOffset[i]) * dir[i];
            jointAngles[$jointNames[i]] = targetAngles[i];
        }
        console.log(angles);
    };

    const updateInverseKinematics = () => {
        const newAngles = inverseKinematics(posture, config).flat();
        setTargetAngles(newAngles);
    };

    updateInverseKinematics();
    function inverseKinematics(dest: number[][], config: HexapodConfig): number[][] {
        const mountX = config.legMountX;
        const mountY = config.legMountY;
        const rootJ1 = config.legRootToJoint1;
        const j1j2 = config.legJoint1ToJoint2;
        const j2j3 = config.legJoint2ToJoint3;
        const j3tip = config.legJoint3ToTip;
        const mountAngle = config.legMountAngle.map(a => (a * Math.PI) / 180);
        const mountPosition = Array.from({ length: 6 }, (_, i) => [mountX[i], mountY[i], 0]);

        const tempDest = dest.map((d, i) => d.map((v, j) => v - mountPosition[i][j]));

        const localDest = tempDest.map((t, i) => {
            const angle = mountAngle[i];
            return [
                t[0] * Math.cos(angle) + t[1] * Math.sin(angle),
                t[0] * Math.sin(angle) - t[1] * Math.cos(angle),
                t[2]
            ];
        });

        const angles = Array.from({ length: 6 }, () => [0, 0, 0]);

        for (let i = 0; i < 6; i++) {
            let x = localDest[i][0] - rootJ1;
            let y = localDest[i][1];
            angles[i][0] = (-Math.atan2(y, x) * 180) / Math.PI;

            x = Math.sqrt(x * x + y * y) - j1j2;
            y = localDest[i][2];
            const ar = Math.atan2(y, x);
            const lr2 = x * x + y * y;
            const lr = Math.sqrt(lr2);
            const a1 = Math.acos((lr2 + j2j3 * j2j3 - j3tip * j3tip) / (2 * j2j3 * lr));
            const a2 = Math.acos((lr2 - j2j3 * j2j3 + j3tip * j3tip) / (2 * j3tip * lr));

            angles[i][1] = ((ar + a1) * 180) / Math.PI;
            angles[i][2] = 90 - ((a1 + a2) * 180) / Math.PI - 90;
        }

        return angles;
    }

    const createScene = async () => {
        if (!canvas) return;
        sceneManager
            .addRenderer({ antialias: true, canvas, alpha: true })
            .addPerspectiveCamera({ x: -0.5, y: 0.5, z: 1 })
            .addOrbitControls(40, 50, orbit)
            .addDirectionalLight({ x: 10, y: 20, z: 10, color: 0xffffff, intensity: 3 })
            .addAmbientLight({ color: 0xffffff, intensity: 0.5 })
            .addFogExp2(0xcccccc, 0.015)
            .addModel($model)
            .fillParent()
            .addRenderCb(render)
            .startRenderLoop();

        if (ground) sceneManager.addGroundPlane();

        if (sky) sceneManager.addSky();
    };

    const smooth = (start: number, end: number, amount: number) => {
        return settings['Smooth motion'] ? lerp(start, end, amount) : end;
    };

    const update_camera = (robot: URDFRobot) => {
        if (!settings['Fix camera on robot']) return;
        sceneManager.orbit.target = robot.position.clone();
    };

    const render = () => {
        const robot = sceneManager.model;
        if (!robot) return;
        update_camera(robot);

        for (let i = 0; i < $jointNames.length; i++) {
            angles[i] = smooth(
                (robot.joints[$jointNames[i]].angle as number) * (180 / Math.PI),
                targetAngles[i],
                0.1
            );
            robot.joints[$jointNames[i]].setJointValue(degToRad(angles[i]));
        }
    };
</script>

<svelte:window onresize={sceneManager.fillParent} />

<canvas bind:this={canvas}></canvas>
