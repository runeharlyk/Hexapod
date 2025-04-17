<script lang="ts">
    import { onDestroy, onMount } from 'svelte';
    import { jointNames, model } from '$lib/stores';
    import { populateModelCache } from '$lib/utilities';
    import SceneBuilder from '$lib/sceneBuilder';
    import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
    import type { URDFRobot } from 'urdf-loader';
    import { degToRad, lerp } from 'three/src/math/MathUtils';
    import Kinematics, { type body_state_t } from '$lib/kinematic';
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

    const kinematics = new Kinematics(config);

    const posture = kinematics.genPosture(60, 75);

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

    const body_state: body_state_t = {
        omega: 0,
        phi: 0,
        psi: 0,
        xm: 0,
        ym: 0,
        zm: -35,
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
        for (const name of ['omega', 'phi', 'psi']) {
            kin.add(body_state, name as any, -Math.PI / 10, Math.PI / 10, 0.01)
                .onChange(updateInverseKinematics)
                .listen();
        }
        kin.add(body_state, 'xm', -60, 60, 0.01).onChange(updateInverseKinematics).listen();
        kin.add(body_state, 'ym', -60, 60, 0.01).onChange(updateInverseKinematics).listen();
        kin.add(body_state, 'zm', -60, 60, 0.01).onChange(updateInverseKinematics).listen();

        const limbs = gui_panel.addFolder('limbs');
        limbs.add({ center: resetLimbs }, 'center');
        for (const name of $jointNames) {
            limbs
                .add(jointAngles, name, -Math.PI / 1.2, Math.PI / 1.2)
                .onChange(updateLimbs)
                .listen();
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
        const newAngles = kinematics.inverseKinematics(body_state).flat();
        const leg_order = [3, 0, 4, 1, 5, 2];
        const reorderedAngles = leg_order.flatMap(i => newAngles.slice(i * 3, i * 3 + 3));

        setTargetAngles(reorderedAngles);
    };

    updateInverseKinematics();

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

    const orient_robot = (robot: URDFRobot) => {
        if (settings['Robot transform controls'] || !settings['Auto orient robot']) return;

        robot.position.z = smooth(robot.position.z, -body_state.xm / 12, 0.1);
        robot.position.x = smooth(robot.position.x, -body_state.ym / 12, 0.1);

        robot.rotation.z = smooth(robot.rotation.z, -body_state.psi + Math.PI / 2, 0.1);
        robot.rotation.y = smooth(robot.rotation.y, body_state.omega, 0.1);
        robot.rotation.x = smooth(robot.rotation.x, -body_state.phi - Math.PI / 2, 0.1);
    };

    const render = () => {
        const robot = sceneManager.model;
        if (!robot) return;
        update_camera(robot);
        orient_robot(robot);

        for (let i = 0; i < $jointNames.length; i++) {
            angles[i] = smooth(robot.joints[$jointNames[i]].angle as number, targetAngles[i], 0.1);
            robot.joints[$jointNames[i]].setJointValue(angles[i]);
        }
    };
</script>

<svelte:window onresize={sceneManager.fillParent} />

<canvas bind:this={canvas}></canvas>
