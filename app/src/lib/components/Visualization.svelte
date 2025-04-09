<script lang="ts">
    import { onDestroy, onMount } from 'svelte';
    import { model } from '$lib/stores';
    import { populateModelCache } from '$lib/utilities';
    import SceneBuilder from '$lib/sceneBuilder';
    import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
    import type { URDFRobot } from 'urdf-loader';

    interface Props {
        sky?: boolean;
        orbit?: boolean;
        panel?: boolean;
        debug?: boolean;
        ground?: boolean;
    }

    let { sky = true, orbit = false, panel = true, debug = false, ground = true }: Props = $props();

    let sceneManager = $state(new SceneBuilder());
    let canvas: HTMLCanvasElement = $state();

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
        omega: 0,
        phi: 0,
        psi: 0,
        xm: 0,
        ym: 0.7,
        zm: 0,
        Background: 'black'
    };

    onMount(async () => {
        await populateModelCache();
        await createScene();
        if (panel) createPanel();
    });

    onDestroy(() => {
        canvas.remove();
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

        const visibility = gui_panel.addFolder('Visualization');
        visibility.add(settings, 'Trace feet');
        visibility.add(settings, 'Trace points', 1, 1000, 1);
        visibility.add(settings, 'Target position');
        visibility.add(settings, 'Smooth motion');
        visibility.addColor(settings, 'Background');
    };

    const createScene = async () => {
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

    const update_camera = (robot: URDFRobot) => {
        if (!settings['Fix camera on robot']) return;
        sceneManager.orbit.target = robot.position.clone();
    };

    const render = () => {
        const robot = sceneManager.model;
        if (!robot) return;
        update_camera(robot);
    };
</script>

<svelte:window onresize={sceneManager.fillParent} />

<canvas bind:this={canvas}></canvas>
