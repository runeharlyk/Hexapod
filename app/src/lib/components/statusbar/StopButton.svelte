<script lang="ts">
    import { MotionModes } from '$lib/motion';
    import { mode } from '$lib/stores';

    const deactivate = async () => {
        mode.set(MotionModes.DEACTIVATED);
    };
</script>

<button onclick={deactivate} class="bg-error text-white btn rounded-none">STOP</button>
