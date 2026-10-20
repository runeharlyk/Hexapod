import { sveltekit } from '@sveltejs/kit/vite'
import { defineConfig } from 'vite'
import Icons from 'unplugin-icons/vite'
import viteLittleFS from './vite-plugin-littlefs'
import EnvCaster from '@niku/vite-env-caster'
import tailwindcss from '@tailwindcss/vite'

const basePath = process.env.BASE_PATH ?? ''

export default defineConfig({
  base: basePath,
  plugins: [
    tailwindcss(),
    sveltekit(),
    Icons({
      compiler: 'svelte'
    }),
    viteLittleFS(),
    EnvCaster()
  ],
  server: {
    // The root animations/ directory is imported by src/lib/animation/library.ts, and the simulation's MuJoCo
    // model by src/lib/simulation/load.ts.
    fs: { allow: ['../animations', '../simulation/src/resources'] },
    proxy: {
      '/api': {
        target: 'http://192.168.0.221',
        changeOrigin: true,
        ws: true
      }
    }
  },
  // A preview server inherits server.proxy by default; the built app is served by it for the integration tests, which
  // must reach no address of a real network, where a robot that does not answer holds each request for a minute.
  preview: { proxy: {} }
})
