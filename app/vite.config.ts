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
    proxy: {
      '/api': {
        target: 'http://192.168.0.221',
        changeOrigin: true,
        ws: true
      }
    }
  }
})
