import { defineConfig } from 'vitest/config'
import { sveltekit } from '@sveltejs/kit/vite'
import EnvCaster from '@niku/vite-env-caster'

export default defineConfig({
  plugins: [sveltekit(), EnvCaster()],
  test: {
    globals: true,
    environment: 'jsdom',
    include: ['tests/unit/**/*.spec.ts']
  }
})
