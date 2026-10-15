import adapter from '@sveltejs/adapter-static'
import { vitePreprocess } from '@sveltejs/vite-plugin-svelte'

const basePath = process.env.BASE_PATH ?? ''
// The robot serves the embedded build over plain http, where a service worker has nothing to add
// and would only pin the robot's own pages to a stale cache.
const servedFromRobot = process.env.VITE_USE_HOST_NAME === 'true'

/** @type {import('@sveltejs/kit').Config} */
const config = {
  preprocess: vitePreprocess(),

  kit: {
    adapter: adapter({
      pages: 'build',
      assets: 'build',
      fallback: 'index.html',
      precompress: false,
      strict: true
    }),
    output: {
      // The firmware serves the embedded build, where one bundle keeps the file list short; the
      // hosted app splits so each page loads its code when it is first opened.
      bundleStrategy: process.env.VITE_USE_HOST_NAME === 'true' ? 'single' : 'split'
    },
    paths: {
      base: basePath
    },
    serviceWorker: {
      register: !servedFromRobot
    }
  }
}

export default config
