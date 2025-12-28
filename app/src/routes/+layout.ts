export const prerender = true
export const ssr = false

const registerFetchIntercept = async () => {
  const { fetch: originalFetch } = window
  const fileService = (await import('$lib/services/file-service')).default
  window.fetch = async (resource, config) => {
    const url = resource instanceof Request ? resource.url : resource.toString()

    let file = await fileService?.getFile(url)
    if (file?.isOk() && file.inner) return new Response(new Uint8Array(file.inner))

    if (url.startsWith('http')) {
      try {
        const urlObj = new URL(url)
        const pathOnly = urlObj.pathname
        file = await fileService?.getFile(pathOnly)
        if (file?.isOk() && file.inner) return new Response(new Uint8Array(file.inner))
      } catch (error) {
        console.error('Error fetching file:', error)
      }
    }

    return originalFetch(resource, config)
  }
}

export const load = async () => {
  await registerFetchIntercept()
  return {
    title: 'Hexapod controller',
    github: 'runeharlyk/Hexapod',
    app_name: 'Hexapod Controller',
    copyright: '2025 Rune Harlyk'
  }
}
