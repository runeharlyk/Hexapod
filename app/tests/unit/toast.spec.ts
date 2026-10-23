// A notification may carry a link, which the toast shows as an anchor after its message.
import { describe, expect, it, vi } from 'vitest'
// The unit config resolves `svelte` to its server build, which cannot mount; the client build is imported by path.
// @ts-expect-error The client build ships no declarations; its types are those of `svelte`.
import * as client from '../../node_modules/svelte/src/index-client.js'
import { get } from 'svelte/store'
import Toast from '../../src/lib/components/toasts/Toast.svelte'
import { notifications } from '../../src/lib/components/toasts/notifications'

const { flushSync, mount, unmount } = client as typeof import('svelte')

// The icons are generated modules the unit config has no plugin for; the toast needs only something it can draw.
vi.mock('../../src/lib/components/icons', () => {
  const icon = () => {}
  return { error: icon, info: icon, success: icon, warning: icon }
})

describe('notifications with a link', () => {
  it('keeps the link on the notification', () => {
    const id = notifications.info('kept', 0, { href: 'http://a.local', label: 'a.local' })

    expect(get(notifications).find(n => n.id === id)?.link).toEqual({
      href: 'http://a.local',
      label: 'a.local'
    })
  })

  it('renders the link as an anchor, and a notification without one as the message alone', () => {
    notifications.info('plain', 0)
    notifications.info('moved', 0, { href: 'http://b.local', label: 'open b.local' })
    const toast = mount(Toast, { target: document.body })
    flushSync()

    const alerts = [...document.querySelectorAll('.alert')]
    const anchorOf = (text: string) =>
      alerts.find(a => a.textContent?.includes(text))?.querySelector('a')
    expect(anchorOf('moved')?.getAttribute('href')).toBe('http://b.local')
    expect(anchorOf('moved')?.textContent).toBe('open b.local')
    expect(alerts.some(a => a.textContent?.includes('plain'))).toBe(true)
    expect(anchorOf('plain')).toBeNull()
    unmount(toast)
  })
})
