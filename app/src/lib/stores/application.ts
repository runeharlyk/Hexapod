import { persistentStore } from '$lib/utilities'
import { get, type Writable } from 'svelte/store'

export interface WidgetConfig {
  id: string | number
  component: 'Visualization' | 'Stream'
}

export interface WidgetContainerConfig {
  id: string | number
  layout?: 'row' | 'column' | 'wrap'
  header?: string
  widgets: Array<WidgetConfig | WidgetContainerConfig>
}

export const isWidgetConfig = (
  widget: WidgetConfig | WidgetContainerConfig
): widget is WidgetConfig => 'component' in widget

interface View {
  name: string
  content: WidgetContainerConfig
}

const defaultViews: View[] = [
  {
    name: 'Stream',
    content: {
      id: 'root',
      layout: 'column',
      widgets: [{ id: 2, component: 'Stream' }]
    }
  },
  {
    name: '3D representation',
    content: {
      id: 'root',
      layout: 'column',
      widgets: [{ id: 2, component: 'Visualization' }]
    }
  },
  {
    name: 'Split screen',
    content: {
      id: 'root',
      widgets: [
        { id: 2, component: 'Stream' },
        { id: 2, component: 'Visualization' }
      ]
    }
  }
]

// Bump the version whenever defaultViews changes shape, so stored layouts do not shadow it.
const VIEWS_KEY = 'views_v2'

export const views: Writable<View[]> = persistentStore(VIEWS_KEY, defaultViews)

export const selectedView = persistentStore('selected_view', get(views)[0].name)
