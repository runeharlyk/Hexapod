import { type AnalyticsData } from '$lib/platform_shared/message'
import { writable } from 'svelte/store'

let analytics_data = {
  uptime: <number[]>[],
  free_heap: <number[]>[],
  total_heap: <number[]>[],
  used_heap: <number[]>[],
  min_free_heap: <number[]>[],
  max_alloc_heap: <number[]>[],
  fs_used: <number[]>[],
  fs_total: <number[]>[],
  core_temp: <number[]>[],
  cpu0_usage: <number[]>[],
  cpu1_usage: <number[]>[],
  cpu_usage: <number[]>[]
}

const maxAnalyticsData = 100

function createAnalytics() {
  const { subscribe, update } = writable(analytics_data)

  return {
    subscribe,
    addData: (content: AnalyticsData) => {
      update(analytics_data => ({
        uptime: [...analytics_data.uptime, content.uptime].slice(-maxAnalyticsData),
        free_heap: [...analytics_data.free_heap, content.freeHeap / 1000].slice(-maxAnalyticsData),
        total_heap: [...analytics_data.total_heap, content.totalHeap / 1000].slice(
          -maxAnalyticsData
        ),
        used_heap: [
          ...analytics_data.used_heap,
          (content.totalHeap - content.freeHeap) / 1000
        ].slice(-maxAnalyticsData),
        min_free_heap: [...analytics_data.min_free_heap, content.minFreeHeap / 1000].slice(
          -maxAnalyticsData
        ),
        max_alloc_heap: [...analytics_data.max_alloc_heap, content.maxAllocHeap / 1000].slice(
          -maxAnalyticsData
        ),
        fs_used: [...analytics_data.fs_used, content.fsUsed / 1000].slice(-maxAnalyticsData),
        fs_total: [...analytics_data.fs_total, content.fsTotal / 1000].slice(-maxAnalyticsData),
        core_temp: [...analytics_data.core_temp, content.coreTemp].slice(-maxAnalyticsData),
        cpu0_usage: [...analytics_data.cpu0_usage, content.cpu0Usage].slice(-maxAnalyticsData),
        cpu1_usage: [...analytics_data.cpu1_usage, content.cpu1Usage].slice(-maxAnalyticsData),
        cpu_usage: [...analytics_data.cpu_usage, content.cpuUsage].slice(-maxAnalyticsData)
      }))
    }
  }
}

export const analytics = createAnalytics()
