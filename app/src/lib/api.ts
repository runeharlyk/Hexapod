import { Err, Ok, type Result } from './utilities'
import { resolveUrl } from './proto-api'

export const api = {
  get: <TResponse>(endpoint: string, params?: RequestInit) =>
    sendRequest<TResponse>(endpoint, 'GET', undefined, params),
  post: <TResponse>(endpoint: string, data?: unknown) =>
    sendRequest<TResponse>(endpoint, 'POST', data),
  put: <TResponse>(endpoint: string, data?: unknown) =>
    sendRequest<TResponse>(endpoint, 'PUT', data),
  remove: <TResponse>(endpoint: string) => sendRequest<TResponse>(endpoint, 'DELETE')
}

async function sendRequest<TResponse>(
  endpoint: string,
  method: string,
  data?: unknown,
  params?: RequestInit
): Promise<Result<TResponse, Error>> {
  const url = resolveUrl(endpoint)
  const body = data === undefined ? undefined : JSON.stringify(data)

  // Only a JSON body needs a content type; adding one to a bare GET turns a cross-origin
  // request (the robot, api.github.com) into a CORS preflight.
  const headers = new Headers(params?.headers)
  if (body !== undefined) headers.set('Content-Type', 'application/json')

  let response: Response
  try {
    response = await fetch(url, { ...params, method, body, headers })
  } catch (error) {
    return Err.new(error instanceof Error ? error : new Error(String(error)), 'Request failed')
  }

  if (response.status < 200 || response.status >= 400) {
    return Err.new(new ApiError(response), `Request failed with status ${response.status}`)
  }

  const contentType = response.headers.get('Content-Type')
  if (contentType?.includes('application/json')) {
    return Ok.new((await response.json()) as TResponse)
  }
  return Ok.new(null as TResponse)
}

class ApiError extends Error {
  constructor(public readonly response: Response) {
    super(`${response.status}`)
  }
}
