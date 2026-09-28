import { Err } from './err'
import { Ok } from './ok'

export type Result<T = unknown, E = unknown, F = unknown> = Ok<T> | Err<E, F>

export const Result = {
  ok: <T = unknown>(value: T) => Ok.new(value),
  err: <E = unknown, F = unknown>(error: E, exception?: F) => Err.new(error, exception)
}
