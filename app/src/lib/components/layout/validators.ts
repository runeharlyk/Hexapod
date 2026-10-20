/**
 * Native constraints the settings forms share, as `pattern` attributes: the browser anchors a pattern to the whole value
 * and compiles it with the `v` flag, and Field shows the hint beside a control that fails it.
 */

const OCTET = '(?:2(?:[0-4][0-9]|5[0-5])|[0-1]?[0-9]?[0-9])'

/** A dotted IPv4 address, each octet 0 to 255. */
export const IPV4_PATTERN = String.raw`(?:${OCTET}\.){3}${OCTET}`

export const IPV4_HINT = 'An IPv4 address, such as 192.168.1.10'
