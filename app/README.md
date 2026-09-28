# Hexapod controller

SvelteKit web controller for the hexapod robot.
It drives the robot and shows its telemetry over Web Bluetooth, WebSocket or Web Serial, all carrying the protobuf messages defined in `../platform_shared/*.proto`.

## Commands

Run from this directory with pnpm.

```sh
pnpm install
pnpm dev                # vite dev server, reachable on the LAN (--host)
pnpm build              # static build into build/ (GitHub Pages)
pnpm build:embedded     # build for embedding into the firmware (VITE_USE_HOST_NAME=true)
pnpm check              # svelte-check type checking
pnpm lint               # prettier --check and eslint
pnpm format             # prettier --write
pnpm test:unit          # vitest, single run
pnpm test:integration   # playwright against a production preview
```

`pnpm dev` and both builds first run `pnpm proto`, which regenerates `src/lib/platform_shared/` from the proto files.
It needs a python with `grpcio-tools` on PATH.

## Build variables

- `BASE_PATH` sets the path the app is served under, for example `/Hexapod` for GitHub Pages.
  Leave it unset when the app is served from the root.
- `VITE_USE_HOST_NAME=true` makes the app talk to the host that served it instead of a saved robot address.
  Use it for the build that the robot hosts itself.

## Transports

Which transport a page can use depends on its origin: an https page gets Bluetooth and USB but no WebSocket or camera, while the robot-hosted http page gets WebSocket and camera but neither Bluetooth nor USB, since both need a secure context.
See `../docs/connectivity.md` for the full matrix and the reasoning behind it.
