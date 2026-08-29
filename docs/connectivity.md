# Connectivity: how the app reaches the robot

The transport story is constrained by browser security rules, not by the firmware.
Getting this wrong wastes effort on firmware features that a browser can never use, so read the constraint section before proposing a connection flow.

## Three problems that get conflated

1. **Provisioning** — how the robot learns which WiFi to join.
2. **Transport** — how bytes get between app and robot.
3. **Origin permission** — whether the *browser* may use that transport from the page's origin.

Espressif's provisioning APIs (DPP, SmartConfig, Unified Provisioning, NAN discovery) all address problem 1 only.
None of them make an https page able to reach a LAN device, so none of them fix camera streaming on their own.

## The binding constraint

Two browser rules collide:

- Web Bluetooth requires a **secure context** (https, `localhost`, or `127.0.0.1`).
- Mixed-content blocking forbids an https page from opening `ws://` or loading `http://` images.

So the origin serving the app decides which transports exist at all:

| Origin | Web Bluetooth | `ws://` to robot | camera MJPEG |
| --- | --- | --- | --- |
| `https://…github.io/Hexapod/` (Pages) | yes | **blocked** | **blocked** |
| `http://esp32.local` (`EMBED_WEBAPP=1`) | **no** | yes | yes |
| `http://localhost:5173` (dev) | yes | yes | yes |

Corollaries that keep coming up:

- An in-place "pair over BLE, then upgrade to WebSocket" handoff only works from `http://localhost`.
- From https the app cannot even *probe* a LAN address, so "are we on the same network?" is unanswerable there. It can only offer an address and let navigation succeed or fail.
- The browser cannot learn its own LAN IP (WebRTC candidates are mDNS-obfuscated), so subnet comparison is not available either.

## Provisioning options

Verified against the Arduino-ESP32 2.0.14 / IDF 4.4.6 toolchain that was pinned at the time of writing (`platform = espressif32 @ 6.6.0`).

| Method | Header present | Client required | Verdict |
| --- | --- | --- | --- |
| SoftAP + captive portal | yes (`APService` + `DNSServer`) | any browser | Already built. Serves nothing while `EMBED_WEBAPP=0` |
| **Custom BLE message** | n/a | our own app | Works from https. Smallest path to the hotspot use case. **Chosen** |
| Unified Provisioning | `wifi_provisioning` + `WiFiProv.h` | Espressif's Android/iOS app or `esp_prov` CLI — no web client | See "the unified protocol" below. **Not chosen** |
| SmartConfig / ESP-Touch | `esp_smartconfig.h` | dedicated phone app broadcasting UDP | A browser cannot send raw UDP broadcast. Dead end |
| DPP / Easy Connect | `esp_dpp.h` | Android 10+ at OS level, no app | ESP32 is enrollee-only and must *display* a QR; no iOS API. Skipped |
| NAN / Wi-Fi Aware | **absent** (needs IDF ≥ 5.1) | Android Wi-Fi Aware, native only | Skipped |

## Transports

| Transport | Browser? | Origin needed | Camera |
| --- | --- | --- | --- |
| HTTP + WebSocket | yes | **http only** | yes |
| Web Bluetooth | yes | **secure only** | no — far too slow |
| ESP-NOW | no | — | no |
| NAN datapath | no | — | needs IDF ≥ 5.1 anyway |
| Web Serial / WebUSB | desktop-only / no iOS | secure | impractical |
| **WebRTC** | yes | **works from https** | yes |

WebRTC is the outlier worth understanding.
It is not subject to mixed-content blocking — it mandates DTLS and is not a mixed-content fetch — so an https page can reach a LAN peer by its raw ICE candidate with no CA certificate.
BLE can carry the SDP/ICE signalling, so no server is required.
That combination is the only known way to get control *and* camera from GitHub Pages.

Caveats: Safari filters host ICE candidates without camera/mic permission, so iOS likely needs a `getUserMedia` grant or a relay.
Firmware side is [`espressif/esp-webrtc-solution`](https://github.com/espressif/esp-webrtc-solution) (ICE/STUN/TURN, DTLS-SRTP, DataChannel, MJPEG/H.264) — an **ESP-IDF component**, which is why this only became practical with the IDF migration.
The mixed-content exemption is the load-bearing claim here and has not been prototyped; validate the browser half against a desktop peer before committing firmware effort.

## What is implemented today

`NET_STATUS = 16` and `NET_COMMAND = 17` (`firmware/include/message_types.h`, mirrored in `app/src/lib/interfaces/transport.interface.ts`).
The topic enum had free slots, so nothing shifted — unlike `MOTION_STATE`.

- `NetStatusMsg` reports `sta`/`ap` state, SSIDs and IPs. Republished only on change; `peek()` gives late subscribers the current value.
- `NetCommandMsg` carries `REPORT` / `FORCE_AP` / `AUTO_AP`. `FORCE_AP` sets `provisionMode = AP_MODE_ALWAYS` via `apService->update`.
- App side is `app/src/lib/stores/network.ts`. After BLE pairs, the landing page offers "Open over WiFi" (navigates to `http://<ip>/`) or "Stay on Bluetooth", which sets a persisted `preferBluetooth` so it stops asking.

The offer is never forced — staying on pure Bluetooth is a first-class choice for bad-WiFi situations.

**Untested against hardware.** The BLE pairing path, the `NET_STATUS` round trip, and the WiFi-upgrade branch all need a real robot.

## Use case: demo the camera with no WiFi available

Camera needs the WebSocket/HTTP transport, so BLE alone cannot carry it.

1. **Join the robot's own AP.** Already automatic — `FACTORY_AP_PROVISION_MODE = AP_MODE_DISCONNECTED` (`ap_settings.h`) raises the AP at `192.168.4.1` whenever the robot is not joined to WiFi, with captive-portal DNS (`APService::handleDNS`). Yields no UI until `EMBED_WEBAPP=1`.
2. **Phone hotspot + provision over BLE.** Start a hotspot, pair over BLE, hand the robot the SSID/password so it joins. Both ends land on one network. Needs the provisioning message; still hits the https origin problem on the last mile.
3. **Fix the origin.** WebRTC (above), or a Capacitor/Tauri wrapper, or TLS from the ESP32.

## Ranked plan

1. **BLE WiFi provisioning** — smallest delta, works from https, unlocks the hotspot demo.
2. **`EMBED_WEBAPP=1`** — one flag; makes the existing AP + captive portal actually serve a UI.
3. **WebRTC over BLE signalling** — the real fix for camera-from-https.
4. Capacitor wrapper — *deprioritized*: maintaining a native app is unattractive when a webapp would do.
5. Unified Provisioning via Web Bluetooth — *not chosen*, see below.
6. DPP / SmartConfig / NAN — *skipped*. All require a native app, at which point Capacitor gives more.

## "The unified protocol": two different things

- **Espressif's Unified Provisioning** (protocomm) would need BLE first, then its protobuf schema *plus* Security1 (X25519 + AES-CTR) or Security2 (SRP6a) implemented in TS over Web Bluetooth. It ships its own protobuf definitions, so the app would carry two schemas. The only payoff is interop with Espressif's phone apps, which is irrelevant to a webapp-first design.
- **Our own protobuf schema shared across transports** is what Spot already built: `platform_shared/*.proto` compiled by nanopb for firmware and ts-proto for the app. Adding a `WifiProvision` message is small once that pipeline is in place.

**Decision: use our own schema.** Same protobuf rigor, one schema, no second crypto stack, works from https.
Revisit protocomm only if Espressif-app interop ever matters.
