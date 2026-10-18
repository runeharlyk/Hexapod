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
| `http://hexapod.local` (`EMBED_WEBAPP=1`) | **no** | yes | yes |
| `http://localhost:5173` (dev) | yes | yes | yes |

Corollaries that keep coming up:

- An in-place "pair over BLE, then upgrade to WebSocket" handoff only works from `http://localhost`.
- From https the app cannot even *probe* a LAN address, so "are we on the same network?" is unanswerable there. It can only offer an address and let navigation succeed or fail.
- The browser cannot learn its own LAN IP (WebRTC candidates are mDNS-obfuscated), so subnet comparison is not available either.

## Provisioning options

Originally verified against the Arduino-ESP32 2.0.14 / IDF 4.4.6 toolchain.
The firmware now builds on ESP-IDF 5.4 (pioarduino); the NAN row below is updated for it.

| Method | Header present | Client required | Verdict |
| --- | --- | --- | --- |
| SoftAP + captive portal | yes (`APService` + `DNSServer`) | any browser | Already built. Serves nothing while `EMBED_WEBAPP=0` |
| **Custom BLE message** | n/a | our own app | Works from https. Smallest path to the hotspot use case. **Chosen** |
| Unified Provisioning | `wifi_provisioning` + `WiFiProv.h` | Espressif's Android/iOS app or `esp_prov` CLI — no web client | See "the unified protocol" below. **Not chosen** |
| SmartConfig / ESP-Touch | `esp_smartconfig.h` | dedicated phone app broadcasting UDP | A browser cannot send raw UDP broadcast. Dead end |
| DPP / Easy Connect | `esp_dpp.h` | Android 10+ at OS level, no app | ESP32 is enrollee-only and must *display* a QR; no iOS API. Skipped |
| NAN / Wi-Fi Aware | `esp_nan.h` present, but the ESP32-S3 lacks `SOC_WIFI_NAN_SUPPORT` | Android Wi-Fi Aware, native only | Skipped |

## Transports

| Transport | Browser? | Origin needed | Camera |
| --- | --- | --- | --- |
| HTTP + WebSocket | yes | **http only** | yes |
| Web Bluetooth | yes | **secure only** | no — far too slow |
| ESP-NOW | no | — | no |
| NAN datapath | no | — | not supported on the ESP32-S3 |
| Web Serial | desktop Chromium only, no iOS | secure | no |
| **WebRTC** | yes | **works from https** | yes |

WebRTC is the outlier worth understanding.
It is not subject to mixed-content blocking — it mandates DTLS and is not a mixed-content fetch — so an https page can reach a LAN peer by its raw ICE candidate with no CA certificate.
BLE can carry the SDP/ICE signalling, so no server is required.
That combination is the only known way to get control *and* camera from GitHub Pages.

Caveats: Safari filters host ICE candidates without camera/mic permission, so iOS likely needs a `getUserMedia` grant or a relay.
Firmware side is [`espressif/esp-webrtc-solution`](https://github.com/espressif/esp-webrtc-solution) (ICE/STUN/TURN, DTLS-SRTP, DataChannel, MJPEG/H.264) — an **ESP-IDF component**, which is why this only became practical with the IDF migration.
The mixed-content exemption is the load-bearing claim here and has not been prototyped; validate the browser half against a desktop peer before committing firmware effort.

## What is implemented today

Web Serial is a third transport: the firmware carries the same protobuf frames over the ESP32-S3's native USB Serial/JTAG port (`firmware/include/communication/serial_adapter.h`), and the app opens it from https like Web Bluetooth.
Provisioning is the `wifi_settings_update` `CorrelationRequest` (`platform_shared/message.proto`), which works over every transport; the app's WiFi page sends it.
`wifi_status_get` / `ap_status_get` answer with the current state, and `WifiStatus` / `APStatus` messages are pushed on change to subscribed clients.
Stored passwords are never sent back: reads return them empty, and an update with an empty password keeps the stored one.

The `NET_STATUS` / `NET_COMMAND` topics, the `FORCE_AP` command, `app/src/lib/stores/network.ts` and the "Open over WiFi" offer described in earlier revisions of this document were never committed; they exist only in the local git stash `animation and nets`.

## Use case: demo the camera with no WiFi available

Camera needs the WebSocket/HTTP transport, so BLE alone cannot carry it.

1. **Join the robot's own AP.** Already automatic — `FACTORY_AP_PROVISION_MODE = AP_MODE_DISCONNECTED` (`ap_settings.h`) raises the AP at `192.168.4.1` whenever the robot is not joined to WiFi, with captive-portal DNS (`DNSServer` in `firmware/include/wifi/dns_server.h`, which runs on its own task). Yields no UI until `EMBED_WEBAPP=1`.
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

## Update: Chromium's Local Network Access relaxes the binding constraint

Checked 2026-10-05 against the primary sources; the WebSocket part re-checked 2026-10-13 and confirmed shipped in Chrome 147.
Chromium's Local Network Access (LNA) exempts a request from mixed-content blocking when Chrome knows before resolving that it targets the local network: a private IP literal or a `.local` name ([Chrome blog](https://developer.chrome.com/blog/local-network-access)).
Such a request asks the user for the local network permission instead, which needs a secure context, so the https Pages origin qualifies.
A hostname that only resolves to a private address gets no exemption.
For `fetch()` and subresources such as the camera's `<img>`, the permission prompt launched in Chrome 142.
For WebSockets, the [Intent to Ship](https://groups.google.com/a/chromium.org/g/blink-dev/c/O6GMKt44Ups) states that "explicit local IP addresses, and `.local` domains are exempted from mixed content checks" and targets Chrome 147 (flag `local-network-access-check-websockets`).
The thread showed LGTM1 (2026-03-13) and LGTM2 (2026-03-18) when read on 2026-10-13; the rest of the thread was truncated in the fetch.
It shipped in Chrome 147, stable on 2026-04-07 (all checked 2026-10-13):
- The [Chrome 147 release notes](https://developer.chrome.com/release-notes/147) list "Local network access restrictions for WebSockets": "WebSockets connections to local addresses now trigger permission prompts."
- The [chromestatus entry](https://chromestatus.com/feature/5197681148428288) gives the shipping milestone as 147 on desktop and Android (none for WebView or iOS), and its milestone-147 listing files the feature under "Enabled by default"; the API Owners Intent to Ship gate reads approved.
- The entry's status field still says "Proposed" (last updated 2026-03-24), which contradicts the milestone and the release note; the release note is taken as authoritative.
- The source at tag `147.0.7727.99` has `kLocalNetworkAccessChecks` and `kLocalNetworkAccessChecksWebSockets` both `FEATURE_ENABLED_BY_DEFAULT` (`services/network/public/cpp/features.cc`), and `MixedContentChecker::IsWebSocketAllowed` (`third_party/blink/renderer/core/loader/mixed_content_checker.cc`) skips the mixed-content block when the URL alone maps to the local or loopback address space, which `GetAddressSpaceFromUrl` grants to private IP literals and `.local` hosts.
So from an https page in Chrome 147 or later, `new WebSocket("ws://192.168.x.y/api/ws")` is no longer blocked as mixed content; it raises the local network permission prompt and connects once the user allows it.
A `ws://` URL whose hostname is neither a private IP literal nor `.local` is still blocked as mixed content.
The release note does not quote the mixed-content wording itself; that part rests on the Intent to Ship and the source above.
A server-side Finch override could still disable the feature for some users; that cannot be checked from outside.
chromestatus lists "No signal" from Firefox and Safari, so other browsers should be assumed to block it until checked.
Where it applies, the https app can open `ws://192.168.x.y/api/ws` and show the camera after one permission prompt, without BLE or USB.
The app still opens the WebSocket automatically only on the robot-hosted `http:` origin (`app/src/routes/+layout.svelte`), and the firmware's origin policy (`firmware/include/communication/origin_policy.h`) already admits the Pages origin.
In browsers without this exemption the table above still holds.

## Update: the WebSocket exemption has shipped

Checked 2026-10-15.
The [Chrome 147 release notes](https://developer.chrome.com/release-notes/147), stable since 2026-04-07, list "Local network access restrictions for WebSockets": a WebSocket to a local address now goes through the same permission prompt.
In Chrome 155 on the Pages origin `https://runeharlyk.github.io` with the permission granted, `ws://192.168.1.151/api/ws` opened and received the robot's greeting, and `fetch('http://192.168.1.151/api/features')` answered 200 through the firmware's CORS headers.
The app therefore opens the WebSocket from https whenever a robot address is saved, and resolves robot URLs to `http://` on every origin.
Firefox and Safari were not tested; a browser that refuses the socket gets an error naming Bluetooth and USB instead of endless retries.

Still open: the https app could learn the robot's IP from the BLE or serial link's network status and offer to switch to WiFi, so the address need not be typed; it should say beforehand that the permission prompt is coming.
