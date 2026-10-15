# Releasing

How a version reaches a robot, and the steps that cut one.
Everything here is read from `.github/workflows/release.yml`, `.github/workflows/deploy.yml`, `.github/workflows/embedded-build.yml`, `firmware/scripts/merge_factory.py`, `flasher/` and `CHANGELOG.md`.
No version has been tagged yet; the first one follows these steps with no previous release to update from (see step 8).

## What a release produces

Pushing a tag that matches `v*` runs `release.yml`.
It calls `embedded-build.yml` with `elf: true`, which runs the host tests (`pio test -e native`) and builds `esp32-wroom-camera`.
Only when both jobs pass does the release job run.
It publishes a GitHub release, with generated notes, holding three files:

| Asset | Made by | Used for |
| --- | --- | --- |
| `esp32-wroom-camera.factory.bin` | `firmware/scripts/merge_factory.py` (bootloader, partition table, OTA data and app, merged for offset 0) | The web flasher, which writes it to a blank or running board in one pass |
| `esp32-wroom-camera.bin` | The plain app image from the build | Over-the-air updates, which write it into the inactive app slot |
| `esp32-wroom-camera.elf` | The build's `firmware.elf` | Decoding a core dump the robot kept (`docs/firmware.md`, "Decoding a core dump") |

The two `.bin` names are load-bearing.
The app's update page installs the asset named exactly `<firmware_built_target>.bin` (`findFirmwareAsset` in `app/src/lib/services/firmware-update.ts`), and `flasher/generate_manifest.py` expects `esp32-wroom-camera.factory.bin`.
Renaming either asset in `release.yml` breaks one of the two install paths without failing the workflow.

The last step of the release job runs `gh workflow run deploy.yml --ref main`.
The `github-pages` environment accepts deployments from `main` only, so the tag cannot deploy the flasher itself.
`deploy.yml` then fetches the newest release's `*.factory.bin` with `gh release view` and `gh release download`, writes `manifest.json` with the tag as its version, and publishes both inside the Pages artifact under `/flash/`.
The flasher serves neither the ELF nor the plain app image.

## The version

The firmware's version is `APP_VERSION` in `firmware/factory_settings.ini`, currently `0.0.1`.
It is the only place to edit.
The firmware reports it in the features response, the static system information and the mDNS TXT record (`firmware/src/main.cpp`, `firmware/src/features.cpp`, `firmware/include/settings/mdns_settings.h`).
The other occurrences in the repository are untracked editor IntelliSense files and do not affect a build.

Nothing derives the version from the tag.
`release.yml` does not read `APP_VERSION`, and `deploy.yml` labels the flasher with the tag name.
Two consumers compare the two strings:

- The update page marks a release as installed when `compareVersions(firmware_version, tag)` is zero (`app/src/routes/system/update/GithubFirmwareManager.svelte`), and `compare-versions` ignores a leading `v`.
- The flasher shows the tag as the firmware version.

The rule is therefore: the tag is `v` followed by `APP_VERSION`, so `APP_VERSION` `0.2.0` is tagged `v0.2.0`.
A mismatch ships without any CI failure, and the update page then highlights the wrong row or none.
Use plain `MAJOR.MINOR.PATCH`; `compareVersions` accepts only semver-shaped strings.

## Procedure

Run the steps in order.
A failure at any step stops the release; fix it, then restart from step 1 so the tag always points at a commit that passed everything.

### 1. Check that `main` is ready

The release builds the tagged commit, and the flasher page comes from `main`, so tag a commit that is on `main`.

```sh
git switch main
git pull
git status
```

The working tree must be clean, and the latest `main` run of each workflow must be green (`gh run list --branch main --limit 8`):

- `PlatformIO CI`: host tests and the firmware build.
- `Deploy GitHub Pages`: app checks, unit and integration tests, build, flasher.
- `Simulation tests`: simulation, parity and framing tests.

### 2. Set the version

Edit `APP_VERSION` in `firmware/factory_settings.ini` to the new `MAJOR.MINOR.PATCH`.
Check it:

```sh
grep -n APP_VERSION firmware/factory_settings.ini
```

### 3. Roll the changelog

`CHANGELOG.md` follows Keep a Changelog.

1. Rename `## [Unreleased]` to `## [X.Y.Z] - YYYY-MM-DD`, with no `v` and the release date.
2. Add a fresh empty `## [Unreleased]` above it.
3. On the first release only, delete the sentence "No version has been tagged yet; everything so far is unreleased."
4. Keep one sentence per line, ASCII punctuation, and the existing headings (Added, Changed, Fixed).

The GitHub release notes are generated from merged pull requests and commits (`--generate-notes`), so the changelog section is the curated record and the generated notes are the raw one.

### 4. Hardware-in-the-loop smoke

Script pending (roadmap 7.3).
Until `simulation/hil_smoke.py` exists, run the bench by hand against a robot flashed with the release candidate: stand, walk each gait briefly, play `wave`, and watch the telemetry for overruns and clamps.
When the script lands, it replaces this paragraph, and its output is pasted into the release notes in step 6.

### 5. Commit and tag

Commit the version and the changelog together, in the repository's commit style (one line, a gitmoji, then a verb, for example "Releases 0.2.0" after the gitmoji).
Then tag that commit and push both.

```sh
git tag v0.2.0
git push origin main v0.2.0
```

The push of the tag starts `release.yml`.
Follow it with `gh run watch`.
A failed run publishes nothing, since the release job needs the build.
Do not move or reuse a tag whose release was published: robots and the flasher may already hold its images, so fix forward with the next patch version.

### 6. Check the release

```sh
gh release view v0.2.0 --json assets -q '.assets[].name'
```

The list must hold `esp32-wroom-camera.factory.bin`, `esp32-wroom-camera.bin` and `esp32-wroom-camera.elf`.
Add the HIL output from step 4 to the notes: start from `gh release view v0.2.0 --json body -q .body`, append the output to that text in a file, and apply it with `gh release edit v0.2.0 --notes-file <file>`, which replaces the notes.

### 7. Check the flasher

The release job started a `Deploy GitHub Pages` run on `main`; wait for it (`gh run list --workflow deploy.yml --limit 3`).
Then open `https://runeharlyk.github.io/Hexapod/flash/manifest.json`.
Its `version` must be the new tag, and its one part must be `firmware/esp32-wroom-camera.factory.bin` at offset 0.
Open `https://runeharlyk.github.io/Hexapod/flash/`: it must read "Firmware vX.Y.Z" with the install button shown, not the empty-state card.
For a full check, install on the bench board from a Chromium browser and confirm the robot boots to the new version (the features response, or the update page's installed row).

Pages deploys also run on every push to `main`, and each takes the newest release, so a later push cannot regress the flasher.
The connection rules that gate what the flashed robot can do next are in `docs/connectivity.md`.

### 8. Check over-the-air update from the previous release

Skip this step for the first release; there is nothing older to update from.

1. Put a robot on the previous release, by flashing its `.factory.bin` from the flasher or by installing it from the update page.
2. Join the robot to a WiFi network with internet access: the robot downloads the image itself over https (`POST /api/firmware/download`, `firmware/include/ota_service.h`).
3. Open the app from an origin that can reach the robot (see `docs/connectivity.md`), go to the update page and install the new release from the Github Firmware Manager.
4. After the restart the app reports whether the new image is running or the robot rolled back, from the ELF hash.
   The new tag's row must be marked installed.
   The firmware confirms the image only after the control loop has run 200 ticks, so a build that crashes sooner reverts to the old slot.

If the robot cannot reach GitHub, upload `esp32-wroom-camera.bin` through the update page's file upload instead; that checks the image but not the download path.

### 9. After the release

Decoding a core dump needs the ELF of the exact build on the robot.
Keep `esp32-wroom-camera.elf` attached to its release, and do not delete a release that a robot still runs.
