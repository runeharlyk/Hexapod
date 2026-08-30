"""Bridge the ESP-NOW handheld controller (over USB) into the MuJoCo sim.

The controller (../../Hardware/esp-now-controller) exposes a line-based CLI on
its native USB Serial/JTAG port. Sent `stream on`, it emits one JSON telemetry
line per sample:

    {"t":"tlm","raw":[...],"n":[lx,ly,rx,ry],"btn":u,"seq":...}

with axes calibrated/normalized to -1000..1000 (0 = centered) and `btn` a bitmask
of BTN_* (see controller_packet.h). This reader thread keeps the latest axes and
accumulates button rising edges so the sim can be driven exactly like the robot
(cf. firmware/src/communication/espnow_adapter.cpp), without polling losing a
transient button press.

Standalone smoke test (prints live axes/buttons):

    uv run python controller_bridge.py            # auto-detect the port
    uv run python controller_bridge.py --serial COM11
"""

from __future__ import annotations

import json
import threading

import serial
from serial.tools import list_ports

# Mirrors controller_packet.h.
AXIS_FULL_SCALE = 1000.0
BTN_LEFT = 1 << 0
BTN_RIGHT = 1 << 1
BTN_A = 1 << 2
BTN_B = 1 << 3
BTN_C = 1 << 4

# ESP32-S3 native USB Serial/JTAG identifies as an Espressif device.
ESPRESSIF_VID = 0x303A


def find_controller_port() -> str | None:
    """Return the first Espressif USB serial device, or None if none is present."""
    for p in list_ports.comports():
        if p.vid == ESPRESSIF_VID:
            return p.device
    return None


class ControllerBridge:
    """Reads controller telemetry on a background thread; exposes a thread-safe snapshot.

    Axes are served as floats in [-1, 1] (raw normalized / AXIS_FULL_SCALE). Button
    rising edges are latched until drained via `take_rising()` so no press is missed
    between sim ticks; the held button mask is available via `buttons`.
    """

    def __init__(self, port: str | None = None, stream_hz: int = 50):
        self.port = port or find_controller_port()
        if not self.port:
            raise RuntimeError(
                "no controller found — plug in the ESP-NOW controller over USB, "
                "or pass --serial COMx explicitly"
            )
        self.stream_hz = stream_hz
        self._lock = threading.Lock()
        self._axes = [0.0, 0.0, 0.0, 0.0]  # lx, ly, rx, ry in [-1, 1]
        self._buttons = 0
        self._rising = 0  # accumulated rising edges, drained by take_rising()
        self._prev_buttons = 0
        self._seq = 0
        self.connected = False
        self._stop = threading.Event()
        self._ser: serial.Serial | None = None
        self._thread = threading.Thread(target=self._run, name="controller", daemon=True)

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> "ControllerBridge":
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ accessors
    def snapshot(self):
        """(lx, ly, rx, ry) floats in [-1, 1]; the latest received axis values."""
        with self._lock:
            return tuple(self._axes)

    @property
    def buttons(self) -> int:
        with self._lock:
            return self._buttons

    def take_rising(self) -> int:
        """Return accumulated button rising edges since the last call, then clear them."""
        with self._lock:
            r = self._rising
            self._rising = 0
            return r

    # ------------------------------------------------------------------ reader
    def _run(self):
        try:
            self._ser = serial.Serial(self.port, baudrate=115200, timeout=0.2)
        except Exception as e:
            print(f"[controller] cannot open {self.port}: {e}")
            return
        # Ask the device to stream at the sim's control rate.
        try:
            self._ser.write(f"streamhz {self.stream_hz}\n".encode())
            self._ser.write(b"stream on\n")
            self._ser.flush()
        except Exception as e:
            print(f"[controller] write failed: {e}")
            return
        print(f"[controller] streaming from {self.port} @ {self.stream_hz} Hz")

        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue
            try:
                msg = json.loads(raw.decode("utf-8", "ignore").strip())
            except (json.JSONDecodeError, ValueError):
                continue
            if msg.get("t") != "tlm":
                continue
            n = msg.get("n")
            if not (isinstance(n, list) and len(n) == 4):
                continue
            btn = int(msg.get("btn", 0))
            with self._lock:
                self._axes = [max(-1.0, min(1.0, v / AXIS_FULL_SCALE)) for v in n]
                self._rising |= btn & ~self._prev_buttons
                self._prev_buttons = btn
                self._buttons = btn
                self._seq = int(msg.get("seq", self._seq))
                self.connected = True


def _btn_names(mask: int) -> str:
    names = [("L", BTN_LEFT), ("R", BTN_RIGHT), ("A", BTN_A), ("B", BTN_B), ("C", BTN_C)]
    held = [name for name, bit in names if mask & bit]
    return "+".join(held) if held else "-"


if __name__ == "__main__":
    import argparse
    import time

    ap = argparse.ArgumentParser(description="Print live controller telemetry (smoke test).")
    ap.add_argument("--serial", default=None, help="serial port (default: auto-detect Espressif)")
    args = ap.parse_args()

    bridge = ControllerBridge(port=args.serial).start()
    print("reading — Ctrl+C to stop")
    try:
        while True:
            lx, ly, rx, ry = bridge.snapshot()
            rising = bridge.take_rising()
            edge = f"  rising={_btn_names(rising)}" if rising else ""
            print(
                f"\rlx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} ry={ry:+.2f} "
                f"held={_btn_names(bridge.buttons):8s}{edge}    ",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        bridge.close()
