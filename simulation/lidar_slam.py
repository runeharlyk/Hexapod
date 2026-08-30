"""Standalone 2D LiDAR SLAM for the LD500 / Waveshare D500-class scanner.

The LD500 belongs to the LDROBOT / STL LiDAR family (LD06, LD19, STL-19P, D500).
They all stream the same frame format over a USB-serial bridge:

    230400 baud, 8N1, little-endian
    47-byte packet:
        [0]      0x54                header
        [1]      0x2C                VerLen (low 5 bits = 12 points per packet)
        [2:4]    uint16  speed       deg/s (rotation speed)
        [4:6]    uint16  start_angle 0.01 deg
        [6:42]   12 x (uint16 dist_mm, uint8 intensity)
        [42:44]  uint16  end_angle   0.01 deg
        [44:46]  uint16  timestamp   ms
        [46]     uint8   crc8        over bytes [0:46]

This script needs nothing beyond numpy + matplotlib + pyserial. Run with no
hardware via --simulate to see the SLAM pipeline working immediately.

Usage:
    python lidar_slam.py                 # auto-detect port, run SLAM
    python lidar_slam.py --port COM5     # force a port
    python lidar_slam.py --list          # list serial ports and exit
    python lidar_slam.py --raw           # just plot the live 360 deg point cloud
    python lidar_slam.py --simulate      # synthetic room, no hardware needed
    python lidar_slam.py --lock-pose     # disable scan-matching (pure mapping)
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

# --------------------------------------------------------------------------- #
# LD500 / LDROBOT serial protocol
# --------------------------------------------------------------------------- #

HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACKET = 12
PACKET_LEN = 47
DEFAULT_BAUD = 230400

# CRC8 lookup table from the official LDROBOT LiDAR SDK (poly 0x4D).
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def list_ports():
    import serial.tools.list_ports as lp

    return list(lp.comports())


class LD500Driver:
    """Reads LD500 packets and assembles full 360 deg scans."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, check_crc: bool = True):
        self.port = port
        self.baud = baud
        self.check_crc = check_crc
        self._buf = bytearray()
        self._scan = []          # accumulating (angle_deg, dist_m, quality)
        self._last_angle = None
        self._bad_crc = 0
        self._packets = 0
        self._reconnects = 0
        self.ser = self._open()

    def _open(self):
        import serial

        return serial.Serial(self.port, self.baud, timeout=1.0)

    def _reopen(self):
        """Recover from a transient serial drop (USB reset / port stolen)."""
        import serial

        self.close()
        self._buf.clear()
        for attempt in range(60):          # ~30 s of retries
            try:
                self.ser = self._open()
                self._reconnects += 1
                print(f"  reconnected to {self.port}")
                return True
            except serial.SerialException:
                time.sleep(0.5)
        return False

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _read_packet(self):
        import serial

        """Sync to a header and return the next valid 47-byte packet."""
        while True:
            # Refill buffer.
            need = PACKET_LEN - len(self._buf)
            if need > 0:
                try:
                    chunk = self.ser.read(max(need, 256))
                except serial.SerialException as e:
                    print(f"  serial error: {e} -- attempting reconnect ...")
                    if not self._reopen():
                        return None
                    continue
                if not chunk:
                    return None
                self._buf.extend(chunk)

            # Find header pair.
            i = 0
            n = len(self._buf)
            found = -1
            while i < n - 1:
                if self._buf[i] == HEADER and self._buf[i + 1] == VERLEN:
                    found = i
                    break
                i += 1
            if found < 0:
                # No header yet; keep last byte in case it is a split header.
                self._buf = self._buf[-1:]
                continue
            if found > 0:
                del self._buf[:found]
            if len(self._buf) < PACKET_LEN:
                continue

            pkt = bytes(self._buf[:PACKET_LEN])
            if self.check_crc and crc8(pkt[:46]) != pkt[46]:
                # Bad packet: drop the header byte and resync.
                self._bad_crc += 1
                del self._buf[:2]
                continue
            del self._buf[:PACKET_LEN]
            self._packets += 1
            return pkt

    @staticmethod
    def _parse(pkt: bytes):
        start = (pkt[4] | (pkt[5] << 8)) / 100.0          # deg
        end = (pkt[42] | (pkt[43] << 8)) / 100.0          # deg
        span = (end - start) % 360.0
        step = span / (POINTS_PER_PACKET - 1)
        out = []
        for k in range(POINTS_PER_PACKET):
            off = 6 + k * 3
            dist = (pkt[off] | (pkt[off + 1] << 8)) / 1000.0   # m
            quality = pkt[off + 2]
            ang = (start + step * k) % 360.0
            out.append((ang, dist, quality))
        return out

    def scans(self):
        """Yield (angles_rad, dists_m, quality) numpy arrays, one per revolution."""
        while True:
            pkt = self._read_packet()
            if pkt is None:
                if self._scan:
                    yield self._emit()
                return
            for ang, dist, q in self._parse(pkt):
                if self._last_angle is not None and ang < self._last_angle - 180.0:
                    # Angle wrapped past 360 -> a full revolution completed.
                    self._last_angle = ang
                    scan = self._emit()
                    self._scan.append((ang, dist, q))
                    if scan is not None:
                        yield scan
                    continue
                self._last_angle = ang
                self._scan.append((ang, dist, q))

    def _emit(self):
        if not self._scan:
            return None
        arr = np.array(self._scan, dtype=np.float64)
        self._scan = []
        ang = np.deg2rad(arr[:, 0])
        dist = arr[:, 1]
        q = arr[:, 2]
        valid = (dist > 0.02) & (dist < 30.0)
        return ang[valid], dist[valid], q[valid]


# --------------------------------------------------------------------------- #
# Synthetic LiDAR (for --simulate, no hardware)
# --------------------------------------------------------------------------- #

class SimulatedLiDAR:
    """Ray-casts a fake room so the SLAM pipeline can run without hardware."""

    def __init__(self, n_beams=450, noise=0.01):
        self.n = n_beams
        self.noise = noise
        # Room: outer walls + a couple of inner obstacles (axis-aligned boxes).
        # Each box is (xmin, ymin, xmax, ymax) in meters; sensor sits near origin.
        self.walls = [
            (-3.0, -2.5, 3.0, 2.5),    # outer room (treated as inside-facing)
        ]
        self.boxes = [
            (1.0, -0.5, 1.6, 1.5),     # pillar
            (-2.0, 0.8, -1.0, 1.4),    # shelf
        ]
        self._t = 0

    def _cast(self, ox, oy, ang):
        best = 30.0
        dx, dy = math.cos(ang), math.sin(ang)
        # Outer room interior walls.
        room = self.walls[0]
        for d in self._box_hits(ox, oy, dx, dy, room, inside=True):
            best = min(best, d)
        for box in self.boxes:
            for d in self._box_hits(ox, oy, dx, dy, box, inside=False):
                best = min(best, d)
        return best

    @staticmethod
    def _box_hits(ox, oy, dx, dy, box, inside):
        xmin, ymin, xmax, ymax = box
        ts = []
        for x in (xmin, xmax):
            if dx != 0:
                t = (x - ox) / dx
                if t > 0:
                    y = oy + t * dy
                    if ymin <= y <= ymax:
                        ts.append(t)
        for y in (ymin, ymax):
            if dy != 0:
                t = (y - oy) / dy
                if t > 0:
                    x = ox + t * dx
                    if xmin <= x <= xmax:
                        ts.append(t)
        return ts

    def scans(self):
        angles = np.linspace(0, 2 * math.pi, self.n, endpoint=False)
        while True:
            ox, oy = 0.0, 0.0   # stationary bench test
            dist = np.array([self._cast(ox, oy, a) for a in angles])
            dist += np.random.normal(0, self.noise, size=dist.shape)
            q = np.full(self.n, 200.0)
            self._t += 1
            time.sleep(0.07)    # ~14 Hz like the real sensor
            yield angles.copy(), dist, q


# --------------------------------------------------------------------------- #
# Occupancy-grid SLAM
# --------------------------------------------------------------------------- #

class OccupancyGridSLAM:
    def __init__(self, size_m=16.0, res=0.03, max_range=12.0):
        self.res = res
        self.max_range = max_range
        self.n = int(size_m / res)
        self.origin = self.n // 2            # grid index of world (0,0)
        self.log = np.zeros((self.n, self.n), dtype=np.float32)
        self.L_OCC, self.L_FREE, self.CLAMP = 0.85, 0.4, 6.0
        self.pose = np.array([0.0, 0.0, 0.0])  # x, y, theta (m, m, rad)
        self._prev_pts = None

    # --- coordinate helpers --------------------------------------------------
    def _to_grid(self, x, y):
        ix = np.round(x / self.res).astype(int) + self.origin
        iy = np.round(y / self.res).astype(int) + self.origin
        return ix, iy

    @staticmethod
    def _polar_to_xy(ang, dist):
        return np.column_stack((dist * np.cos(ang), dist * np.sin(ang)))

    # --- scan matching (ICP) -------------------------------------------------
    def _icp(self, cur_pts, prev_pts, iters=8):
        """Estimate the rigid (dx, dy, dtheta) that aligns cur onto prev."""
        if prev_pts is None or len(cur_pts) < 30 or len(prev_pts) < 30:
            return 0.0, 0.0, 0.0
        theta, tx, ty = 0.0, 0.0, 0.0
        src = cur_pts.copy()
        for _ in range(iters):
            # Nearest neighbour by brute force (scans are small).
            d2 = ((src[:, None, :] - prev_pts[None, :, :]) ** 2).sum(-1)
            idx = d2.argmin(1)
            tgt = prev_pts[idx]
            keep = d2[np.arange(len(src)), idx] < (0.25 ** 2)
            if keep.sum() < 20:
                break
            s, t = src[keep], tgt[keep]
            sc, tc = s.mean(0), t.mean(0)
            H = (s - sc).T @ (t - tc)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = Vt.T @ U.T
            tt = tc - R @ sc
            src = (R @ src.T).T + tt
            dth = math.atan2(R[1, 0], R[0, 0])
            theta += dth
            tx, ty = (R @ np.array([tx, ty])) + tt
            if abs(dth) < 1e-4 and np.hypot(*tt) < 1e-4:
                break
        # Reject implausible jumps (keeps a stationary sensor pinned).
        if abs(theta) > 0.5 or math.hypot(tx, ty) > 0.5:
            return 0.0, 0.0, 0.0
        return tx, ty, theta

    # --- main update ---------------------------------------------------------
    def update(self, ang, dist, lock_pose=False):
        pts_local = self._polar_to_xy(ang, dist)

        if not lock_pose and self._prev_pts is not None:
            dx, dy, dth = self._icp(pts_local, self._prev_pts)
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            self.pose[0] += c * dx - s * dy
            self.pose[1] += s * dx + c * dy
            self.pose[2] += dth
        self._prev_pts = pts_local

        # Transform endpoints into world frame.
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        wx = self.pose[0] + c * pts_local[:, 0] - s * pts_local[:, 1]
        wy = self.pose[1] + s * pts_local[:, 0] + c * pts_local[:, 1]

        self._integrate(self.pose[0], self.pose[1], wx, wy, dist)
        return self.pose.copy()

    def _integrate(self, px, py, wx, wy, dist):
        # Mark occupied endpoints (within range only).
        hit = dist < self.max_range
        ex, ey = self._to_grid(wx[hit], wy[hit])
        inb = (ex >= 0) & (ex < self.n) & (ey >= 0) & (ey < self.n)
        np.add.at(self.log, (ey[inb], ex[inb]), self.L_OCC)

        # Mark free space by sampling along each ray (vectorised).
        n_samp = max(2, int(self.max_range / self.res) // 2)
        t = np.linspace(0.0, 0.95, n_samp)[None, :]          # exclude endpoint
        fx = (px + (wx - px)[:, None] * t).ravel()
        fy = (py + (wy - py)[:, None] * t).ravel()
        gx, gy = self._to_grid(fx, fy)
        inb = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        np.add.at(self.log, (gy[inb], gx[inb]), -self.L_FREE)

        np.clip(self.log, -self.CLAMP, self.CLAMP, out=self.log)

    def prob_map(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log))   # occupancy probability


# --------------------------------------------------------------------------- #
# Visualisation
# --------------------------------------------------------------------------- #

def run_slam(source, lock_pose=False, title="LD500 SLAM"):
    import matplotlib.pyplot as plt

    slam = OccupancyGridSLAM()
    extent = [-slam.origin * slam.res, (slam.n - slam.origin) * slam.res] * 2

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title(title)
    img = ax.imshow(slam.prob_map(), cmap="bone_r", origin="lower",
                    extent=extent, vmin=0, vmax=1)
    scan_dots, = ax.plot([], [], ".", ms=1.5, color="#ff5252", alpha=0.7)
    pose_dot, = ax.plot([], [], "o", ms=9, color="#2196f3")
    heading, = ax.plot([], [], "-", lw=2, color="#2196f3")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(title)
    fig.tight_layout()

    t0 = time.time()
    n = 0
    try:
        for ang, dist, q in source:
            pose = slam.update(ang, dist, lock_pose=lock_pose)
            n += 1

            img.set_data(slam.prob_map())
            c, s = math.cos(pose[2]), math.sin(pose[2])
            lx = pose[0] + c * dist * np.cos(ang) - s * dist * np.sin(ang)
            ly = pose[1] + s * dist * np.cos(ang) + c * dist * np.sin(ang)
            scan_dots.set_data(lx, ly)
            pose_dot.set_data([pose[0]], [pose[1]])
            heading.set_data([pose[0], pose[0] + 0.4 * c], [pose[1], pose[1] + 0.4 * s])

            if n % 5 == 0:
                hz = n / (time.time() - t0)
                ax.set_title(f"{title}   scans={n}  {hz:4.1f} Hz   "
                             f"pose=({pose[0]:+.2f}, {pose[1]:+.2f}, {math.degrees(pose[2]):+.0f}°)")
            plt.pause(0.001)
            if not plt.fignum_exists(fig.number):
                break
    except KeyboardInterrupt:
        pass
    print(f"\nStopped after {n} scans.")
    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.show()


def run_raw(source, title="LD500 raw scan"):
    import matplotlib.pyplot as plt

    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    fig.canvas.manager.set_window_title(title)
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    dots, = ax.plot([], [], ".", ms=2, color="#ff5252")
    ax.set_rmax(6)
    ax.set_title(title)
    try:
        for ang, dist, q in source:
            dots.set_data(ang, dist)
            ax.set_title(f"{title}   {len(dist)} pts")
            plt.pause(0.001)
            if not plt.fignum_exists(fig.number):
                break
    except KeyboardInterrupt:
        pass
    plt.ioff()


# --------------------------------------------------------------------------- #
# Port auto-detection
# --------------------------------------------------------------------------- #

def autodetect_port(baud=DEFAULT_BAUD):
    import serial

    cands = [p.device for p in list_ports()]
    # Prefer typical USB-serial bridges and COM5 (the user's "PORT 5").
    cands.sort(key=lambda d: (0 if d.upper() == "COM5" else 1, d))
    for dev in cands:
        try:
            with serial.Serial(dev, baud, timeout=0.5) as s:
                data = s.read(512)
                if bytes([HEADER, VERLEN]) in data:
                    print(f"Detected LD500 on {dev}")
                    return dev
        except Exception:
            continue
    return None


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser(description="LD500 LiDAR SLAM")
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--raw", action="store_true", help="show live point cloud only")
    ap.add_argument("--simulate", action="store_true", help="synthetic room, no hardware")
    ap.add_argument("--lock-pose", action="store_true", help="disable scan-matching")
    ap.add_argument("--no-crc", action="store_true", help="skip CRC validation")
    args = ap.parse_args()

    if args.list:
        ports = list_ports()
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print(f"  {p.device:8s} {p.description}  [{p.hwid}]")
        return

    if args.simulate:
        print("Running in SIMULATE mode (no hardware). Close the window to stop.")
        src = SimulatedLiDAR().scans()
        run_raw(src, "LD500 raw (sim)") if args.raw else run_slam(src, args.lock_pose, "LD500 SLAM (sim)")
        return

    port = args.port or autodetect_port(args.baud)
    if not port:
        print("ERROR: no LD500 found on any serial port.", file=sys.stderr)
        print("  - Check the USB cable / power and that the rotor is spinning.", file=sys.stderr)
        print("  - Run 'python lidar_slam.py --list' to see ports.", file=sys.stderr)
        print("  - Or try it without hardware: 'python lidar_slam.py --simulate'.", file=sys.stderr)
        sys.exit(2)

    print(f"Opening {port} @ {args.baud} baud ...  (Ctrl-C or close window to stop)")
    drv = LD500Driver(port, args.baud, check_crc=not args.no_crc)
    try:
        if args.raw:
            run_raw(drv.scans(), f"LD500 raw ({port})")
        else:
            run_slam(drv.scans(), args.lock_pose, f"LD500 SLAM ({port})")
    finally:
        drv.close()
        if drv._packets:
            print(f"{drv._packets} packets, {drv._bad_crc} CRC errors.")


if __name__ == "__main__":
    main()
