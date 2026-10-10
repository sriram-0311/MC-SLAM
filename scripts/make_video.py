#!/usr/bin/env python3
"""Compose a 1080p H.264 video from a mc_slam_app --record_dir recording.

Left: the map as it was being built (keyframe point snapshots, rendered here with an orbiting
view), the trajectory estimated so far and the current camera. Right: the stereo tracking view
and a live plot of the per-frame processing time. The video clock is the measured processing time
(frames.csv) sped up by --speed, so the playback rate on screen is what the system really did.

    scripts/make_video.py <record_dir> <out.mp4> --title "MC-SLAM · EuRoC MH_01_easy" --speed 3

Needs python3 with numpy, opencv and pillow, and gst-launch-1.0 with x264enc and mp4mux.
"""
import argparse
import bisect
import csv
import os
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1920, 1080, 30
BG = (14, 16, 20)
PANEL = (24, 26, 32)
FG = (232, 234, 238)
DIM = (140, 146, 156)
TRACK = (86, 180, 233)      # tracking frames
KEYF = (240, 170, 60)       # keyframes
LIMIT = (220, 100, 100)     # camera period
TRAJ = (255, 214, 102)      # trajectory (all colours are RGB)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
_fonts = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in _fonts:
        _fonts[key] = ImageFont.truetype(FONT_B if bold else FONT, size)
    return _fonts[key]


def load(rec):
    rows = [r for r in csv.DictReader(open(os.path.join(rec, "frames.csv")))
            if r.get("tz") not in (None, "") and float(r["timestamp"]) > 0]
    f = lambda k: np.array([float(r[k]) for r in rows])
    ids = np.array([int(r["frame"]) for r in rows])
    R = np.stack([np.array([[float(r[f"r{i}{j}"]) for j in range(3)] for i in range(3)]) for r in rows])
    C = np.stack([f("tx"), f("ty"), f("tz")], 1)
    snaps = sorted(int(n[7:13]) for n in os.listdir(rec) if n.startswith("points_"))
    return ids, f("timestamp"), f("process_ms"), f("keyframe") > 0, f("lost") > 0, R, C, snaps


class MapRenderer:
    """Point-splat renderer: points coloured by height, density-weighted, with an orbiting camera."""

    def __init__(self, rec, C, snaps, w, h):
        self.rec, self.w, self.h, self.snaps = rec, w, h, snaps
        self.cache = (None, None)
        # VO world frame = first camera (x right, y down, z forward); the rig flies roughly level,
        # so -y is close to up. Build a z-up frame from it.
        self.A = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
        Cw = C @ self.A.T
        last = self.points(snaps[-1]) if snaps else Cw
        lo, hi = np.percentile(last, 5, 0), np.percentile(last, 95, 0)
        self.box = (lo - 0.5, hi + 0.5)
        self.zlo, self.zhi = np.percentile(last[:, 2], 5), np.percentile(last[:, 2], 95)
        self.target, self.radius = None, None      # smoothed framing
        self.f = 0.8 * w
        lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8)[:, None], cv2.COLORMAP_PLASMA)[:, 0, ::-1]
        self.lut = lut.astype(np.float32) / 255.0

    def points(self, k):
        if self.cache[0] != k:
            p = np.fromfile(os.path.join(self.rec, f"points_{k:06d}.bin"), np.float32).reshape(-1, 3)
            p = p[np.isfinite(p).all(1)].astype(np.float64) @ self.A.T
            self.cache = (k, p)
        return self.cache[1]

    def view(self, t_s, Cw, i, P):
        """Orbit around what has been mapped so far; target and distance follow it smoothly."""
        traj = Cw[: i + 1]
        final = i == len(Cw) - 1                    # end hold: frame the whole map
        if final and P is not None and len(P):
            tgt = 0.5 * (np.percentile(P, 10, 0) + np.percentile(P, 90, 0))
        else:
            tgt = 0.7 * np.median(traj, 0) + 0.3 * traj[-1]
        ext = np.linalg.norm(traj - tgt, axis=1)
        r = np.percentile(ext, 95) if len(ext) > 10 else 0.0
        if P is not None and len(P):
            sub = P[:: max(1, len(P) // 4000)]
            r = max(r, 0.6 * np.percentile(np.linalg.norm(sub - tgt, axis=1), 80))
        r = max((2.6 if final else 2.2) * r, 5.0)
        if self.target is None:
            self.target, self.radius = tgt, r
        self.target = 0.96 * self.target + 0.04 * tgt
        self.radius = 0.98 * self.radius + 0.02 * r
        az = np.deg2rad(-60 + 6.0 * t_s)           # slow orbit
        el = np.deg2rad(40)
        eye = self.target + self.radius * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        fwd = self.target - eye
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right)
        down = np.cross(fwd, right)
        return eye, np.stack([right, down, fwd])

    def project(self, P, eye, Rv):
        q = (P - eye) @ Rv.T
        ok = q[:, 2] > 0.3
        uv = self.f * q[:, :2] / np.maximum(q[:, 2:3], 1e-6) + [self.w / 2, self.h / 2]
        return uv, ok, q[:, 2]

    def render(self, frame_id, R, C, i, t_s):
        Cw = C @ self.A.T
        acc = np.zeros((self.h, self.w, 3), np.float32)
        k = bisect.bisect_right(self.snaps, frame_id) - 1
        P = None
        if k >= 0:
            P = self.points(self.snaps[k])
            P = P[((P >= self.box[0]) & (P <= self.box[1])).all(1)]
        eye, Rv = self.view(t_s, Cw, i, P)
        if P is not None:
            uv, ok, _ = self.project(P, eye, Rv)
            u, v = uv[:, 0].astype(int), uv[:, 1].astype(int)
            ok &= (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
            zc = np.clip((P[ok, 2] - self.zlo) / max(1e-6, self.zhi - self.zlo), 0, 1)
            col = self.lut[(70 + zc * 185).astype(int)]
            np.add.at(acc, (v[ok], u[ok]), col)
        img = 1.0 - np.exp(-1.6 * acc)                          # tone-map density
        img = np.clip(img + 0.35 * cv2.GaussianBlur(img, (0, 0), 3), 0, 1)
        out = (np.array(BG[::-1], np.float32) / 255 * (1 - img) + img)
        out = (np.clip(out, 0, 1) * 255).astype(np.uint8)
        # trajectory so far, with a soft glow
        uv, ok, _ = self.project(Cw[: i + 1], eye, Rv)
        pts = uv[ok].astype(np.int32).reshape(-1, 1, 2)
        if len(pts) > 1:
            glow = np.zeros_like(out)
            cv2.polylines(glow, [pts], False, TRAJ[::-1], 7, cv2.LINE_AA)
            out = cv2.add(out, (cv2.GaussianBlur(glow, (0, 0), 4) * 0.6).astype(np.uint8))
            cv2.polylines(out, [pts], False, TRAJ[::-1], 2, cv2.LINE_AA)
        # current camera frustum
        s = 0.03 * self.radius
        corners = np.array([[0, 0, 0], [-1, -0.7, 1.4], [1, -0.7, 1.4], [1, 0.7, 1.4], [-1, 0.7, 1.4]]) * s
        Fw = (corners @ R[i].T + C[i]) @ self.A.T
        fuv, fok, _ = self.project(Fw, eye, Rv)
        if fok.all():
            p = fuv.astype(int)
            for a, b in [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]:
                cv2.line(out, tuple(p[a]), tuple(p[b]), (255, 255, 255), 2, cv2.LINE_AA)
        return out, (0 if k < 0 else len(self.points(self.snaps[k])))


def legend(d, x, y, items, mark):
    for col, lab in items:
        d.text((x, y), mark, font=font(17), fill=col)
        d.text((x + 22, y), lab, font=font(17), fill=DIM)
        x += 22 + d.textlength(lab, font=font(17)) + 26


def draw_plot(d, box, t, kf, i, period_ms, window=240, ymax=500):
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=PANEL)
    lo = max(0, i - window + 1)
    px = lambda k: x0 + 64 + (k - lo) / (window - 1) * (x1 - x0 - 70)
    py = lambda v: y1 - 8 - min(v, ymax) / ymax * (y1 - y0 - 16)
    for v in (0, 100, 200, 300, 400, 500):
        d.line([(x0 + 64, py(v)), (x1 - 6, py(v))], fill=(40, 43, 52))
        d.text((x0 + 58, py(v)), f"{v} ms" if v == ymax else f"{v}", font=font(15), fill=DIM, anchor="rm")
    for k in range(lo, i + 1):
        d.line([(px(k), py(0)), (px(k), py(t[k]))], fill=KEYF if kf[k] else TRACK, width=3)
    d.line([(x0 + 64, py(period_ms)), (x1 - 6, py(period_ms))], fill=LIMIT, width=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("out")
    ap.add_argument("--title", default="MC-SLAM")
    ap.add_argument("--subtitle", default="stereo visual odometry · factor-graph back end · CPU only")
    ap.add_argument("--speed", type=float, default=3.0, help="playback speed over measured processing time")
    ap.add_argument("--camera_hz", type=float, default=20.0)
    ap.add_argument("--hold_s", type=float, default=4.0, help="hold (and keep orbiting) the last frame")
    ap.add_argument("--max_s", type=float, default=0, help="stop after this many seconds of video (preview)")
    a = ap.parse_args()

    ids, ts, t, kf, lost, R, C, snaps = load(a.rec)
    n = len(t)
    cum = np.cumsum(t)
    total_s, data_s = cum[-1] / 1e3, ts[-1] - ts[0]
    n_out = int(total_s / a.speed * FPS) + int(a.hold_s * FPS)
    if a.max_s:
        n_out = min(n_out, int(a.max_s * FPS))
    med_track = float(np.median(t[~kf]))
    med_kf = float(np.median(t[kf]))
    print(f"{n} frames, {data_s:.0f} s of data, {total_s:.0f} s processing, {n_out / FPS:.0f} s of video;"
          f" median tracking {med_track:.1f} ms, keyframe {med_kf:.0f} ms")

    PAD, HEAD = 24, 96
    MAPW, MAPH = 1060, H - HEAD - PAD
    RX = PAD + MAPW + PAD
    RW = W - RX - PAD
    mr = MapRenderer(a.rec, C, snaps, MAPW, MAPH)

    gst = ["gst-launch-1.0", "-q", "fdsrc", "fd=0", f"blocksize={W * H * 3}", "!",
           "rawvideoparse", f"width={W}", f"height={H}", "format=bgr", f"framerate={FPS}/1", "!",
           "videoconvert", "!", "video/x-raw,format=I420", "!",
           "x264enc", "speed-preset=slow", "bitrate=14000", "key-int-max=60", "!",
           "video/x-h264,profile=high", "!", "mp4mux", "!", "filesink", f"location={a.out}"]
    enc = subprocess.Popen(gst, stdin=subprocess.PIPE)

    last_track = None
    for j in range(n_out):
        t_s = j / FPS
        i = min(n - 1, bisect.bisect_left(cum, t_s * a.speed * 1e3))
        frame = np.full((H, W, 3), BG[::-1], np.uint8)
        mimg, npts = mr.render(ids[i], R, C, i, t_s)
        frame[HEAD:HEAD + MAPH, PAD:PAD + MAPW] = mimg

        p = os.path.join(a.rec, f"track_{ids[i]:06d}.jpg")
        if os.path.exists(p):
            last_track = cv2.imread(p)
        th = 0
        if last_track is not None:
            tv = last_track[28:]                                   # drop the app's text bar
            th = round(tv.shape[0] * RW / tv.shape[1])
            frame[HEAD:HEAD + th, RX:RX + RW] = cv2.resize(tv, (RW, th), interpolation=cv2.INTER_AREA)

        pil = Image.fromarray(frame[:, :, ::-1])
        d = ImageDraw.Draw(pil)
        d.text((PAD, 20), a.title, font=font(36, True), fill=FG)
        d.text((PAD, 64), a.subtitle, font=font(20), fill=DIM)
        d.text((W - PAD, 22), f"{a.speed:g}× speed", font=font(32, True), fill=FG, anchor="ra")
        d.text((W - PAD, 64), "clock = measured processing time", font=font(18), fill=DIM, anchor="ra")
        d.text((PAD + 16, HEAD + 14), f"map: {npts:,} points", font=font(18), fill=DIM)
        d.text((PAD + 16, HEAD + MAPH - 34), "trajectory", font=font(18, True), fill=TRAJ)

        y = HEAD + th + 10
        legend(d, RX, y, [((60, 200, 60), "tracked map points"), ((220, 60, 60), "rejected by RANSAC"),
                          ((150, 150, 150), "new features")], "●")
        y += 52
        d.text((RX, y), "Processing time per frame", font=font(22, True), fill=FG)
        y += 34
        legend(d, RX, y, [(TRACK, "tracking"), (KEYF, "keyframe + back end"), (LIMIT, "20 Hz camera period")], "■")
        pbox = (RX, y + 32, RX + RW, y + 382)
        draw_plot(d, pbox, t, kf, i, 1e3 / a.camera_hz)

        sy = pbox[3] + 34
        stats = [(f"{ts[i] - ts[0]:.0f} / {data_s:.0f} s", "sequence time"),
                 (f"{int(kf[:i + 1].sum())}", "keyframes"),
                 (f"{med_track:.0f} ms", "median tracking"),
                 (f"{med_kf:.0f} ms", "median keyframe")]
        cx = [RX, RX + int(0.30 * RW), RX + int(0.50 * RW), RX + int(0.75 * RW)]
        for k, (val, lab) in enumerate(stats):
            cw = cx[k] - RX
            d.text((RX + cw, sy), val, font=font(30, True), fill=FG)
            d.text((RX + cw, sy + 40), lab, font=font(17), fill=DIM)

        enc.stdin.write(np.asarray(pil)[:, :, ::-1].tobytes())
        if j % 300 == 0:
            print(f"  {j}/{n_out}", flush=True)

    enc.stdin.close()
    enc.wait()
    print("wrote", a.out)


if __name__ == "__main__":
    main()
