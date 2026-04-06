#!/usr/bin/env python3
"""
camera_stream.py
TCP-only frame sender for the Smart Parking system.

Sends grayscale frames to frame_feeder over TCP.

WIRE FORMAT
-----------
Per frame:
  [uint32 frame_id]
  [uint32 width]
  [uint32 height]
  [width * height bytes grayscale pixels]

All integers are little-endian.

USAGE
-----
  python3 camera_stream.py <source> --host <QNX_IP> --port <PORT> [options]

EXAMPLES
--------
  python3 camera_stream.py carPark.mp4 --host 192.168.56.20 --port 5000
  python3 camera_stream.py carPark.mp4 --host 192.168.56.20 --port 5000 --width 640
  python3 camera_stream.py 0 --host 192.168.56.20 --port 5000 --fps 15

NOTES
-----
- <source> can be:
    * an MP4/video file path
    * a camera device index like 0 or 1
- Frames are converted to grayscale before sending.
- This script is intended to run on the Windows/host side.
"""

import argparse
import os
import socket
import struct
import sys
import time

import cv2

_HDR_PACK = struct.Struct("<III")


def parse_args():
    p = argparse.ArgumentParser(
        description="Stream grayscale frames over TCP to frame_feeder on QNX"
    )
    p.add_argument("source", help="MP4 file path or camera device index")
    p.add_argument("--host", required=True, help="QNX target IP address")
    p.add_argument("--port", type=int, required=True, help="QNX TCP port")
    p.add_argument("--fps", type=float, default=None, help="Output FPS limit")
    p.add_argument("--width", type=int, default=None, help="Resize width")
    p.add_argument("--height", type=int, default=None, help="Resize height")
    p.add_argument("--start", type=float, default=0.0, help="Start time in seconds (video only)")
    p.add_argument("--loop", action="store_true", help="Loop the video continuously")
    p.add_argument("--info", action="store_true", help="Print source info and exit")
    return p.parse_args()


def open_source(source_str):
    try:
        idx = int(source_str)
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera device {idx}")
        return cap, True
    except ValueError:
        pass

    if not os.path.exists(source_str):
        raise FileNotFoundError(f"File not found: {source_str}")

    cap = cv2.VideoCapture(source_str)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video file: {source_str}")
    return cap, False


def connect_tcp(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    except OSError:
        pass
    sock.connect((host, port))
    return sock


def send_frame(sock, frame_id, gray):
    h, w = gray.shape
    header = _HDR_PACK.pack(frame_id, w, h)
    sock.sendall(header)
    sock.sendall(gray.tobytes())


def compute_output_size(src_w, src_h, req_w, req_h):
    out_w, out_h = src_w, src_h

    if req_w and not req_h:
        out_w = req_w
        out_h = int(src_h * req_w / src_w)
    elif req_h and not req_w:
        out_h = req_h
        out_w = int(src_w * req_h / src_h)
    elif req_w and req_h:
        out_w, out_h = req_w, req_h

    return out_w, out_h


def stream(args):
    cap, is_camera = open_source(args.source)

    src_fps = cap.get(cv2.CAP_PROP_FPS)
    if not src_fps or src_fps <= 0:
        src_fps = 25.0

    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    src_label = f"camera:{args.source}" if is_camera else args.source
    print(f"[CS] source  : {src_label}", file=sys.stderr)
    print(f"[CS] native  : {src_w}x{src_h} @ {src_fps:.2f} fps", file=sys.stderr)
    if not is_camera:
        print(f"[CS] frames  : {src_frames}", file=sys.stderr)
    print(f"[CS] target  : {args.host}:{args.port}", file=sys.stderr)

    if args.info:
        cap.release()
        return

    out_w, out_h = compute_output_size(src_w, src_h, args.width, args.height)
    resize = (out_w != src_w or out_h != src_h)

    target_fps = args.fps if args.fps else src_fps
    interval_s = (1.0 / target_fps) if target_fps and target_fps > 0 else 0.0

    print(f"[CS] output  : {out_w}x{out_h} @ {target_fps:.2f} fps", file=sys.stderr)
    print(f"[CS] loop    : {'true' if args.loop else 'false'}", file=sys.stderr)

    if not is_camera and args.start > 0:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000.0)

    sock = connect_tcp(args.host, args.port)
    print("[CS] TCP connected", file=sys.stderr)

    frame_id = 0
    loops = 0
    t_next = time.monotonic()

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                if args.loop and not is_camera:
                    loops += 1
                    print(f"[CS] loop #{loops} – rewinding", file=sys.stderr)
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue

                print(f"[CS] source exhausted – {frame_id} frames sent", file=sys.stderr)
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if resize:
                gray = cv2.resize(gray, (out_w, out_h), interpolation=cv2.INTER_AREA)

            send_frame(sock, frame_id, gray)
            frame_id += 1

            if interval_s > 0:
                t_next += interval_s
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()

    finally:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()
        cap.release()
        print("[CS] done", file=sys.stderr)


if __name__ == "__main__":
    args = parse_args()
    try:
        stream(args)
    except KeyboardInterrupt:
        print("\n[CS] interrupted", file=sys.stderr)
        sys.exit(0)
    except Exception as e:
        print(f"[CS] error: {e}", file=sys.stderr)
        sys.exit(1)