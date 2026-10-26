#!/usr/bin/env python3
"""Mock PBx-A1 camera streaming endpoint, for testing without a sensor.

Serves GET /api/v2/camera/data?cam=CAMx&data=HIRES_VID&rate=1 as a never-ending multipart/x-mixed-replace
stream (chunked), in the format that fpsdk::common::cam::CamStream parses (X-Fp-Meta with ts/dt in integer ns,
firmware 2.147+). The frames come from a raw H.265/H.264 Annex-B file, which is split into
access units (one part per picture, VPS/SPS/PPS are kept together with the IDR picture, like the sensor does).

CamStream hardcodes port 80 (no host:port support), so this needs to run as root on port 80, or use --port with a
patched SDK.

    ffmpeg -f lavfi -i testsrc2=size=1024x768:rate=25 -t 4 -c:v libx265 -x265-params \
        keyint=25:bframes=0:repeat-headers=1:log-level=error -f hevc test.h265
    ./mock_sensor.py --file test.h265 --codec h265 --port 80 [--drop-after N] [--start-at-p]

Replay of a real recording (.fpl from a PBx-A1): extract the camera data with the Fixposition SDK's fpltool and replay the
frames with their real meta data (seq, frame times, dt, frm, size) and real timing, looping at the end:

    fpltool extract -e jsonl some.fpl          # writes some.jsonl (all streams, includes the CAMDATA records)
    ./mock_sensor.py --jsonl some.jsonl --cam CAM1 --data HIRES_VID --port 80
"""
import argparse
import base64
import json
import socket
import threading
import time


def split_nals(buf):
    """Split Annex-B byte stream into NAL units (without start codes)."""
    idx = []
    i = 0
    n = len(buf)
    while i + 3 <= n:
        if buf[i] == 0 and buf[i + 1] == 0 and buf[i + 2] == 1:
            idx.append(i)
            i += 3
        else:
            i += 1
    nals = []
    for k, s in enumerate(idx):
        e = idx[k + 1] if k + 1 < len(idx) else n
        nal = buf[s + 3:e]
        nals.append(nal.rstrip(b"\x00") if k + 1 < len(idx) else nal)
    return nals


def access_units(buf, codec):
    """Group NAL units into access units. Returns [(is_idr, bytes)]."""
    aus = []
    cur = []
    cur_has_vcl = False
    cur_idr = False
    for nal in split_nals(buf):
        if codec == "h265":
            t = (nal[0] >> 1) & 0x3F
            is_vcl = t < 32
            is_idr = 16 <= t <= 23
        else:
            t = nal[0] & 0x1F
            is_vcl = 1 <= t <= 5
            is_idr = t == 5
        # A non-VCL NAL after a VCL NAL starts the next access unit
        if cur and cur_has_vcl and not is_vcl:
            aus.append((cur_idr, b"".join(b"\x00\x00\x00\x01" + x for x in cur)))
            cur, cur_has_vcl, cur_idr = [], False, False
        # Second VCL NAL: also a new access unit (single slice per picture)
        if cur and cur_has_vcl and is_vcl:
            aus.append((cur_idr, b"".join(b"\x00\x00\x00\x01" + x for x in cur)))
            cur, cur_has_vcl, cur_idr = [], False, False
        cur.append(nal)
        if is_vcl:
            cur_has_vcl = True
            cur_idr = is_idr
    if cur:
        aus.append((cur_idr, b"".join(b"\x00\x00\x00\x01" + x for x in cur)))
    return aus


def chunk(data):
    return b"%x\r\n" % len(data) + data + b"\r\n"


def serve_client(conn, aus, args, width, height):
    try:
        req = b""
        while b"\r\n\r\n" not in req:
            d = conn.recv(4096)
            if not d:
                return
            req += d
        line = req.split(b"\r\n")[0].decode()
        print("request:", line, flush=True)
        if "/api/v2/camera/data" not in line:
            conn.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 34\r\n\r\n"
                         b'{"_ok":false,"_message":"request fail"}'[:34])
            return
        boundary = "randomword"
        conn.sendall(("HTTP/1.1 200 OK\r\nServer: fixposition-httpd/1.0.0\r\nCache-Control: no-store\r\n"
                      "Pragma: no-cache\r\nContent-type: multipart/x-mixed-replace;boundary=%s\r\n"
                      "Access-Control-Allow-Origin: *\r\nTransfer-Encoding: chunked\r\n\r\n" % boundary).encode())
        fmt = "H265_NAL" if args.codec == "h265" else "H264_NAL"
        ctype = "video/hevc" if args.codec == "h265" else "video/avc"
        seq = args.seq0
        k = 0
        sent = 0
        if args.start_at_p:  # connect "mid-GOP": skip to first P frame
            while aus[k % len(aus)][0]:
                k += 1
            k += 1
        t0 = time.time()
        ts0 = time.time_ns() - 20_000_000
        while True:
            is_idr, payload = aus[k % len(aus)]
            k += 1
            if args.skip_seq and sent == args.skip_seq:
                seq += 3  # only the counter jumps
            if args.lose_after and sent == args.lose_after:
                k += 1  # a frame that is never sent
                sent += 1
                seq += 1
                is_idr, payload = aus[k % len(aus)]
                k += 1
            ts = ts0 + sent * 40_000_000  # exact 25Hz frame times (like the sensor), ~20ms ago (mid exposure)
            meta = ("ts=%d; dt=%d; seq=%d; fmt=%s; frm=%s; width=%d; height=%d; hstride=0; vstride=0; planes=1; "
                    "again=6.02" % (ts, 5174000, seq, fmt, "I_FRAME" if is_idr else "P_FRAME", width, height))
            part = ("--%s\r\nContent-type: %s\r\nX-Fp-Meta: %s\r\nContent-length: %d\r\n\r\n" %
                    (boundary, ctype, meta, len(payload))).encode() + payload + b"\r\n\r\n"
            conn.sendall(chunk(part))
            seq += 1
            sent += 1
            if args.drop_after and sent >= args.drop_after:
                print("dropping connection after %d frames" % sent, flush=True)
                return
            if args.stall_after and sent >= args.stall_after:
                print("stalling after %d frames (connection stays open)" % sent, flush=True)
                time.sleep(args.stall_secs)
                return
            # 25 Hz
            nxt = t0 + sent / 25.0
            time.sleep(max(0.0, nxt - time.time()))
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        conn.close()


def load_jsonl(path, cam, data_type):
    """Load CAMDATA records of one camera/stream from an fpltool jsonl. Returns [dict(meta..., payload)]."""
    frames = []
    with open(path, "r") as f:
        for line in f:
            if '"CAMDATA"' not in line:
                continue
            r = json.loads(line)
            if r.get("_type") != "CAMDATA" or r.get("cam_id") != cam or r.get("type") != data_type:
                continue
            if r.get("fmt") not in ("H265_NAL", "H264_NAL") or not r.get("data_b64"):
                continue
            frames.append(dict(fmt=r["fmt"], frm=r["frm"], width=r["width"], height=r["height"], seq=r["seq"],
                               ts=r["ts"], dt=r.get("dt", 0), payload=base64.b64decode(r["data_b64"])))
    return frames


def serve_client_jsonl(conn, frames, args):
    """Replay real frames: original seq, original frame intervals (from ts), ts shifted to 'now'."""
    try:
        req = b""
        while b"\r\n\r\n" not in req:
            d = conn.recv(4096)
            if not d:
                return
            req += d
        print("request:", req.split(b"\r\n")[0].decode(), flush=True)
        boundary = "randomword"
        conn.sendall(("HTTP/1.1 200 OK\r\nServer: fixposition-httpd/1.0.0\r\nCache-Control: no-store\r\n"
                      "Pragma: no-cache\r\nContent-type: multipart/x-mixed-replace;boundary=%s\r\n"
                      "Access-Control-Allow-Origin: *\r\nTransfer-Encoding: chunked\r\n\r\n" % boundary).encode())
        n = len(frames)
        span_ts = frames[-1]["ts"] - frames[0]["ts"] + (frames[1]["ts"] - frames[0]["ts"] if n > 1 else 40_000_000)
        span_seq = frames[-1]["seq"] - frames[0]["seq"] + 1
        k = 0
        if args.start_at_p:
            while frames[k % n]["frm"] == "I_FRAME":
                k += 1
            k += 1
        t0 = time.time()
        ts_first = frames[k % n]["ts"] + (k // n) * span_ts
        ts_base = time.time_ns() - 20_000_000
        sent = 0
        while True:
            fr = frames[k % n]
            loop = k // n
            ts_orig = fr["ts"] + loop * span_ts
            seq = fr["seq"] + loop * span_seq
            ts = ts_base + (ts_orig - ts_first)  # the recorded frame times, shifted to "now"
            ctype = "video/hevc" if fr["fmt"] == "H265_NAL" else "video/avc"
            meta = ("ts=%d; dt=%d; seq=%d; fmt=%s; frm=%s; width=%d; height=%d; hstride=0; vstride=0; planes=1" %
                    (ts, fr["dt"], seq, fr["fmt"], fr["frm"], fr["width"], fr["height"]))
            part = ("--%s\r\nContent-type: %s\r\nX-Fp-Meta: %s\r\nContent-length: %d\r\n\r\n" %
                    (boundary, ctype, meta, len(fr["payload"]))).encode() + fr["payload"] + b"\r\n\r\n"
            conn.sendall(chunk(part))
            k += 1
            sent += 1
            if args.drop_after and sent >= args.drop_after:
                print("dropping connection after %d frames" % sent, flush=True)
                return
            nxt = frames[k % n]["ts"] + (k // n) * span_ts
            time.sleep(max(0.0, t0 + (nxt - ts_first) * 1e-9 - time.time()))
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", help="raw Annex-B H.265/H.264 file")
    ap.add_argument("--jsonl", help="fpltool jsonl of a real recording (replaces --file)")
    ap.add_argument("--cam", default="CAM1")
    ap.add_argument("--data", default="HIRES_VID")
    ap.add_argument("--codec", choices=["h265", "h264"], default="h265")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--width", type=int, default=1024)
    ap.add_argument("--height", type=int, default=768)
    ap.add_argument("--seq0", type=int, default=236159)
    ap.add_argument("--drop-after", type=int, default=0, help="close connection after N frames (every client)")
    ap.add_argument("--start-at-p", action="store_true", help="first frame sent is a P frame")
    ap.add_argument("--skip-seq", type=int, default=0, help="introduce a seq gap (but no missing frame) after N frames")
    ap.add_argument("--stall-after", type=int, default=0, help="stop sending after N frames but keep the connection open")
    ap.add_argument("--stall-secs", type=float, default=3600.0)
    ap.add_argument("--lose-after", type=int, default=0, help="really lose one frame after N frames (seq and time jump)")
    args = ap.parse_args()

    frames = aus = None
    if args.jsonl:
        frames = load_jsonl(args.jsonl, args.cam, args.data)
        if not frames:
            raise SystemExit("no %s %s CAMDATA frames in %s" % (args.cam, args.data, args.jsonl))
        print("%d real frames (%s %s), %d I" % (len(frames), args.cam, args.data,
                                               sum(1 for x in frames if x["frm"] == "I_FRAME")), flush=True)
    else:
        if not args.file:
            raise SystemExit("need --file or --jsonl")
        aus = access_units(open(args.file, "rb").read(), args.codec)
        print("%d access units, %d IDR" % (len(aus), sum(1 for a in aus if a[0])), flush=True)

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(5)
    print("listening on %s:%d" % (args.bind, args.port), flush=True)
    while True:
        conn, _ = srv.accept()
        if frames:
            threading.Thread(target=serve_client_jsonl, args=(conn, frames, args), daemon=True).start()
        else:
            threading.Thread(target=serve_client, args=(conn, aus, args, args.width, args.height), daemon=True).start()


if __name__ == "__main__":
    main()
