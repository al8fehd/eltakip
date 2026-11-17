#!/usr/bin/env python3
"""Deneyap Hand Tracker — Phase 1 PC bridge.

serial (pyserial) -> framed stream -> pose math -> WebSocket broadcast +
HTTP-served self-contained UI. Single entry point:

    python3 server.py --sim          # synthetic stream, no hardware needed
    python3 server.py                # real Kart v2, auto-detected serial port
    python3 server.py --port /dev/ttyACM0

Data flow (pub/sub, Phase 2/3 ready):

    SerialSource / SimSource -> Pipeline (pub/sub) -> WsHub (browser UI)
                                       |
                                       +----> Phase 2/3 subscribers

A Phase 2 (sign language) or Phase 3 (air mouse) module is a plain callable
registered in main():

    async def my_module(pose): ...          # pose = same JSON dict as the UI
    pipeline.subscribe("phase2_gestures", my_module)

No Phase 1 code needs to change.

Env overrides: DENEYAP_SERIAL_PORT, DENEYAP_BAUD, DENEYAP_WS_PORT,
DENEYAP_HTTP_PORT, DENEYAP_HOST.
"""

from __future__ import annotations

import argparse
import asyncio
import inspect
import json
import logging
import math
import os
import sys
import threading
import time
from pathlib import Path

import protocol
import pose

log = logging.getLogger("bridge")

try:
    import serial
    import serial.tools.list_ports
except ImportError:                              # pyserial missing: hardware mode unavailable
    serial = None

try:
    from websockets.asyncio.server import serve as _ws_serve_new      # websockets >= 13
    WS = "new"
except ImportError:
    try:
        from websockets.server import serve as _ws_serve_old          # websockets < 13
        WS = "old"
    except ImportError:
        WS = "none"

# Import websockets.exceptions for proper exception handling
if WS != "none":
    try:
        from websockets.exceptions import ConnectionClosed
    except ImportError:
        # Dummy class if websockets is not installed
        class ConnectionClosed(Exception):
            pass
else:
    # Dummy class if websockets is not installed
    class ConnectionClosed(Exception):
        pass

WEB_DIR = Path(__file__).resolve().parent / "web"


# ---------------------------------------------------------------------------
# Pipeline: the pub/sub hub every pose passes through
# ---------------------------------------------------------------------------

class Pipeline:
    """Fans every processed pose out to subscribers (see module docstring).

    Subscribers are sync callables or coroutines taking one argument: the
    JSON-ready pose dict that WebSocket clients also receive.
    """

    def __init__(self, tracker: pose.PoseTracker, src_name: str):
        self.tracker = tracker
        self.src_name = src_name
        self._subs = {}
        self._rate_hz = 0.0
        self._last_pose = None
        self._last_t = None
        self.status_cb = None                    # async callable(status_dict)

    def subscribe(self, name: str, fn) -> None:
        if not callable(fn):
            raise TypeError("subscriber must be callable")
        self._subs[name] = fn
        log.info("subscriber added: %s", name)

    def unsubscribe(self, name: str) -> None:
        self._subs.pop(name, None)

    def latest(self):
        return self._last_pose

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    async def zero(self, source: str = "user") -> None:
        """Re-pin the neutral pose from the newest frame (UI command)."""
        if self.tracker.capture_neutral_from_last(source):
            log.info("neutral pose captured (%s)", source)
            await self._notify_status()

    async def set_deadzone(self, imu, deg) -> None:
        self.tracker.set_deadzone(imu, deg)
        log.info("deadzone updated: imu=%s deg=%.1f", imu, float(deg))
        await self._notify_status()

    async def reset_deadzones(self) -> None:
        self.tracker.reset_deadzones()
        log.info("deadzones reset to defaults")
        await self._notify_status()

    async def set_mount(self, yaw_deg, invert_yaw, invert_pitch, invert_roll) -> None:
        self.tracker.set_mount(yaw_deg, invert_yaw, invert_pitch, invert_roll)
        log.info("mount updated: %s", self.tracker.mount)
        await self._notify_status()

    # -- ingest -------------------------------------------------------------

    async def ingest_frame(self, frame) -> None:
        """Frame from the serial parser (or a synthesized one)."""
        pose_dict = self.tracker.on_frame(frame)
        if pose_dict is None:
            return
        now = time.time()
        if self._last_t:
            dt = now - self._last_t
            if 0.0 < dt < 1.0:
                a = 0.10
                self._rate_hz += a * (1.0 / dt - self._rate_hz)
        self._last_t = now
        pose_dict["rate_hz"] = round(self._rate_hz, 2)
        self._last_pose = pose_dict

        for name, fn in list(self._subs.items()):
            try:
                res = fn(pose_dict)
                if inspect.iscoroutine(res):
                    await res
            except Exception:
                log.exception("subscriber %r failed", name)

    # -- status -------------------------------------------------------------

    def status_dict(self) -> dict:
        t = self.tracker
        return {
            "type": "status",
            "t": time.time(),
            "src": self.src_name,
            "zeroed": t.zeroed,
            "zeroed_at": t.zeroed_at,
            "zero_source": t.zero_source,
            "rate_hz": round(self._rate_hz, 2),
            "deadzones_deg": list(t.deadzones_deg),
            "mount": t.mount,
        }

    async def _notify_status(self) -> None:
        if self.status_cb is None:
            return
        try:
            await self.status_cb(self.status_dict())
        except Exception:
            log.exception("status callback failed")


# ---------------------------------------------------------------------------
# Sim source: synthetic per-IMU world quaternions through the real pipeline
# ---------------------------------------------------------------------------

class SimSource:
    """Generates plausible glove motion and feeds it through the *same* path
    as hardware (per-IMU world quaternions -> relative math -> joints), so
    --sim exercises everything except the serial link."""

    def __init__(self, pipeline: Pipeline, hz: float = 60.0):
        self.pipeline = pipeline
        self.hz = hz

    async def run(self) -> None:
        t0 = time.perf_counter()
        seq = 0
        while True:
            t = time.perf_counter() - t0
            two_pi = 2.0 * math.pi

            # Slow whole-hand motion: yaw, pitch, roll in the world frame.
            hand = pose.qcompose([
                pose.qaxis_angle(0, 1, 0, 0.55 * math.sin(t * two_pi / 9.0)),
                pose.qaxis_angle(1, 0, 0, 0.30 * math.sin(t * two_pi / 13.0)),
                pose.qaxis_angle(0, 0, 1, 0.22 * math.sin(t * two_pi / 7.0)),
            ])
            quats = [hand]

            for i, finger in enumerate(pose.FINGERS):
                phase = t * (1.1 + 0.13 * i) + 1.3 * i
                curl = 0.5 + 0.5 * math.sin(phase)
                if finger == "thumb":
                    axis = (0.0, 0.0, 1.0)              # opposition plane
                    lower = 0.35 * curl
                    upper = 0.45 * math.sin(phase * 1.3 + 0.6)
                else:
                    axis = (1.0, 0.0, 0.0)              # flexion plane
                    lower = (0.45 + 0.08 * i) * curl
                    upper = (0.22 + 0.05 * i) * math.sin(phase * 1.6 + 0.8)

                q_lower = pose.qmul(hand, pose.qaxis_angle(*axis, -lower))
                if finger == "index":                   # subtle lateral sway
                    q_lower = pose.qmul(q_lower, pose.qaxis_angle(0, 0, 1, 0.15 * math.sin(t * 2.3)))
                q_upper = pose.qmul(q_lower, pose.qaxis_angle(*axis, -upper))

                lo, hi = pose.FINGERS[finger]
                quats.insert(lo, q_lower)
                quats.insert(hi, q_upper)

            frame = protocol.Frame(protocol.TYPE_POSE, seq & 0xFF, b"",
                                   flags=0x01, status_lo=0xFF, status_hi=0x07,
                                   quats=quats)
            await self.pipeline.ingest_frame(frame)
            seq += 1
            await asyncio.sleep(1.0 / self.hz)


# ---------------------------------------------------------------------------
# Serial source: hardware stream reader (blocking reads in a thread)
# ---------------------------------------------------------------------------

class SerialSource:
    """Reads the framed stream from the Deneyap Kart v2.

    A dedicated thread does the blocking serial reads; chunks cross into the
    event loop over an asyncio.Queue. The parser is incremental, so any byte
    misalignment (mid-frame connect, host restart, USB re-enumeration)
    self-heals, and the loop reconnects if the device disappears.
    """

    def __init__(self, pipeline: Pipeline, port: str, baud: int):
        if serial is None:
            raise SystemExit("pyserial is not installed — run: pip install -r requirements.txt")
        self.pipeline = pipeline
        self.port = port
        self.baud = baud

    async def run(self) -> None:
        while True:
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.05)
            except Exception as exc:
                log.warning("cannot open %s (%s) — retrying in 2 s", self.port, exc)
                await asyncio.sleep(2.0)
                continue

            parser = protocol.FrameParser()
            loop = asyncio.get_running_loop()
            q: asyncio.Queue = asyncio.Queue(maxsize=128)

            def reader():
                while ser.is_open:
                    try:
                        chunk = ser.read(4096)
                    except Exception:
                        break
                    if not chunk:
                        continue
                    try:
                        loop.call_soon_threadsafe(q.put_nowait, chunk)
                    except RuntimeError:
                        break

            threading.Thread(target=reader, name="serial-reader", daemon=True).start()
            log.info("serial connected: %s @ %d baud", self.port, self.baud)

            stats_t = time.time()
            try:
                while True:
                    chunk = await q.get()
                    for f in parser.feed(chunk):
                        await self._handle(f)
                    now = time.time()
                    if now - stats_t >= 5.0:
                        stats_t = now
                        log.info("link: frames=%d badcrc=%d resync=%d B rate=%.1f Hz",
                                 parser.frame_count, parser.bad_crc,
                                 parser.dropped_bytes, self.pipeline.rate_hz)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("serial error: %s", exc)
            finally:
                ser.close()
                log.info("serial disconnected: %s — reconnecting", self.port)
            await asyncio.sleep(1.0)

    async def _handle(self, f: protocol.Frame) -> None:
        if f.ftype == protocol.TYPE_LOG:
            text = f.payload.decode("ascii", errors="replace").rstrip("\r\n")
            log.info("[FW] %s", text)
        elif f.ftype == protocol.TYPE_POSE:
            await self.pipeline.ingest_frame(f)


# ---------------------------------------------------------------------------
# WebSocket hub + static HTTP server for the UI
# ---------------------------------------------------------------------------

class WsHub:
    """WebSocket endpoint for the browser UI: latest pose + status broadcast."""

    def __init__(self, pipeline: Pipeline, host: str, port: int):
        self.pipeline = pipeline
        self.host = host
        self.port = port
        self.clients: set = set()
        pipeline.subscribe("ws", self._on_pose)          # poses -> clients
        pipeline.status_cb = self._on_status             # status -> clients

    async def _on_pose(self, pose) -> None:
        await self.broadcast(pose)

    async def _on_status(self, status) -> None:
        await self.broadcast(status)

    async def _handler(self, ws) -> None:
        self.clients.add(ws)
        log.info("ws client connected (%d)", len(self.clients))
        try:
            await ws.send(json.dumps(self.pipeline.status_dict()))
            latest = self.pipeline.latest()
            if latest is not None:
                await ws.send(json.dumps(latest))
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                kind = msg.get("type")
                if kind == "zero":
                    await self.pipeline.zero("user")
                elif kind == "set_deadzone":
                    try:
                        await self.pipeline.set_deadzone(msg.get("imu"), msg.get("deg"))
                    except ValueError as exc:
                        await ws.send(json.dumps({"type": "error", "message": str(exc)}))
                elif kind == "reset_deadzones":
                    await self.pipeline.reset_deadzones()
                elif kind == "set_mount":
                    try:
                        await self.pipeline.set_mount(
                            msg.get("yaw_deg"),
                            msg.get("invert_yaw"),
                            msg.get("invert_pitch"),
                            msg.get("invert_roll"),
                        )
                    except ValueError as exc:
                        await ws.send(json.dumps({"type": "error", "message": str(exc)}))
                elif kind == "ping":
                    await ws.send(json.dumps({"type": "pong", "t": time.time()}))
        except ConnectionClosed:
            pass                                    # closed client, drop it
        except Exception:
            log.exception("websocket handler unexpected error")
            pass                                    # closed client, drop it
        finally:
            self.clients.discard(ws)
            log.info("ws client disconnected (%d)", len(self.clients))

    async def broadcast(self, obj) -> None:
        if not self.clients:
            return
        text = json.dumps(obj)
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def run(self) -> None:
        if WS == "none":
            raise SystemExit("websockets is not installed — run: pip install -r requirements.txt")
        if WS == "new":
            stop = _ws_serve_new(self._handler, self.host, self.port)
        else:
            # For compatibility with websockets < 13
            stop = _ws_serve_old(self._handler, self.host, self.port)
        async with stop:
            log.info("websocket: ws://%s:%d", self.host, self.port)
            await asyncio.Future()                  # run until cancelled

    # Note: _handler_legacy was removed as it was unused and identical to _handler


class HttpServer:
    """Minimal stdlib HTTP server for the self-contained UI in web/."""

    ALLOWED = {
        "/": "index.html",
        "/index.html": "index.html",
        "/app.js": "app.js",
        "/vendor/three.module.min.js": "vendor/three.module.min.js",
    }
    TYPES = {
        "html": "text/html; charset=utf-8",
        "js": "text/javascript; charset=utf-8",
        "css": "text/css; charset=utf-8",
        "json": "application/json",
        "svg": "image/svg+xml",
        "png": "image/png",
        "wasm": "application/wasm",
    }

    def __init__(self, root: Path, host: str, port: int):
        self.root = root
        self.host = host
        self.port = port

    async def run(self) -> None:
        server = await asyncio.start_server(self._on_conn, self.host, self.port)
        log.info("web UI: http://%s:%d  (web dir: %s)", self.host, self.port, self.root)
        async with server:
            await server.serve_forever()

    async def _on_conn(self, reader, writer) -> None:
        try:
            request = (await asyncio.wait_for(reader.read(2048), 5.0)).decode("ascii", errors="ignore")
            first = request.split("\r\n", 1)[0]
            parts = first.split(" ")
            if len(parts) < 2 or parts[0] != "GET":
                await self._respond(writer, 405, "text/plain", b"method not allowed")
                return
            path = parts[1].split("?")[0]
            if path.startswith("/favicon"):
                await self._respond(writer, 204, "text/plain", b"")
                return
            rel = self.ALLOWED.get(path)
            if rel is None:
                await self._respond(writer, 404, "text/plain", b"not found")
                return
            data = (self.root / rel).read_bytes()
            ext = rel.rsplit(".", 1)[-1]
            await self._respond(writer, 200, self.TYPES.get(ext, "application/octet-stream"), data)
        except Exception:
            # More specific error handling to avoid swallowing critical errors
            try:
                await self._respond(writer, 500, "text/plain", b"internal error")
            except Exception:
                pass
        finally:
            try:
                writer.close()
            except Exception:
                pass

    async def _respond(self, writer, code: int, ctype: str, body: bytes) -> None:
        reason = {200: "OK", 204: "No Content", 404: "Not Found",
                  405: "Method Not Allowed", 500: "Internal Server Error"}[code]
        header = (f"HTTP/1.1 {code} {reason}\r\n"
                  f"Content-Type: {ctype}\r\n"
                  f"Content-Length: {len(body)}\r\n"
                  f"Connection: close\r\n"
                  f"Cache-Control: no-store\r\n\r\n").encode("ascii")
        writer.write(header + body)
        await writer.drain()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def detect_port() -> str:
    """Pick the most likely Deneyap port: native USB CDC (ttyACM) first."""
    candidates = []
    if serial is None:
        return "/dev/ttyUSB0"  # fallback
    for c in serial.tools.list_ports.comports():
        name = c.device
        # Prefer ttyACM (CDC ACM) which is typical for USB-to-serial adapters
        if name.startswith("/dev/ttyACM"):
            return name
        # Fall back to ttyUSB if no ttyACM found
        candidates.append(name)
    # No ttyACM; return the first ttyUSB if any, else a generic fallback
    return candidates[0] if candidates else "/dev/ttyUSB0"


def parse_args(argv):
    p = argparse.ArgumentParser(description="Deneyap Hand Tracker — Phase 1 bridge")
    p.add_argument("--sim", action="store_true",
                   help="run on a synthetic IMU stream (no hardware)")
    p.add_argument("--port", help="serial port (default: auto-detect)")
    p.add_argument("--baud", type=int, default=int(os.environ.get("DENEYAP_BAUD", "921600")))
    p.add_argument("--ws-host", default=os.environ.get("DENEYAP_WS_HOST", "127.0.0.1"))
    p.add_argument("--ws-port", type=int, default=int(os.environ.get("DENEYAP_WS_PORT", "8765")))
    p.add_argument("--http-host", default=os.environ.get("DENEYAP_HTTP_HOST", "127.0.0.1"))
    p.add_argument("--http-port", type=int, default=int(os.environ.get("DENEYAP_HTTP_PORT", "8080")))
    p.add_argument("--no-auto-zero", action="store_true",
                   help="do not auto-capture the neutral pose on the first frame")
    p.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    if args.list_ports:
        if serial is None:
            print("pyserial is not installed", file=sys.stderr)
            return 1
        for c in serial.tools.list_ports.comports():
            print(f"{c.device}\t{c.description}")
        return 0

    if WS == "none":
        print("websockets is not installed — run: pip install -r requirements.txt",
              file=sys.stderr)
        return 1

    if args.sim:
        tracker = pose.PoseTracker(auto_zero=not args.no_auto_zero)
        pipeline = Pipeline(tracker, "sim")
        source = SimSource(pipeline)
        mode = "sim"
    else:
        port = args.port or os.environ.get("DENEYAP_SERIAL_PORT") or detect_port()
        tracker = pose.PoseTracker(auto_zero=not args.no_auto_zero)
        pipeline = Pipeline(tracker, "hw")
        source = SerialSource(pipeline, port, args.baud)
        mode = f"hardware @ {port}"

    hub = WsHub(pipeline, args.ws_host, args.ws_port)
    http = HttpServer(WEB_DIR, args.http_host, args.http_port)

    log.info("Deneyap Hand Tracker bridge started (mode: %s)", mode)
    log.info("open the UI at  http://%s:%d", args.http_host, args.http_port)

    async def run_all():
        await asyncio.gather(source.run(), hub.run(), http.run())

    try:
        asyncio.run(run_all())
    except KeyboardInterrupt:
        log.info("shutting down")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())