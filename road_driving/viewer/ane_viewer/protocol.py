"""The line protocol between the viewer and `road_drive live` (see ../PROTOCOL.md). Pure Python.

Backend to viewer: one JSON object per line on stdout, each with "v" (the protocol version) and "type";
simulation messages also carry "tick" (the 10 ms world tick) and "t" (simulated seconds). Diagnostics go
to stderr and are never parsed. Viewer to backend: one JSON object per line on stdin, with "cmd".
"""
import json

VERSION = 1
MESSAGE_TYPES = {
    "hello", "session", "scene", "tick", "gnss", "scan", "event", "route", "path", "closure", "status", "applied",
    "end", "saved", "error", "bye",
}


class ProtocolError(ValueError):
    pass


def decode(line):
    """One backend line to a message dict; raises ProtocolError for anything that is not a v1 message."""
    line = line.strip()
    if not line:
        raise ProtocolError("empty line")
    try:
        msg = json.loads(line)
    except json.JSONDecodeError as e:
        raise ProtocolError("not JSON: %s" % e) from None
    if not isinstance(msg, dict):
        raise ProtocolError("not an object")
    if msg.get("v") != VERSION:
        raise ProtocolError("protocol version %r, expected %d" % (msg.get("v"), VERSION))
    if msg.get("type") not in MESSAGE_TYPES:
        raise ProtocolError("unknown message type %r" % msg.get("type"))
    return msg


def encode(cmd):
    return json.dumps(cmd, separators=(",", ":")) + "\n"


# ---- Commands -----------------------------------------------------------------------------------------
def reset(config):
    return {"cmd": "reset", "config": dict(config)}


def replay(path):
    return {"cmd": "replay", "session": str(path)}


def pause():
    return {"cmd": "pause"}


def resume():
    return {"cmd": "resume"}


def step(ticks=1):
    return {"cmd": "step", "ticks": int(ticks)}


def speed(value):
    return {"cmd": "speed", "value": float(value)}


def gnss_outage(on):
    return {"cmd": "gnss_outage", "on": bool(on)}


def gnss_bias(on, metres=5.0, bearing_deg=90.0):
    """A constant offset on every fix, `metres` at `bearing_deg` from the car's heading when applied."""
    return {"cmd": "gnss_bias", "on": bool(on), "metres": float(metres), "bearing_deg": float(bearing_deg)}


def kerb_blackout(on):
    return {"cmd": "kerb_blackout", "on": bool(on)}


def closure_ahead(distance=40.0):
    return {"cmd": "closure", "action": "ahead", "distance": float(distance)}


def closure_lift():
    return {"cmd": "closure", "action": "lift"}


def save_session(path):
    return {"cmd": "save_session", "path": str(path)}


def quit_():
    return {"cmd": "quit"}
