#!/usr/bin/env python3
"""A pretend FCB camera on a pseudo-terminal, for testing the driver without hardware.

    fake_visca_camera.py            # prints the pty path, e.g. /dev/pts/7
    ros2 launch fcb_camera fcb_camera.launch.py visca_port:=/dev/pts/7

Implements ACK/completion semantics, the inquiries the driver polls, zoom and
focus movement with realistic travel time, and error replies for unknown
commands. Only the byte-level protocol is simulated; there is no video.
"""
import os
import pty
import select
import sys
import threading
import time
import tty

ZOOM_OPTICAL_END = 0x4000
ZOOM_DIGITAL_END = 0x7AC0


class Camera:
    def __init__(self):
        self.zoom = 0
        self.zoom_target = 0
        self.zoom_speed = 0  # +/- units per tick when in tele/wide mode
        self.focus = 0x8000
        self.focus_target = 0x8000
        self.focus_speed = 0
        self.focus_auto = True
        self.dzoom = False
        self.icr = 0x03
        self.auto_icr = 0x03
        self.ae = 0x00
        self.shutter, self.iris, self.gain = 0x0E, 0x11, 0x00
        self.wb = 0x00
        self.stabilizer = 0x03
        self.power = 0x02
        self.presets = {}
        self.lock = threading.Lock()

    def tick(self):
        with self.lock:
            limit = ZOOM_DIGITAL_END if self.dzoom else ZOOM_OPTICAL_END
            if self.zoom_speed:
                self.zoom = max(0, min(limit, self.zoom + self.zoom_speed))
                self.zoom_target = self.zoom
            elif self.zoom != self.zoom_target:
                step = 600
                self.zoom += max(-step, min(step, self.zoom_target - self.zoom))
            if self.focus_speed:
                self.focus = max(0x1000, min(0xF000, self.focus + self.focus_speed))
                self.focus_target = self.focus
            elif self.focus != self.focus_target:
                step = 2000
                self.focus += max(-step, min(step, self.focus_target - self.focus))

    def moving(self):
        return self.zoom != self.zoom_target or self.focus != self.focus_target


def nib(v, n):
    return bytes((v >> (4 * (n - 1 - i))) & 0x0F for i in range(n))


def val(body, off, n):
    v = 0
    for b in body[off:off + n]:
        v = (v << 4) | (b & 0x0F)
    return v


def handle(cam, body, send, log):
    """body = bytes between header and 0xFF. Returns after replies are queued."""
    ack = bytes([0x90, 0x41, 0xFF])
    done = bytes([0x90, 0x51, 0xFF])

    def completion_after(pred, timeout=10.0):
        def wait():
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout and not pred():
                time.sleep(0.02)
            send(done)
        threading.Thread(target=wait, daemon=True).start()

    if body == b"\x01\x00\x01":  # IF_Clear: completion only, no ACK
        send(done)
        return
    if body[:2] == b"\x09\x00" and body[2:3] == b"\x02":  # version
        send(bytes([0x90, 0x50, 0x00, 0x20, 0x0A, 0x50, 0x01, 0x23, 0x02, 0xFF]))
        return
    if body[0] == 0x09 and body[1] == 0x04:  # inquiries
        c = body[2]
        with cam.lock:
            table = {
                0x47: nib(cam.zoom, 4), 0x48: nib(cam.focus, 4),
                0x38: bytes([0x02 if cam.focus_auto else 0x03]),
                0x06: bytes([0x02 if cam.dzoom else 0x03]),
                0x01: bytes([cam.icr]), 0x51: bytes([cam.auto_icr]),
                0x39: bytes([cam.ae]), 0x4A: nib(cam.shutter, 4), 0x4B: nib(cam.iris, 4),
                0x4C: nib(cam.gain, 4), 0x35: bytes([cam.wb]), 0x34: bytes([cam.stabilizer]),
                0x00: bytes([cam.power]),
            }
        if c in table:
            send(bytes([0x90, 0x50]) + table[c] + b"\xFF")
        else:
            send(bytes([0x90, 0x60, 0x02, 0xFF]))  # syntax error
        return
    if body[0] == 0x01 and body[1] == 0x04:
        c = body[2]
        send(ack)
        with cam.lock:
            if c == 0x07:  # zoom
                sub = body[3]
                if sub == 0x00:
                    cam.zoom_speed = 0
                    cam.zoom_target = cam.zoom
                elif sub & 0xF0 == 0x20:
                    cam.zoom_speed = 150 * ((sub & 0x07) + 1)
                elif sub & 0xF0 == 0x30:
                    cam.zoom_speed = -150 * ((sub & 0x07) + 1)
                send(done)
            elif c == 0x47:
                limit = ZOOM_DIGITAL_END if cam.dzoom else ZOOM_OPTICAL_END
                cam.zoom_speed = 0
                cam.zoom_target = min(limit, val(body, 3, 4))
                completion_after(lambda: not cam.moving())
            elif c == 0x06:
                cam.dzoom = body[3] == 0x02
                send(done)
            elif c == 0x08:
                sub = body[3]
                if sub == 0x00:
                    cam.focus_speed = 0
                    cam.focus_target = cam.focus
                elif sub & 0xF0 == 0x20:
                    cam.focus_speed = -800 * ((sub & 0x07) + 1)
                elif sub & 0xF0 == 0x30:
                    cam.focus_speed = 800 * ((sub & 0x07) + 1)
                send(done)
            elif c == 0x38:
                cam.focus_auto = body[3] == 0x02
                send(done)
            elif c == 0x18:
                cam.focus_target = 0x6000
                completion_after(lambda: not cam.moving())
            elif c == 0x48:
                cam.focus_target = max(0x1000, min(0xF000, val(body, 3, 4)))
                completion_after(lambda: not cam.moving())
            elif c == 0x01:
                cam.icr = body[3]
                threading.Timer(0.3, lambda: send(done)).start()
            elif c == 0x51:
                cam.auto_icr = body[3]
                send(done)
            elif c == 0x39:
                cam.ae = body[3]
                send(done)
            elif c == 0x4A:
                cam.shutter = val(body, 5, 2); send(done)
            elif c == 0x4B:
                cam.iris = val(body, 5, 2); send(done)
            elif c == 0x4C:
                cam.gain = val(body, 5, 2); send(done)
            elif c == 0x35:
                cam.wb = body[3]; send(done)
            elif c == 0x34:
                cam.stabilizer = body[3]; send(done)
            elif c == 0x00:
                cam.power = body[3]; send(done)
            elif c == 0x3F:
                action, slot = body[3], body[4]
                if action == 0x01:
                    cam.presets[slot] = (cam.zoom, cam.focus)
                    send(done)
                elif action == 0x02 and slot in cam.presets:
                    cam.zoom_target, cam.focus_target = cam.presets[slot]
                    completion_after(lambda: not cam.moving())
                elif action == 0x02:
                    send(bytes([0x90, 0x60, 0x41, 0xFF]))
                else:
                    cam.presets.pop(slot, None)
                    send(done)
            elif c in (0x21, 0x2C, 0x4D, 0x3E, 0x4E, 0x33, 0x5A, 0x10, 0x43, 0x44, 0x37, 0x53, 0x42,
                       0x66, 0x61, 0x62):
                send(done)  # accepted, no simulated effect
            else:
                send(bytes([0x90, 0x60, 0x02, 0xFF]))
        return
    send(bytes([0x90, 0x60, 0x02, 0xFF]))


def main():
    master, slave = pty.openpty()
    tty.setraw(master)
    path = os.ttyname(slave)
    print(path, flush=True)
    cam = Camera()
    lock = threading.Lock()

    def send(data):
        with lock:
            os.write(master, data)

    def log(msg):
        print(msg, file=sys.stderr, flush=True)

    def ticker():
        while True:
            cam.tick()
            time.sleep(0.02)

    threading.Thread(target=ticker, daemon=True).start()
    buf = bytearray()
    while True:
        r, _, _ = select.select([master], [], [], 1.0)
        if not r:
            continue
        try:
            data = os.read(master, 64)
        except OSError:
            time.sleep(0.1)
            continue
        for b in data:
            buf.append(b)
            if b == 0xFF:
                frame = bytes(buf)
                buf.clear()
                if len(frame) >= 3 and (frame[0] & 0xF8) == 0x80:
                    log(f"<- {frame.hex(' ')}")
                    handle(cam, frame[1:-1], send, log)


if __name__ == "__main__":
    main()
