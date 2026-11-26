"""Gamepad -> car controller over Bluetooth serial (HC-05), no window.

Usage:  python fin.py [PORT]
  PORT is a serial port (Windows default: COM8 @ 9600) or the HC-05's Bluetooth
  address (Linux default: 98:D3:21:F8:79:52), opened directly over RFCOMM, so no
  /dev/rfcomm0 or `rfcomm bind` is needed. The module must be paired first.

Controls (Xbox pad), car-style:
  RT             throttle forward (analog)
  LT             throttle reverse (analog, ignored while RT is pressed)
  left stick X   steering
  A              horn (buzzer on the car while held)
  B              emergency stop (all motors 0)
The gamepad rumbles while the car reports an obstacle closer than 20 cm ("CLOSE").

Packet sent to the car (6 bytes):  'y' <throttle>  'x' <steering>  'h' <A button>
Both -127..127, encoded as one byte by encode_axis(). Throttle: forward +, reverse -.
Steering: stick right = +. While driving it sets how much the inner wheel slows down
(127 = it stops): outer wheel = throttle, inner wheel = throttle * (127 - |X|) / 127.
With no throttle the car spins on the spot: wheels = +-(6 * X) / 10 (max 76 at X = 127).
Dead zones, sensitivity and minimum speed are all done here, so tuning never needs a re-flash.
"""

import os
import re
import socket
import sys
import threading
import time

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame
import serial

# Stop packet: both motors 0, horn off
ZERO_PACKET = b"\x79\x00\x78\x00\x68\x00"

# Throttle (Y) from the triggers. Just past the trigger dead zone the value jumps to
# THROTTLE_MIN_OUT (the lowest value that still turns the motors), then rises linearly to
# 127 * THROTTLE_SCALE with the trigger fully pressed.
THROTTLE_SCALE = 0.8     # full trigger = 0.8 * 127 = 102. 1.0 = full speed
THROTTLE_MIN_OUT = 20    # raise if the wheels hum but don't turn at a light press, lower if they jerk
TRIGGER_DEADZONE = 0.05  # trigger travel ignored at the start (stops a resting trigger from driving)
LT_AXIS, RT_AXIS = 4, 5  # joystick axis numbers of the triggers (depends on driver / OS)


def shape_throttle(value: float) -> int:
    """Throttle -1..1 -> value sent, 0 or +-THROTTLE_MIN_OUT..+-(127 * THROTTLE_SCALE)."""
    if value == 0.0:
        return 0
    max_out = 127 * THROTTLE_SCALE
    min_out = min(THROTTLE_MIN_OUT, max_out)
    out = min_out + (max_out - min_out) * abs(value)
    return int(round(out)) if value > 0 else -int(round(out))


# Steering (X) from the left stick, linear. Sent as 127 * TURN_SCALE at full lock; on the car it
# sets how much the inner wheel slows: 1.0 = it stops, 0.70 = it keeps 30 % of the throttle.
TURN_SCALE = 0.70      # how sharply it turns at full lock, 1.0 = sharpest. Lower = rides straighter
STEER_DEADZONE = 0.15  # stick X within this of centre = no turning (keeps "forward" straight)

# Spin on the spot (no trigger pressed). The car runs the wheels at +-(6 * X) / 10, so X = 127 gives
# 76 (60 % power), the most the firmware allows. Just past the dead zone X jumps to PIVOT_MIN_OUT,
# then rises linearly to 127 * PIVOT_SCALE at full stick.
PIVOT_SCALE = 1.0      # full stick = 127 -> wheels at 76
PIVOT_MIN_OUT = 80     # X sent at the first touch -> wheels at 48. Raise if it hums but won't spin


def shape_pivot(value: float) -> int:
    """Dead-zoned stick X -1..1 -> X sent while standing still, 0 or +-PIVOT_MIN_OUT..+-(127 * PIVOT_SCALE)."""
    if value == 0.0:
        return 0
    max_out = 127 * PIVOT_SCALE
    min_out = min(PIVOT_MIN_OUT, max_out)
    out = min_out + (max_out - min_out) * abs(value)
    return int(round(out)) if value > 0 else -int(round(out))


DEFAULT_PORT = "COM8" if os.name == "nt" else "98:D3:21:F8:79:52"
BT_ADDR_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


class RfcommSerial:
    """Minimal pyserial-like wrapper around a Linux Bluetooth RFCOMM socket (channel 1 = HC-05 SPP)."""

    def __init__(self, addr: str, channel: int = 1, timeout: float = 0.5) -> None:
        self._sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self._sock.settimeout(10)
        self._sock.connect((addr, channel))
        self._sock.settimeout(timeout)
        self.is_open = True

    in_waiting = 0  # unknown for a socket; read() returns whatever has arrived

    def read(self, size: int = 1) -> bytes:
        try:
            data = self._sock.recv(max(size, 256))
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("Bluetooth link closed")
        return data

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        self._sock.setblocking(False)
        try:
            while self._sock.recv(1024):
                pass
        except (BlockingIOError, OSError):
            pass
        finally:
            self._sock.settimeout(0.5)

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False
        self._sock.close()


def open_link(port: str):
    if BT_ADDR_RE.match(port):
        return RfcommSerial(port)
    return serial.Serial(port, 9600, timeout=0.1, write_timeout=0.5)


def encode_axis(value: float) -> int:
    """Speed -127..127 -> one byte: 0..127 stays, negative -v becomes 127 + v (128..254)."""
    c = max(-127, min(127, int(round(value))))
    return c if c >= 0 else 127 + abs(c)


def apply_deadzone(value: float, deadzone: float = 0.12) -> float:
    """Stick axis -1..1: ignore small movements near the centre, rescale the rest to -1..1."""
    if abs(value) <= deadzone:
        return 0.0
    scaled = min(1.0, (abs(value) - deadzone) / (1.0 - deadzone))
    return scaled if value > 0.0 else -scaled


def apply_trigger_deadzone(raw_value: float, deadzone: float = TRIGGER_DEADZONE) -> float:
    """Trigger axis (-1 released .. 1 pressed) -> 0..1 with a small dead zone."""
    norm = max(0.0, min(1.0, (raw_value + 1.0) * 0.5))
    return 0.0 if norm <= deadzone else min(1.0, (norm - deadzone) / (1.0 - deadzone))


def stop_rumble(joy) -> None:
    if joy is None:
        return
    try:
        joy.stop_rumble() if hasattr(joy, "stop_rumble") else joy.rumble(0.0, 0.0, 0)
    except Exception:
        pass


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    pygame.init()
    try:
        pygame.display.set_mode((1, 1), pygame.HIDDEN)
    except Exception:
        pass
    pygame.joystick.init()

    joy, jid, btn_a, last_raw_a, lt_seen, rt_seen = None, None, 0, 0, False, False

    def attach(idx: int) -> None:
        nonlocal joy, jid, lt_seen, rt_seen
        joy = pygame.joystick.Joystick(idx)
        joy.init()
        jid, lt_seen, rt_seen = joy.get_instance_id(), False, False
        print(f"Pad connected: {joy.get_name()}", flush=True)

    count = pygame.joystick.get_count()
    if count > 0:
        attach(next((i for i in range(count) if "xbox" in pygame.joystick.Joystick(i).get_name().lower()), 0))
    else:
        print("Pad not connected", flush=True)

    # Car link. Windows: COM port of the paired HC-05; Linux: its Bluetooth address (RFCOMM socket)
    try:
        ser = open_link(port)
        time.sleep(1.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(ZERO_PACKET)
        ser.flush()
        print(f"Car connected ({port})", flush=True)
    except Exception as exc:
        print(f"Car not connected ({port}): {exc}", flush=True)
        pygame.quit()
        return

    running, prox_close = True, False
    io_lock, tx_cond, tx_slot = threading.Lock(), threading.Condition(), [None]

    # Background thread: read status lines from the car ("CLOSE" / "CLEAR" = obstacle < 20 cm or not)
    def reader_loop() -> None:
        nonlocal running, prox_close
        buf = ""
        try:
            while running and ser.is_open:
                raw = ser.read(ser.in_waiting or 1)
                if not raw:
                    continue
                buf = (buf + raw.decode("utf-8", errors="ignore"))[-1024:]
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    cmd = line.strip().upper()
                    if cmd == "CLOSE":
                        prox_close = True
                    elif cmd == "CLEAR":
                        prox_close = False
        except Exception:
            if running:
                print(f"Car disconnected ({port})", flush=True)
            running = False
            with tx_cond:
                tx_cond.notify_all()

    # Background thread: send the newest packet (only the latest one waits in tx_slot)
    def writer_loop() -> None:
        nonlocal running
        while running and ser.is_open:
            with tx_cond:
                while tx_slot[0] is None and running:
                    tx_cond.wait(timeout=0.2)
                pkt, tx_slot[0] = tx_slot[0], None
            if pkt is None or not running:
                continue
            try:
                with io_lock:
                    if ser.is_open:
                        ser.write(pkt)
            except Exception as exc:
                if running:
                    print(f"Car disconnected ({port}): {exc}", flush=True)
                running = False

    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=writer_loop, daemon=True).start()

    last_axes, cur_a, last_sent = (0, 0), 0, (0, 0, 0)
    last_tx, pending, prev_prox, last_rumble = 0.0, False, False, 0.0
    clock = pygame.time.Clock()

    # Queue a packet for sending; skipped if it is identical to the last one (unless force=True)
    def transmit(y: int, x: int, a: int, force: bool = False) -> None:
        nonlocal last_sent, last_tx, pending
        pkt = bytes([0x79, encode_axis(y), 0x78, encode_axis(x), 0x68, 1 if a else 0])
        state = (pkt[1], pkt[3], pkt[5])
        if not force and state == last_sent:
            return
        last_sent, last_tx, pending = state, time.monotonic(), False
        with tx_cond:
            tx_slot[0] = pkt
            tx_cond.notify()

    try:
        while running:
            stop_all = False
            for ev in pygame.event.get():
                if ev.type == pygame.JOYDEVICEADDED and joy is None:
                    attach(ev.device_index)
                elif ev.type == pygame.JOYDEVICEREMOVED and ev.instance_id == jid:
                    print("Pad disconnected", flush=True)
                    stop_rumble(joy)
                    joy, jid, btn_a, last_raw_a, lt_seen, rt_seen = None, None, 0, 0, False, False
                    if last_axes != (0, 0) or cur_a != 0:
                        last_axes, cur_a, stop_all = (0, 0), 0, True
                elif ev.type == pygame.JOYAXISMOTION and ev.instance_id == jid:
                    lt_seen = lt_seen or ev.axis == LT_AXIS
                    rt_seen = rt_seen or ev.axis == RT_AXIS
                elif ev.type == pygame.JOYBUTTONDOWN and ev.instance_id == jid:
                    if ev.button == 1:
                        stop_all = True
                    elif ev.button == 0:
                        btn_a = 1
                elif ev.type == pygame.JOYBUTTONUP and ev.instance_id == jid and ev.button == 0:
                    btn_a = 0

            if stop_all:
                btn_a, cur_a, last_axes = 0, 0, (0, 0)
                transmit(0, 0, 0, force=True)
            elif joy is not None:
                if joy.get_numbuttons() >= 1:
                    raw_a = 1 if joy.get_button(0) else 0
                    if raw_a != last_raw_a:
                        last_raw_a, btn_a = raw_a, raw_a
                a_changed = btn_a != cur_a
                cur_a = btn_a

                # Triggers: -1 released .. 1 pressed. Until a trigger has moved once, SDL may report 0
                # (= half pressed), so it counts as released until then (lt_seen / rt_seen).
                n = joy.get_numaxes()
                raw_lt = joy.get_axis(LT_AXIS) if n > LT_AXIS else -1.0
                raw_rt = joy.get_axis(RT_AXIS) if n > RT_AXIS else -1.0
                lt_seen = lt_seen or abs(raw_lt) > 1e-4
                rt_seen = rt_seen or abs(raw_rt) > 1e-4
                lt = apply_trigger_deadzone(raw_lt) if lt_seen else 0.0
                rt = apply_trigger_deadzone(raw_rt) if rt_seen else 0.0
                throttle = rt if rt > 0.0 else -lt   # RT wins; LT only counts while RT is released

                # Steering: left stick X (axis 0), right = +. While driving it is softened by
                # TURN_SCALE; with no throttle it is boosted for spinning on the spot.
                raw_x = joy.get_axis(0) if n >= 1 else 0.0
                stick_x = apply_deadzone(raw_x, STEER_DEADZONE)
                y_out = shape_throttle(throttle)
                x_out = int(round(stick_x * TURN_SCALE * 127)) if y_out != 0 else shape_pivot(stick_x)

                # The STM32 turns (throttle, steering) into left/right wheel speeds
                new_axes = (y_out, x_out)
                axes_changed = new_axes != last_axes
                last_axes = new_axes

                # Send at most every 62.5 ms (16/s) while moving; A button and stopping go out immediately
                if a_changed or (axes_changed and new_axes == (0, 0)):
                    transmit(last_axes[0], last_axes[1], cur_a)
                elif axes_changed:
                    if time.monotonic() - last_tx >= 0.0625:
                        transmit(last_axes[0], last_axes[1], cur_a)
                    else:
                        pending = True

            if pending and time.monotonic() - last_tx >= 0.0625:
                transmit(last_axes[0], last_axes[1], cur_a)

            # Rumble the gamepad while the car reports an obstacle
            now = time.monotonic()
            if prox_close and joy is not None and (not prev_prox or now - last_rumble >= 0.35):
                last_rumble = now
                joy.rumble(0.85, 0.85, 250)
            elif not prox_close and prev_prox:
                stop_rumble(joy)
            prev_prox = prox_close
            clock.tick(60)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        with tx_cond:
            tx_cond.notify_all()
        stop_rumble(joy)
        try:
            with io_lock:
                if ser.is_open:
                    ser.write(ZERO_PACKET)
                    ser.flush()
                    ser.close()
        except Exception:
            pass
        pygame.quit()


if __name__ == "__main__":
    main()
