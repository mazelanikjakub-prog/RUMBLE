"""Gamepad -> car controller over Bluetooth serial (COM8 @ 9600 baud), no window.

Controls (Xbox pad):
  left stick     Y = forward/back, X = left/right (sent raw, the car does the mixing)
  A              horn (buzzer on the car while held)
  B              emergency stop (all motors 0)
The gamepad rumbles while the car reports an obstacle closer than 20 cm ("CLOSE").

Packet sent to the car (6 bytes):  'y' <stick Y>  'x' <stick X>  'h' <A button>
Stick values are -127..127 (Y: forward +, X: right +), encoded as one byte by encode_axis().
"""

import os
import threading
import time

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame
import serial

# Stop packet: both motors 0, horn off
ZERO_PACKET = b"\x79\x00\x78\x00\x68\x00"


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


def apply_trigger_deadzone(raw_value: float, deadzone: float = 0.05) -> float:
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

    # Bluetooth serial port of the car. Windows: COM port of the paired module; Linux: e.g. /dev/rfcomm0
    try:
        ser = serial.Serial("COM8", 9600, timeout=0.1, write_timeout=0.5)
        time.sleep(1.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(ZERO_PACKET)
        ser.flush()
        print("Car connected (COM8 @ 9600)", flush=True)
    except Exception as exc:
        print(f"Car not connected (COM8): {exc}", flush=True)
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
                print("Car disconnected (COM8)", flush=True)
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
                    print(f"Car disconnected (COM8): {exc}", flush=True)
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
                    lt_seen = lt_seen or ev.axis == 4
                    rt_seen = rt_seen or ev.axis == 5
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

                # Left stick, raw (only a dead zone): X = axis 0, Y = axis 1.
                # pygame gives Y = -1 when the stick is pushed forward, so flip it: forward = +127.
                # The STM32 turns (Y, X) into left/right motor speeds.
                n = joy.get_numaxes()
                raw_x = joy.get_axis(0) if n >= 1 else 0.0
                raw_y = joy.get_axis(1) if n >= 2 else 0.0
                stick_x = apply_deadzone(raw_x)    # right = +
                stick_y = -apply_deadzone(raw_y)   # forward = +
                new_axes = (
                    max(-127, min(127, int(round(stick_y * 127)))),
                    max(-127, min(127, int(round(stick_x * 127)))),
                )
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
