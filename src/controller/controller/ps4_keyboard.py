#!/usr/bin/env python3
"""
Keyboard -> virtual PS4 (DualShock 4) gamepad for Ubuntu.

Run:   sudo python3 ps4_keyboard.py
Stop:  Ctrl+C   (or:  sudo pkill -f ps4_keyboard.py)

TAP the RIGHT CTRL key at any time to switch gamepad mode ON/OFF.
While ON, your keyboard only controls the gamepad (it doesn't type).
While OFF, your keyboard types normally again.

CONTROLS
  W A S D        Left stick
  Arrow keys     Right stick
  I J K L        D-pad (up, left, down, right)
  Z              Cross (X)
  X              Circle
  C              Square
  V              Triangle
  Q / E          L1 / R1
  R / F          L2 / R2  (triggers)
  T / G          L3 / R3  (stick clicks)
  Tab            Share
  Enter          Options
  P              PS button
  Right Ctrl     Toggle gamepad mode on/off (just tap it alone)
"""

import select
import signal
import sys
import time

from evdev import AbsInfo, InputDevice, UInput, ecodes as e, list_devices

TOGGLE_KEY = e.KEY_RIGHTCTRL

# keyboard key -> gamepad button
BUTTONS = {
    e.KEY_Z: e.BTN_SOUTH,      # Cross
    e.KEY_X: e.BTN_EAST,       # Circle
    e.KEY_V: e.BTN_NORTH,      # Triangle
    e.KEY_C: e.BTN_WEST,       # Square
    e.KEY_Q: e.BTN_TL,         # L1
    e.KEY_E: e.BTN_TR,         # R1
    e.KEY_R: e.BTN_TL2,        # L2 (button)
    e.KEY_F: e.BTN_TR2,        # R2 (button)
    e.KEY_TAB: e.BTN_SELECT,   # Share
    e.KEY_ENTER: e.BTN_START,  # Options
    e.KEY_P: e.BTN_MODE,       # PS button
    e.KEY_T: e.BTN_THUMBL,     # L3
    e.KEY_G: e.BTN_THUMBR,     # R3
}

LEFT_STICK = {"up": e.KEY_W, "down": e.KEY_S, "left": e.KEY_A, "right": e.KEY_D}
RIGHT_STICK = {"up": e.KEY_UP, "down": e.KEY_DOWN, "left": e.KEY_LEFT, "right": e.KEY_RIGHT}
DPAD = {"up": e.KEY_I, "down": e.KEY_K, "left": e.KEY_J, "right": e.KEY_L}

CENTER = 128


def make_gamepad():
    stick = AbsInfo(value=CENTER, min=0, max=255, fuzz=0, flat=0, resolution=0)
    trigger = AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)
    hat = AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)
    capabilities = {
        e.EV_KEY: list(BUTTONS.values()),
        e.EV_ABS: [
            (e.ABS_X, stick), (e.ABS_Y, stick),      # left stick
            (e.ABS_RX, stick), (e.ABS_RY, stick),    # right stick
            (e.ABS_Z, trigger), (e.ABS_RZ, trigger),  # L2 / R2 analog
            (e.ABS_HAT0X, hat), (e.ABS_HAT0Y, hat),  # D-pad
        ],
    }
    # 054c:09cc = Sony DualShock 4
    return UInput(
        capabilities,
        name="Wireless Controller",
        vendor=0x054C,
        product=0x09CC,
        version=0x8111,
        bustype=e.BUS_USB,
    )


def find_keyboards():
    keyboards = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except Exception:
            continue
        keys = dev.capabilities().get(e.EV_KEY, [])
        # a real keyboard has letters and space, and is not our own virtual pad
        if e.KEY_A in keys and e.KEY_SPACE in keys and dev.name != "Wireless Controller":
            keyboards.append(dev)
    return keyboards


def wait_for_release(keyboards, timeout=3.0):
    """Wait until no key is held, so the desktop never gets a 'stuck' key."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if not any(kb.active_keys() for kb in keyboards):
                return
        except OSError:
            return
        time.sleep(0.02)


def handle_term(signum, frame):
    raise KeyboardInterrupt  # lets us clean up properly on 'pkill'


def axis_value(pressed, neg_key, pos_key):
    value = 0
    if neg_key in pressed:
        value -= 1
    if pos_key in pressed:
        value += 1
    return value


def main():
    keyboards = find_keyboards()
    if not keyboards:
        print("No keyboard found. Did you run it with sudo?")
        sys.exit(1)

    pad = make_gamepad()
    pressed = set()
    enabled = False

    def send_state():
        """Send the full gamepad state based on currently pressed keys."""
        for key, button in BUTTONS.items():
            pad.write(e.EV_KEY, button, 1 if key in pressed else 0)

        lx = axis_value(pressed, LEFT_STICK["left"], LEFT_STICK["right"])
        ly = axis_value(pressed, LEFT_STICK["up"], LEFT_STICK["down"])
        rx = axis_value(pressed, RIGHT_STICK["left"], RIGHT_STICK["right"])
        ry = axis_value(pressed, RIGHT_STICK["up"], RIGHT_STICK["down"])
        pad.write(e.EV_ABS, e.ABS_X, min(255, CENTER + lx * 127))
        pad.write(e.EV_ABS, e.ABS_Y, min(255, CENTER + ly * 127))
        pad.write(e.EV_ABS, e.ABS_RX, min(255, CENTER + rx * 127))
        pad.write(e.EV_ABS, e.ABS_RY, min(255, CENTER + ry * 127))

        pad.write(e.EV_ABS, e.ABS_Z, 255 if e.KEY_R in pressed else 0)
        pad.write(e.EV_ABS, e.ABS_RZ, 255 if e.KEY_F in pressed else 0)

        pad.write(e.EV_ABS, e.ABS_HAT0X, axis_value(pressed, DPAD["left"], DPAD["right"]))
        pad.write(e.EV_ABS, e.ABS_HAT0Y, axis_value(pressed, DPAD["up"], DPAD["down"]))
        pad.syn()

    def set_enabled(on):
        nonlocal enabled
        enabled = on
        pressed.clear()
        if on:
            wait_for_release(keyboards)
        for kb in keyboards:
            try:
                kb.grab() if on else kb.ungrab()
            except OSError:
                pass  # already grabbed / already released
        send_state()
        print("Gamepad mode: ON  (keyboard = gamepad)" if on
              else "Gamepad mode: OFF (keyboard types normally)")

    print("Virtual PS4 gamepad created. Keyboards used:",
          ", ".join(k.name for k in keyboards))
    print("Tap RIGHT CTRL to toggle gamepad mode. Ctrl+C to quit.")
    send_state()
    set_enabled(True)

    signal.signal(signal.SIGTERM, handle_term)
    devices = {kb.fd: kb for kb in keyboards}
    toggle_tapped = False
    try:
        while True:
            ready, _, _ = select.select(list(devices), [], [])
            for fd in ready:
                try:
                    events = list(devices[fd].read())
                except OSError:
                    del devices[fd]  # keyboard unplugged
                    continue
                for ev in events:
                    if ev.type != e.EV_KEY or ev.value == 2:  # ignore auto-repeat
                        continue
                    if ev.code == TOGGLE_KEY:
                        if ev.value == 1:
                            toggle_tapped = True
                        elif ev.value == 0 and toggle_tapped:
                            toggle_tapped = False
                            set_enabled(not enabled)  # act on RELEASE
                        continue
                    toggle_tapped = False  # another key used: not a pure tap
                    if not enabled:
                        continue
                    if ev.value == 1:
                        pressed.add(ev.code)
                    else:
                        pressed.discard(ev.code)
                    send_state()
            if not devices:
                print("All keyboards disconnected.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        for kb in keyboards:
            try:
                kb.ungrab()
            except Exception:
                pass
        pad.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
