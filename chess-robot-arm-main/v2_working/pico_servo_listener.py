# pico_servo_listener.py
# Flash this onto the Raspberry Pi Pico.
# It listens on USB serial for commands from the calibration tool
# and drives the servos accordingly.
#
# Command format (sent from PC):  S<pin>:<pulse_us>\n
# Example:  S0:1500\n  → sets GP0 servo to 1500µs (90°)
#
# Servos:
#   GP0 = Base      (MG995)
#   GP1 = Shoulder  (MG995)
#   GP2 = Elbow     (MG995)
#   GP3 = Wrist Pitch (SG90)
#   GP4 = Wrist Roll  (SG90)
#   GP5 = Gripper     (SG90)

import sys
import select
from machine import Pin, PWM

# ── PWM setup ──────────────────────────────────────────────────────────────
SERVO_PINS = [0, 1, 2, 3, 4, 5]
pwms = []
for pin_num in SERVO_PINS:
    p = PWM(Pin(pin_num))
    p.freq(50)          # 50 Hz — standard for all hobby servos
    pwms.append(p)

# ── Safe pulse limits (µs) ─────────────────────────────────────────────────
# Prevents mechanical damage if bad values are sent.
# MG995: rated 500–2500µs but 600–2400 is safer
# SG90:  rated 500–2400µs
PULSE_SAFE = [
    (600, 2400),   # GP0 Base      MG995
    (600, 2400),   # GP1 Shoulder  MG995
    (600, 2400),   # GP2 Elbow     MG995
    (500, 2400),   # GP3 Wrist P   SG90
    (500, 2400),   # GP4 Wrist R   SG90
    (500, 1900),   # GP5 Gripper   SG90 (limited travel)
]

def set_servo_pulse(pin_idx, pulse_us):
    lo, hi = PULSE_SAFE[pin_idx]
    pulse_us = max(lo, min(hi, pulse_us))
    # duty_ns expects nanoseconds; PWM period at 50Hz = 20,000,000 ns
    pwms[pin_idx].duty_ns(pulse_us * 1000)

def angle_to_pulse(angle):
    return int(500 + (angle / 180.0) * 2000)

# ── Boot: move all servos to 90° (safe center) ────────────────────────────
print("Chess Arm Pico listener starting...")
for i in range(len(SERVO_PINS)):
    set_servo_pulse(i, angle_to_pulse(90))
print("All servos centered at 90°")
print("Ready. Waiting for commands (format: S<pin>:<pulse_us>)")

# ── Main loop ──────────────────────────────────────────────────────────────
buf = ""
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

while True:
    # Non-blocking read — poll with 10ms timeout
    events = poll.poll(10)
    if not events:
        continue

    ch = sys.stdin.read(1)
    if ch == '\n':
        line = buf.strip()
        buf = ""

        if not line:
            continue

        # Parse "S<pin>:<pulse>"
        if line.startswith('S') and ':' in line:
            try:
                parts = line[1:].split(':')
                pin_idx = int(parts[0])
                pulse   = int(parts[1])

                if 0 <= pin_idx < len(SERVO_PINS):
                    set_servo_pulse(pin_idx, pulse)
                    print(f"OK GP{SERVO_PINS[pin_idx]} → {pulse}µs")
                else:
                    print(f"ERR unknown pin index {pin_idx}")

            except (ValueError, IndexError) as e:
                print(f"ERR parse error: {e}")

        # Convenience: "R" = go to rest (all 90°)
        elif line == 'R':
            for i in range(len(SERVO_PINS)):
                set_servo_pulse(i, angle_to_pulse(90))
            print("OK all servos → 90°")

        # "P" = print current pulse widths (debug)
        elif line == 'P':
            for i, p in enumerate(pwms):
                print(f"  GP{SERVO_PINS[i]}: {p.duty_ns() // 1000}µs")

        else:
            print(f"ERR unknown command: {line}")

    else:
        buf += ch
