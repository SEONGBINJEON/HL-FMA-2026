#!/usr/bin/env python3
"""Static pin-map check for firmware/joystick_manual (2026-09-18 D50 change).

Verifies, from the source text and the official Mega variant table, that
  * STEER_DIR is 50 and no output/diagnostic still points at D6 (PH3),
  * frame[43]/frame[44] bit0 read PORTB/PINB bit3, which the Arduino Mega variant maps to D50,
  * D50 is not reused by any other pin constant (ultrasonic TRIG/ECHO, encoders, PWM, BRK),
  * D50 has no timer (so no PWM side effect) and SPI is not included by the sketch,
  * frame layout / bit meaning for the host parser (bit0 DIR, bit1 BRK) is unchanged.
It reads files only; no device access. Exit 0 = all checks pass, 1 = a check failed.
"""
import re, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "firmware/joystick_manual"
VARIANT = ROOT / ".setup/arduino-cli/data/packages/arduino/hardware/avr/1.8.8/variants/mega/pins_arduino.h"

def table(text, name):
    body = re.search(name + r"\[\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
    rows = [l.split(",")[0].strip() for l in body.splitlines() if l.strip() and not l.strip().startswith("//")]
    return rows

def main():
    fails = []
    ino = (SRC / "joystick_manual.ino").read_text()
    var = VARIANT.read_text()
    ports = table(var, "digital_pin_to_port_PGM"); masks = table(var, "digital_pin_to_bit_mask_PGM"); timers = table(var, "digital_pin_to_timer_PGM")
    pins = {k: int(v) for k, v in re.findall(r"constexpr uint8_t (\w+) = (\d+);", ino)}
    trigs = [int(x) for x in re.search(r"TRIGS\[3\] = \{([^}]*)\}", ino).group(1).split(",")]
    echos = [int(x) for x in re.search(r"ECHOS\[3\] = \{([^}]*)\}", ino).group(1).split(",")]
    used = {**{k: v for k, v in pins.items() if k not in ("RX_BUDGET", "JOY_STATUS_SIZE")}, **{f"TRIG{i}": p for i, p in enumerate(trigs)}, **{f"ECHO{i}": p for i, p in enumerate(echos)}}
    used.pop("STEER_ADC", None)  # A15 = 69, not a digital-side conflict with 50
    if pins.get("STEER_DIR") != 50: fails.append(f"STEER_DIR is {pins.get('STEER_DIR')}, expected 50")
    dup = [k for k, v in used.items() if v == 50 and k != "STEER_DIR"]
    if dup: fails.append(f"D50 also used by {dup}")
    if ports[50] != "PB" or masks[50] != "_BV( 3 )": fails.append(f"variant maps D50 to {ports[50]} {masks[50]}, expected PB _BV(3)")
    if timers[50] != "NOT_ON_TIMER": fails.append(f"D50 has timer {timers[50]}")
    if ports[6] != "PH" or masks[6] != "_BV( 3 )": fails.append("variant table for D6 unexpected")
    if re.search(r"(DDRH|PORTH|PINH)\s*&\s*_BV\(3\)", ino): fails.append("diagnostics still read PH3 (D6)")
    for reg, idx in (("PORTB", 43), ("PINB", 44)):
        if not re.search(rf"frame\[{idx}\] = \(\({reg} & _BV\(3\)\) \? 1 : 0\)", ino): fails.append(f"frame[{idx}] bit0 does not read {reg} bit3")
    if not re.search(r"frame\[42\] = \(\(DDRB & _BV\(5\)\) \? 1 : 0\) \|\s*\(\(DDRB & _BV\(3\)\) \? 2 : 0\) \| \(\(DDRH & _BV\(4\)\) \? 4 : 0\)", ino): fails.append("frame[42] mode bits not (PB5, PB3, PH4)")
    for reg, idx in (("PORTH", 43), ("PINH", 44)):
        if not re.search(rf"frame\[{idx}\] = .*\(\({reg} & _BV\(4\)\) \? 2 : 0\)", ino): fails.append(f"frame[{idx}] bit1 (STEER_BRK D7) changed")
    if pins.get("STEER_BRK") != 7 or pins.get("STEER_PWM") != 11: fails.append("STEER_BRK/STEER_PWM changed")
    if re.search(r"#include\s*<SPI\.h>", ino): fails.append("SPI.h included; D50 is MISO")
    if "digitalRead(STEER_DIR)" in ino or "digitalRead(STEER_PWM)" in ino: fails.append("digitalRead on steering output pin")
    if "JOY_STATUS_SIZE = 47" not in ino: fails.append("status size changed")
    for name in ("control_core.h", "control_core.cpp", "step_gate.h"):
        if re.search(r"\b(digitalWrite|pinMode|PORT[A-L]|DDR[A-L])\b", (SRC / name).read_text()): fails.append(f"{name} touches pins")
    print("pins:", used)
    print("D50 ->", ports[50], masks[50], timers[50], "| D6 ->", ports[6], masks[6])
    for f in fails: print("FAIL:", f)
    print("RESULT:", "PASS" if not fails else f"{len(fails)} FAIL")
    return 1 if fails else 0

if __name__ == "__main__":
    sys.exit(main())
