#!/usr/bin/env python3
"""Bounded real-Tk smoke check of step panel; demo only, hardware inaccessible.

Observes entry to Tk.mainloop with the Python profile API, then schedules real
Tk button/key/focus events. It does not replace application functions, the
session state machine, or DemoTransport. This checks synthetic UI event paths,
not human ergonomics or physical actuation. Hard deadline: eight seconds.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import tkinter as tk
import uuid

import step_manual_control as app
from step_manual_protocol import FrameParser, Command, Status, CONTROL, DISARM


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="New, nonexistent evidence folder")
    args = parser.parse_args(argv)
    args.connect = False  # No command-line path can enable physical device access.
    output = args.output or app.ROOT / "data" / (
        datetime.now().astimezone().strftime("step_ui_smoke_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    if output.exists():
        parser.error("output directory must not already exist")
    started = time.monotonic()
    report = {"started_utc": datetime.now(timezone.utc).isoformat(),
              "mode": ("BENCH MegaTransport" if args.connect else "unchanged DemoTransport") + " / actual Tk mainloop / synthetic UI events",
              "hardware_access": args.connect, "checks": [], "layout": [],
              "human_operation_verified": False, "board_integration_verified": False,
              "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    failures = []
    root_seen = []

    def save_report():
        output.mkdir(parents=True, exist_ok=True)
        (output / "ui_smoke_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        (output / "executed_ui_check.py").write_bytes(Path(__file__).read_bytes())

    def check(name, passed, detail=None):
        row = {"name": name, "passed": bool(passed), "elapsed_s": time.monotonic() - started}
        if detail is not None:
            row["detail"] = detail
        report["checks"].append(row)
        if not passed:
            failures.append(name)

    def deadline(signum, frame):
        report.update(passed=False, failure="Eight-second hard deadline", elapsed_s=time.monotonic() - started)
        save_report()
        print(json.dumps({"passed": False, "reason": "deadline", "output": str(output)}), flush=True)
        os._exit(124)

    old_handler = signal.signal(signal.SIGALRM, deadline)
    signal.setitimer(signal.ITIMER_REAL, 8.)

    def attach(root):
        root_seen.append(root)
        report["display"] = {"screen_pixels": [root.winfo_screenwidth(), root.winfo_screenheight()],
                             "tk_points_to_pixels": float(root.tk.call("tk", "scaling"))}

        def descendants(widget):
            for child in widget.winfo_children():
                yield child
                yield from descendants(child)

        def labels():
            result = []
            for widget in descendants(root):
                if widget.winfo_class() != "Label":
                    continue
                variable = widget.cget("textvariable")
                result.append(str(root.getvar(variable)) if variable else str(widget.cget("text")))
            return result

        def phase_is(value):
            return any("상태: " + app.PHASE_LABELS[value] in label for label in labels())

        def button(text):
            return next(widget for widget in descendants(root)
                        if widget.winfo_class() == "Button" and widget.cget("text") == text)

        def event(key, down=True, timestamp=1):
            root.event_generate("<KeyPress>" if down else "<KeyRelease>",
                                keysym=key, time=timestamp, when="tail")

        def inspect_layout_and_arm():
            root_width, root_height = root.winfo_width(), root.winfo_height()
            for widget in descendants(root):
                if widget.winfo_class() not in ("Label", "Button"):
                    continue
                x = widget.winfo_rootx() - root.winfo_rootx()
                y = widget.winfo_rooty() - root.winfo_rooty()
                width, height = widget.winfo_width(), widget.winfo_height()
                req_width, req_height = widget.winfo_reqwidth(), widget.winfo_reqheight()
                label = str(widget.cget("text"))
                visible = bool(widget.winfo_ismapped())
                within = x >= 0 and y >= 0 and x + width <= root_width and y + height <= root_height
                uncompressed = width >= req_width and height >= req_height
                report["layout"].append({"widget": str(widget), "class": widget.winfo_class(),
                    "text": label, "rect": [x, y, width, height], "requested": [req_width, req_height],
                    "mapped": visible, "within_root": within, "uncompressed": uncompressed})
            check("all labels/buttons visible within root", all(
                item["mapped"] and item["within_root"] and item["uncompressed"] for item in report["layout"]),
                {"root_size": [root_width, root_height]})
            check("held drive and protected steer labels visible", any("수동 조작" in text for text in labels())
                  and ("보드 연결 · 벤치 시험" if args.connect else "화면 연습 · 장치 연결 없음") in labels()
                  and any("누르는 동안 계속" in text and "조향은 한 번 누를 때 최대 1초" in text for text in labels()))
            button("조작 준비").invoke()

        def ready_and_press():
            check("ready after real Tk arm button", phase_is("ready"), labels())
            check("input method disabled for this Tk application", not bool(int(root.tk.call("tk", "useinputmethods"))))
            event("Up", True, 1010)

        def active_and_add_steer():
            check("active after Tk Up-only press", phase_is("active"), labels())
            check("recognized arrow visible without Shift", any("인식된 키:" in text and "↑ 전진" in text and "Shift" not in text for text in labels()))
            check("transport reports requested forward", any("요청 구동/조향: +1 / +0" in text for text in labels()))
            event("Left", True, 1150)

        def simultaneous_and_release_steer():
            check("forward and left are simultaneous", any("요청 구동/조향: +1 / -1" in text for text in labels()))
            event("Left", False, 1250)

        def straight_and_release():
            check("releasing steer keeps forward active", any("요청 구동/조향: +1 / +0" in text for text in labels()))
            event("Up", False, 1200)

        def released():
            check("key release keeps prepared UI at zero", phase_is("ready"), labels())

        def second_press():
            check("same prepared session accepts a later key", phase_is("ready"), labels())
            event("w", True, 2010)

        def lose_focus():
            check("second active input uses W alone", phase_is("active"), labels())
            root.event_generate("<FocusOut>", when="tail")

        def regain_focus():
            check("FocusOut locks UI", phase_is("locked"), labels())
            root.event_generate("<FocusIn>", when="tail")
            event("w", False, 2200)

        def explicit_stop():
            check("FocusIn does not rearm", phase_is("locked"), labels())
            button("정지  ·  Esc").invoke()
            event("Escape", True, 2400)

        def close_window():
            check("stop and Escape remain locked", phase_is("locked"), labels())
            command = root.protocol("WM_DELETE_WINDOW")
            check("real close handler installed", bool(command))
            root.tk.call(command)

        actions = [(100, root.focus_force), (350, inspect_layout_and_arm),
                   (700, ready_and_press), (950, active_and_add_steer),
                   (1100, simultaneous_and_release_steer), (1250, straight_and_release), (1400, released),
                   (1700, second_press),
                   (1950, lose_focus), (2150, regain_focus), (2350, explicit_stop),
                   (2550, close_window)]
        for delay_ms, action in actions:
            def guarded(callback=action):
                try:
                    callback()
                except Exception as exc:
                    check("scheduled callback: " + callback.__name__, False,
                          f"{type(exc).__name__}: {exc}")
            root.after(delay_ms + (1500 if args.connect else 0), guarded)

    def profile(frame, event, arg):
        if event == "call" and frame.f_code is tk.Misc.mainloop.__code__:
            sys.setprofile(None)
            attach(frame.f_locals["self"])

    try:
        sys.setprofile(profile)
        app.run_ui(connect=args.connect, output=output)
        sys.setprofile(None)
        check("Tk mainloop observed and returned", len(root_seen) == 1)
        events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
        check("journal identifies selected transport", json.loads((output / "metadata.json").read_text())["mode"] == ("connect" if args.connect else "demo"))
        check("no client failure", not any(row["event"] == "connection_failed" for row in events))
        check("focus loss event recorded", any(row["event"] == "focus" and not row["focused"] for row in events))
        check("one explicit successful arm click", sum(row["event"] == "arm_click" and row["accepted"] for row in events) == 1)
        commands, errors = FrameParser().feed((output / "tx.bin").read_bytes(), 0.)
        check("recorded TX consists of valid commands", not errors and commands and all(isinstance(c, Command) for c in commands))
        check("UI input reached serialized CONTROL", any(c.action == CONTROL and c.drive == 1 and c.deadman == 1 for c in commands))
        check("close serialized final DISARM", bool(commands) and commands[-1].action == DISARM)
    except Exception as exc:
        check("UI smoke execution", False, f"{type(exc).__name__}: {exc}")
    finally:
        sys.setprofile(None)
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_handler)
    report.update(passed=not failures, failures=failures, elapsed_s=time.monotonic() - started)
    report['board_integration_verified'] = False
    save_report()
    print(json.dumps({"passed": report["passed"], "checks": len(report["checks"]),
                      "failures": failures, "elapsed_s": report["elapsed_s"], "output": str(output)},
                     ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
