#!/usr/bin/env python3
"""Small development-run recorder (docs/handoff/AI_WORKFLOW.md).

Runs ONE command from the project root and stores under
logs/development/<KST YYYYMMDD_HHMMSS>_<name>/ :
  run.json    argv, cwd, start/end KST, real child exit code, status, source hashes
  stdout.log  child stdout          stderr.log  child stderr
  result.md   skeleton with exit code; the operator appends the judgment.

It records only the given command's output. It does not collect environment
variables or the AI conversation. Exit code = child's exit code.

Operator, stage and hardware use are DECLARED by the caller, not observed:
  --operator  who ran it (default "unspecified")
  --stage     work step, e.g. C1.1 or A1 (default "unspecified")
  --hardware  none | used | unknown (default "unknown"); "used" should be
              accompanied by --note describing device/power/stop means.
Argv and output are stored verbatim. This tool is NOT a credential scrubber:
never pass secrets or credential file contents on the command line or let the
command print them. run.json and result.md carry the same declared values.

usage: dev_run_log.py --name NAME [--operator WHO] [--stage STEP]
                      [--hardware none|used|unknown] [--source PATH ...]
                      [--note TEXT] [--input-kind synthetic|measured|recorded|none]
                      [--log-root DIR] -- CMD [ARGS...]
"""
import argparse, datetime, hashlib, json, os, platform, subprocess, sys, tempfile, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KST = datetime.timezone(datetime.timedelta(hours=9), "KST")


def kst_now():
    return datetime.datetime.now(KST)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def write_json_atomic(path, data):
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=".run.", suffix=".json")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True)
    ap.add_argument("--source", action="append", default=[], help="project-relative file to hash")
    ap.add_argument("--note", default="")
    ap.add_argument("--input-kind", default="synthetic", choices=["synthetic", "measured", "recorded", "none"],
                    help="synthetic = invented; measured = live device data; recorded = replay of stored/external recordings; none")
    ap.add_argument("--operator", default="unspecified", help="declared, not observed")
    ap.add_argument("--stage", default="unspecified", help="declared work step, e.g. C1.1 / A1")
    ap.add_argument("--hardware", default="unknown", choices=["none", "used", "unknown"],
                    help="declared hardware access; not verified by this tool")
    ap.add_argument("--log-root", default=None, help="override logs/development (tests)")
    ap.add_argument("cmd", nargs=argparse.REMAINDER)
    args = ap.parse_args(argv)
    cmd = args.cmd[1:] if args.cmd[:1] == ["--"] else args.cmd
    if not cmd:
        ap.error("command required after --")
    start = kst_now()
    log_root = Path(args.log_root) if args.log_root else ROOT / "logs" / "development"
    run_dir = log_root / f"{start.strftime('%Y%m%d_%H%M%S')}_{args.name}"
    run_dir.mkdir(parents=True, exist_ok=False)
    sources = {}
    for rel in args.source:
        p = ROOT / rel
        sources[rel] = sha256(p) if p.is_file() else None
    record = {
        "name": args.name, "operator_declared": args.operator, "stage_declared": args.stage,
        "status": "running",
        "start_kst": start.isoformat(timespec="seconds"), "end_kst": None,
        "cwd": str(ROOT), "argv": cmd, "exit_code": None,
        "stdout": "stdout.log", "stderr": "stderr.log",
        "source_sha256": sources, "git_repository": (ROOT / ".git").exists(),
        "environment": {"python": platform.python_version(), "platform": platform.platform()},
        "input_kind": args.input_kind, "hardware_access_declared": args.hardware,
        "declared_not_observed": ["operator_declared", "stage_declared", "hardware_access_declared", "input_kind"],
        "credential_scrubbing": False, "note": args.note,
    }
    write_json_atomic(run_dir / "run.json", record)
    t0 = time.monotonic()
    with open(run_dir / "stdout.log", "wb") as out, open(run_dir / "stderr.log", "wb") as err:
        try:
            proc = subprocess.run(cmd, cwd=str(ROOT), stdout=out, stderr=err)
            code = proc.returncode
            status = "completed" if code == 0 else "failed"
        except FileNotFoundError as exc:
            err.write(str(exc).encode()); code = 127; status = "failed"
        except KeyboardInterrupt:
            code = 130; status = "interrupted"
    record.update(status=status, exit_code=code, end_kst=kst_now().isoformat(timespec="seconds"),
                  duration_s=round(time.monotonic() - t0, 3))
    write_json_atomic(run_dir / "run.json", record)
    hw_text = {"none": "없음(선언)", "used": "사용(선언)", "unknown": "미확인(선언 없음)"}[args.hardware]
    (run_dir / "result.md").write_text(
        f"# {args.name}\n\n- 상태: {status}, 종료코드 {code}\n- 시작/끝 KST: {record['start_kst']} → {record['end_kst']}\n"
        f"- 실행자(선언): {args.operator}, 단계(선언): {args.stage}\n"
        f"- 명령: `{' '.join(cmd)}`\n- 입력(선언): {args.input_kind}, 하드웨어 접근(선언): {hw_text}\n"
        f"- 이 값들은 실행자가 선언한 것이며 도구가 관측한 결과가 아니다.\n\n## 판단\n\n(작성 전)\n",
        encoding="utf-8")
    shown = run_dir.relative_to(ROOT) if run_dir.is_relative_to(ROOT) else run_dir
    print(f"[dev_run_log] {shown} status={status} exit={code}")
    return code


if __name__ == "__main__":
    sys.exit(main())
