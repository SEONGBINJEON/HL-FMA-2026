"""Parse the observed three-sensor test output; no device access."""
import math
import re

VALUE = r'(NO ECHO|[0-9]+(?:\.[0-9]+)? cm)'
PATTERN = re.compile(r'LEFT\s*:\s*'+VALUE+r'\s*\|\s*CENTER\s*:\s*'+VALUE+r'\s*\|\s*RIGHT\s*:\s*'+VALUE)


def parse_test_line(line):
    match = PATTERN.fullmatch(line.strip())
    if not match:
        return None
    values = [None if s == 'NO ECHO' else float(s[:-3]) for s in match.groups()]
    if any(v is not None and not math.isfinite(v) for v in values):
        return None
    return {'format':'three_sensor_test', 'ultrasonic_cm':values,
            'no_echo':[v is None for v in values]}
