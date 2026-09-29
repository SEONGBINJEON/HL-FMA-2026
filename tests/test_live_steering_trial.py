"""live_steering_trial: the SAME assembly (assemble()) as the CLI, driven with a fake clock over
steer_target_sim's MockTransport + BoardModel, plus a real PTY for the serial transport.
Everything is synthetic; no vehicle device is opened. The operational config must stay null."""
import io, json, os, select, subprocess, sys, tempfile, termios, time, unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import live_steering_trial as lst
import steer_target_host as h
import steer_target_sim as sim
import step_manual_control as smc
import step_manual_protocol as smp
from step_manual_protocol import DISARM, ARM, CONTROL, DISARMED, ARMED

SYNTH = ROOT / 'tests/fixtures/steering_calibration_synthetic.json'
ASCII_LINES = (b"=====\r\n F8 PID HILL HOLD READY\r\n=====\r\n",
               b"[MONITOR] ENC=0 TARGET=0 ERROR=0 MODE=HOLD | POT_RAW=519 POT=519.00 STEER_TARGET=519 STEER_CENTER=519\r\n")


class FailingTransport(sim.MockTransport):
    """MockTransport whose read raises OSError once `dead` is set (cable pulled)."""
    dead = False

    def read(self, size):
        if self.dead: raise OSError(5, 'Input/output error')
        return super().read(size)


class Harness:
    """5 ms fake-clock engine around one LiveTrial (host) and one BoardModel (board)."""

    def __init__(self, lib, tmp, angle_cdeg=400, board='type150', boot=57, adc0=600, **kw):
        self.lib, self.board_mode = lib, board
        self.fake = sim.FakeTime(); self.real_time = smc.time; smc.time = self.fake
        self.transport = FailingTransport()
        self.journal = lst.TrialJournal(Path(tmp) / 'out', 'model_synthetic', {'test': True})
        self.trial = lst.assemble(lib, SYNTH, angle_cdeg, self.transport, self.journal, allow_synthetic=True, **kw)
        self.sign = self.trial.core.config.steer_sign_for_adc_increase
        self.board = sim.BoardModel(lib, boot, adc0, self.sign)
        self.ms = 0; self.alive = True; self.adc_hist = []; self.frames_on = True
        self.frozen = False        # True: the wheel does not move even though output is commanded
        self.noise = 0             # +/- ADC counts added around the frozen value (sensor noise)
        self._noise_phase = 0

    def close(self):
        smc.time = self.real_time
        self.board.destroy()
        if self.journal.events and not self.journal.events.closed: self.journal.close()

    def board_ms(self): return (self.ms + 123456) & 0xFFFFFFFF

    def freeze(self, noise=0):
        """Stop the simulated rack from moving (jammed linkage / slipped coupling), optionally noisy."""
        self._frozen_at = self.board.adc; self.frozen = True; self.noise = noise

    def restart_board(self, boot):
        adc = self.board.adc; self.board.destroy(); self.board = sim.BoardModel(self.lib, boot, adc, self.sign)

    def run(self, seconds, keys=None, until_finished=True):
        """keys: {ms: 'p'|'l'|'r'|'0'|'s'|'q'}. Returns True while the trial is alive."""
        keys = keys or {}
        end = self.ms + int(seconds * 1000)
        while self.ms < end:
            now = self.ms / 1000; self.fake.now = now
            if self.ms in keys: self.trial.key(keys[self.ms], now)
            self.board.tick(self.board_ms())
            frame = self.board.status(self.board_ms())
            if self.frames_on:
                if self.board_mode == 'type150' and frame: self.transport.rx += frame
                elif self.board_mode == 'ascii' and self.ms % 200 == 0: self.transport.rx += ASCII_LINES[self.ms % 400 // 200]
                elif self.board_mode == 'type144' and self.ms % 50 == 0:
                    self.transport.rx += smp.encode_status(smp.Status(41, self.ms // 50 + 1, self.ms, 0, 500))
            self.alive = self.trial.tick(now)
            for raw in self.transport.pending_tx: self.board.feed(raw, self.board_ms())
            self.transport.pending_tx.clear()
            if self.frozen:
                # the rack does not move: keep the measured value put, optionally with small sensor noise
                if self.noise:
                    self._noise_phase = (self._noise_phase + 1) % 4
                    base = getattr(self, '_frozen_at', self.board.adc)
                    self.board.adc = base + (self.noise if self._noise_phase < 2 else -self.noise)
                elif not hasattr(self, '_frozen_at'):
                    self._frozen_at = self.board.adc
            else:
                self.board.plant(0.005)
            self.adc_hist.append((self.ms, self.board.adc))
            self.ms += 5
            if not self.alive and until_finished: break
        return self.alive

    def tx(self): return [sim.decode_tx(r) for r in self.transport.tx_frames]

    def controls(self): return [d for d in self.tx() if d.get('action') == CONTROL]

    def steers(self): return [d['steer'] for d in self.controls()]

    def core_reasons(self): return {t.core_reason for t in self.trial.adapter.trace}

    def invariants(self, tc):
        tx = self.tx()
        tc.assertTrue(all(not d.get('malformed') for d in tx))
        tc.assertTrue(all(d.get('drive', 0) == 0 and d.get('pwm', 0) == 0 for d in tx), 'drive/PWM must be 0')
        tc.assertEqual(self.board.max_actual_drive, 0)
        tc.assertNotIn(7, self.board.reasons_seen, 'board reported REVERSAL')
        prev = 0
        for s in self.steers():
            tc.assertFalse(s and prev and s == -prev, 'direct sign flip in TX'); prev = s


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not h.LIB_PATH.is_file():
            subprocess.run(['bash', str(ROOT / 'firmware/steer_target/host/build.sh')], check=True, capture_output=True)
        cls.lib = h.load_library()

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix='lst_'); self.tmp = self._tmp.name; self.hz = None

    def tearDown(self):
        if self.hz: self.hz.close()
        self._tmp.cleanup()

    def harness(self, **kw):
        self.hz = Harness(self.lib, self.tmp, **kw); return self.hz


class PreflightAndUnits(Base):
    def test_operational_null_is_blocked_and_live_refused_before_open(self):
        out = io.StringIO()
        with redirect_stdout(out): rc = lst.main([])
        self.assertEqual(rc, lst.EXIT_BLOCKED); self.assertIn('PREFLIGHT: BLOCKED', out.getvalue()); self.assertIn('calibration:', out.getvalue())
        master, slave = os.openpty()
        try:
            a = termios.tcgetattr(slave); a[3] |= termios.ECHO; termios.tcsetattr(slave, termios.TCSANOW, a)
            new = Path(self.tmp) / 'never'
            err = io.StringIO()
            with redirect_stderr(err):
                rc = lst.main(['--live', '--port', os.ttyname(slave), '--output-dir', str(new)])
            self.assertEqual(rc, lst.EXIT_BLOCKED); self.assertIn('REFUSED before opening', err.getvalue())
            self.assertFalse(new.exists())
            self.assertTrue(termios.tcgetattr(slave)[3] & termios.ECHO, 'port was touched')   # never opened
            # synthetic record is refused on the live CLI too (no flag can allow it)
            err = io.StringIO()
            with redirect_stderr(err):
                rc = lst.main(['--live', '--calibration', str(SYNTH), '--port', os.ttyname(slave), '--output-dir', str(new)])
            self.assertEqual(rc, lst.EXIT_BLOCKED); self.assertIn('synthetic', err.getvalue()); self.assertFalse(new.exists())
        finally:
            os.close(slave); os.close(master)

    def test_preflight_report_and_angle_range(self):
        rep = Path(self.tmp) / 'pre.json'
        with redirect_stdout(io.StringIO()): rc = lst.main(['--preflight', '--out', str(rep)])
        r = json.loads(rep.read_text())
        self.assertEqual((rc, r['status'], r['live_capable'], r['hardware_access']), (2, 'BLOCKED', False, False))
        # 운용 기록은 '측정 완료' 상태일 때만 통과한다. 2026-09-18 현재 그 파일은
        # blocked_angle_unverified로 차단돼 있으므로 status 사유로 거부되는 것이 정상이다.
        self.assertTrue(any("not 'measured'" in m or 'unmeasured' in m for m in r['missing']), r['missing'])
        self.assertEqual(lst.preflight(SYNTH, 5.0, allow_synthetic=True)['status'], 'READY')
        self.assertIn('synthetic record refused for operation', ' '.join(lst.preflight(SYNTH, 5.0)['missing']))
        self.assertTrue(any(m.startswith('angle:') for m in lst.preflight(SYNTH, 40.0, allow_synthetic=True)['missing']))
        self.assertTrue(any(m.startswith('angle:') for m in lst.preflight(SYNTH, float('nan'), allow_synthetic=True)['missing']))
        self.assertTrue(any('not present' in m for m in lst.preflight(SYNTH, 5.0, port='/dev/does-not-exist', allow_synthetic=True)['missing']))

    def test_degree_to_centidegree_and_table_check(self):
        self.assertEqual((lst.deg_to_cdeg(5.0), lst.deg_to_cdeg(-4.5), lst.deg_to_cdeg(0)), (500, -450, 0))
        for bad in (float('nan'), float('inf'), True, 400.0, '5'):
            with self.assertRaises(ValueError): lst.deg_to_cdeg(bad)
        v = lst.preflight(SYNTH, 5.0, allow_synthetic=True)['validated']
        self.assertEqual(lst.angle_in_table(v, 1500), '')
        self.assertIn('outside usable', lst.angle_in_table(v, 1600))     # adc 830 > usable 820
        self.assertIn('outside the measured table', lst.angle_in_table(v, 2000))

    def test_cli_argument_rejection(self):
        for argv in (['--angle-deg', 'nan'], ['--max-hold-s', '0'], ['--model', '--live']):
            with self.assertRaises(SystemExit) as cm, redirect_stderr(io.StringIO()), redirect_stdout(io.StringIO()):
                lst.main(argv)
            self.assertEqual(cm.exception.code, 2, argv)
        with redirect_stderr(io.StringIO()):       # live without --output-dir on the null record: blocked before anything
            self.assertEqual(lst.main(['--live', '--port', '/dev/null']), lst.EXIT_BLOCKED)
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(lst.main(['--model', '--angle-deg', '40', '--output-dir', str(Path(self.tmp) / 'x')]), lst.EXIT_BLOCKED)
        self.assertFalse((Path(self.tmp) / 'x').exists())

    def test_help_and_import_open_nothing(self):
        r = subprocess.run([sys.executable, 'tools/live_steering_trial.py', '--help'], cwd=str(ROOT), capture_output=True, text=True)
        self.assertEqual(r.returncode, 0); self.assertIn('STEERING ONLY', r.stdout)
        with self.assertRaises(ValueError): lst.zero_drive_encoder(smp.Command(1, 1, CONTROL, 1, 0, 1))
        self.assertEqual(lst.zero_drive_encoder(smp.Command(1, 1, CONTROL, 0, -1, 1))[16], 0)


class PtyTransport(Base):
    def test_raw_no_echo_hupcl_off_kept_after_close(self):
        master, slave = os.openpty()
        try:
            a = termios.tcgetattr(slave); a[3] |= termios.ECHO | termios.ICANON; a[2] |= termios.HUPCL
            termios.tcsetattr(slave, termios.TCSANOW, a)
            t = lst.PosixSerialTransport(os.ttyname(slave), check_identity=False)
            f = termios.tcgetattr(slave)
            self.assertFalse(f[3] & termios.ECHO); self.assertFalse(f[3] & termios.ICANON); self.assertFalse(f[2] & termios.HUPCL)
            os.write(master, b'\xf1\xa5junk'); time.sleep(0.02)
            self.assertEqual(t.in_waiting, 6); self.assertEqual(t.read(64), b'\xf1\xa5junk'); self.assertEqual(t.out_waiting, 0)
            self.assertEqual(select.select([master], [], [], 0.1)[0], [], 'echoed while open')
            t.write(b'x'); self.assertEqual(os.read(master, 8), b'x')     # host->board path works
            t.close(); self.assertIsNone(t.fd)
            f = termios.tcgetattr(slave)
            self.assertFalse(f[3] & termios.ECHO); self.assertFalse(f[2] & termios.HUPCL)
            os.write(master, b'[MONITOR] ENC=0\r\n')
            self.assertEqual(select.select([master], [], [], 0.2)[0], [], 'kernel echo re-enabled by close')
            with self.assertRaises(RuntimeError): lst.PosixSerialTransport(os.ttyname(slave), check_identity=True)  # not the Mega
        finally:
            os.close(slave); os.close(master)


class Assembly(Base):
    def test_assemble_refuses_null_and_synthetic_without_flag(self):
        with self.assertRaises(ValueError): lst.assemble(self.lib, lst.DEFAULT_CAL, 400, sim.MockTransport(), None)
        with self.assertRaises(ValueError): lst.assemble(self.lib, SYNTH, 400, sim.MockTransport(), None)
        hz = self.harness()
        self.assertIsInstance(hz.trial.client.parser, __import__('joystick_protocol').Parser)   # type150, not type144 default
        self.assertIs(hz.trial.client.encoder, lst.zero_drive_encoder)
        self.assertLessEqual(hz.trial.refresh_s, hz.trial.validated['command_stale_ms'] / 2000)

    def test_ascii_hill_hold_board_gets_zero_bytes_and_exit4(self):
        hz = self.harness(board='ascii', identify_timeout_s=5.0)
        hz.run(2.0, keys={300: 'p', 600: 'l', 900: 's', 1200: 'q'})
        self.assertTrue(hz.trial.finished); self.assertEqual(hz.trial.exit_code, lst.EXIT_UNIDENTIFIED)
        self.assertIn('ASCII firmware', hz.trial.exit_reason); self.assertEqual(hz.transport.tx_frames, [])
        self.assertEqual(hz.journal.tx_frames, 0); self.assertFalse(hz.trial.adapter.identified)

    def test_type144_or_silent_board_times_out_with_zero_bytes(self):
        for mode in ('type144', 'none'):
            with self.subTest(mode):
                hz = Harness(self.lib, tempfile.mkdtemp(dir=self.tmp), board=mode, identify_timeout_s=1.0)
                try:
                    hz.run(2.0, keys={300: 'p', 500: 'l'})
                    self.assertEqual(hz.trial.exit_code, lst.EXIT_UNIDENTIFIED); self.assertEqual(hz.transport.tx_frames, [])
                finally:
                    hz.close()

    def test_prepare_target_finite_hold_stop_ack_reprepare_reverse_quit(self):
        hz = self.harness(max_hold_s=1.5)
        hz.run(0.3); self.assertTrue(hz.trial.adapter.identified)
        self.assertEqual(hz.trial.adapter.phase, 'locked')
        self.assertTrue(all(d['action'] == DISARM for d in hz.tx()), 'only DISARM before explicit prepare')
        hz.run(0.5, keys={300: 'l', 400: 'p'})                     # 'l' before prepare is refused (no lock/no tx change)
        self.assertTrue(any('refused: not prepared' in t for _, t in hz.trial.events))
        self.assertTrue(hz.trial.prepared(), hz.trial.adapter.message)
        hz.run(2.5, keys={1000: 'l'})                               # +4.00 deg -> adc 660, hold 1.5 s then explicit stop
        self.assertIn('AT_TARGET', hz.core_reasons()); self.assertIn(-1, hz.steers())
        self.assertLessEqual(abs(hz.board.adc - 660), hz.trial.core.config.stop_band_adc + 3)
        self.assertEqual(hz.trial.last_stop_reason, 'hold window expired'); self.assertTrue(hz.trial.stop_ack_observed)
        self.assertTrue(hz.trial.adapter.locked_by_fault); self.assertIsNone(hz.trial.target_cdeg)
        seq_after_hold = hz.trial.seq
        hz.run(0.3, keys={hz.ms + 50: 'r'})                          # refused while locked
        self.assertEqual(hz.trial.seq, seq_after_hold); self.assertEqual(hz.trial.adapter.phase, 'locked')
        hz.run(0.6, keys={hz.ms + 50: 'p'}); self.assertTrue(hz.trial.prepared())
        hz.run(3.0, keys={hz.ms + 50: 'r'})                          # -4.00 deg -> adc 542 (decreasing side of the table)
        self.assertIn(1, hz.steers()); self.assertLess(hz.board.adc, 600)
        hz.run(1.0, keys={hz.ms + 50: 'q'})
        self.assertTrue(hz.trial.finished); self.assertEqual(hz.trial.exit_code, lst.EXIT_OK); self.assertTrue(hz.trial.stop_ack_observed)
        hz.invariants(self)
        # sequences strictly increase on one core; the window was refreshed with new sequences, not the same one
        self.assertEqual(hz.trial.core.last_sequence(), hz.trial.seq); self.assertGreater(hz.trial.seq, 6)
        hz.journal.close(exit_code=hz.trial.exit_code)
        meta = json.loads((hz.journal.path / 'metadata.json').read_text())
        self.assertEqual(meta['data_kind'], 'model_synthetic'); self.assertGreater(meta['tx_frames'], 0)
        events = [json.loads(l) for l in (hz.journal.path / 'events.jsonl').read_text().splitlines()]
        self.assertTrue(any(e['event'] == 'tick' and e['drive_pwm'] == 0 for e in events))
        self.assertTrue((hz.journal.path / 'tx.bin').stat().st_size == sum(len(r) for r in hz.transport.tx_frames))

    def test_hold_extension_and_reversal_inside_hold_waits_for_neutral(self):
        hz = self.harness(max_hold_s=3.0)
        hz.run(0.8, keys={300: 'p'}); self.assertTrue(hz.trial.prepared())
        hz.run(2.0, keys={1000: 'l', 2500: 'l'})                    # second press extends the hold
        self.assertGreater(hz.trial.hold_until, 2.5 + 2.9)
        hz.run(3.0, keys={hz.ms + 5: 'r'})                          # reversal inside the hold
        self.assertIn(1, hz.steers()); self.assertIn(-1, hz.steers())
        self.assertTrue(any(t.note.startswith('reversal allowed') for t in hz.trial.adapter.trace))
        hz.invariants(self)

    def test_operator_stop_locks_and_new_sequence_alone_never_resumes(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'}); hz.run(0.6, keys={900: 'l'})
        hz.run(0.3, keys={hz.ms + 5: 's'})
        self.assertTrue(hz.trial.adapter.locked_by_fault); self.assertEqual(hz.trial.last_stop_reason, 'operator stop')
        self.assertTrue(hz.trial.stop_ack_observed)
        adc_at_stop = hz.board.adc
        hz.run(1.0, keys={hz.ms + 5: 'l', hz.ms + 300: '0'})
        self.assertAlmostEqual(hz.board.adc, adc_at_stop, delta=0.5, msg='moved after stop without prepare')
        self.assertTrue(all(d['action'] == DISARM for d in hz.tx()[-5:]))
        hz.invariants(self)

    def test_board_restart_locks_and_never_controls_new_boot_without_prepare(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'}); hz.run(0.5, keys={900: 'l'})
        hz.restart_board(58)
        hz.run(1.0, keys={hz.ms + 200: 'l'})
        ad = hz.trial.adapter
        self.assertTrue(ad.locked_by_fault); self.assertIn('재시작', ad.first_fault or ''); self.assertIsNone(hz.trial.target_cdeg)
        self.assertFalse(any(d.get('boot') == 58 and d['action'] in (ARM, CONTROL) for d in hz.tx()))
        self.assertTrue(any('target dropped' in t for _, t in hz.trial.events))
        hz.run(1.5, keys={hz.ms + 5: 'p'}); self.assertTrue(hz.trial.prepared())   # explicit re-prepare on the new boot
        hz.invariants(self)

    def test_stale_status_locks_and_resumed_frames_do_not_move(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'}); hz.run(0.4, keys={900: 'l'})
        hz.frames_on = False; hz.run(0.4); hz.frames_on = True
        self.assertTrue(hz.trial.adapter.locked_by_fault); self.assertIn('만료', hz.trial.adapter.first_fault or '')
        adc = hz.board.adc; hz.run(1.0)
        self.assertAlmostEqual(hz.board.adc, adc, delta=0.5)
        self.assertTrue(all(d['action'] == DISARM for d in hz.tx()[-3:]))
        hz.invariants(self)

    def test_cable_loss_exits_1_without_reconnect_and_reports_no_ack(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'}); hz.run(0.4, keys={900: 'l'})
        hz.transport.dead = True
        alive = hz.run(1.0)
        self.assertFalse(alive); self.assertEqual(hz.trial.exit_code, lst.EXIT_CONN); self.assertTrue(hz.transport.closed)
        self.assertFalse(hz.trial.stop_ack_observed); self.assertIn('no reconnect', hz.trial.exit_reason)
        n = len(hz.transport.tx_frames); hz.trial.tick(hz.ms / 1000 + 1); self.assertEqual(len(hz.transport.tx_frames), n)

    def test_quit_without_board_answer_exits_3(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'})
        hz.frames_on = False
        hz.run(1.0, keys={hz.ms + 5: 'q'})
        self.assertEqual(hz.trial.exit_code, lst.EXIT_NO_ACK); self.assertEqual(hz.trial.stop_attempts, 1)
        self.assertFalse(hz.trial.stop_ack_observed); self.assertEqual(hz.tx()[-1]['action'], DISARM)

    def test_target_outside_table_is_refused_before_the_core(self):
        hz = self.harness(angle_cdeg=1600)                         # adc 830 > usable 820
        hz.run(0.8, keys={300: 'p'}); hz.run(0.3, keys={900: 'l'})
        self.assertTrue(any('out of range' in t for _, t in hz.trial.events))
        self.assertEqual(hz.trial.core.last_sequence(), 0)         # never reached the C ABI
        # review fix: an out-of-range target is a planner fault -> stop + lock (C2.1 policy), not a silent refusal
        self.assertTrue(hz.trial.adapter.locked_by_fault); self.assertEqual(hz.tx()[-1]['action'], DISARM)
        self.assertAlmostEqual(hz.board.adc, 600, delta=0.5)


class ModelCli(Base):
    def test_scripted_model_run_real_clock(self):
        out = Path(self.tmp) / 'model'
        with redirect_stdout(io.StringIO()):
            rc = lst.main(['--model', '--script', '0.3:p,1.0:l,2.2:q', '--max-hold-s', '0.8', '--output-dir', str(out), '--quiet'])
        meta = json.loads((out / 'metadata.json').read_text())
        self.assertEqual((rc, meta['exit_code'], meta['data_kind'], meta['drive_pwm']), (0, 0, 'model_synthetic', 0))
        self.assertTrue(meta['stop_ack_observed']); self.assertGreater(meta['tx_frames'], 5)
        self.assertEqual(meta['last_stop_reason'], 'operator quit'); self.assertFalse(meta['termios_restore_on_close'])
        tx = (out / 'tx.bin').read_bytes()
        self.assertEqual(len(tx) % 19, 0)
        for i in range(0, len(tx), 19):
            d = sim.decode_tx(tx[i:i + 19]); self.assertFalse(d.get('malformed')); self.assertEqual((d['drive'], d['pwm']), (0, 0))


class IdentifyMode(Base):
    """--identify on a PTY: type150 frames from the board model are recognised, ASCII is reported, TX stays 0."""

    def feed_and_identify(self, payload_fn, seconds=0.4):
        master, slave = os.openpty()
        try:
            t = lst.PosixSerialTransport(os.ttyname(slave), check_identity=False)
            journal = lst.TrialJournal(Path(tempfile.mkdtemp(dir=self.tmp)) / 'id', 'live_vehicle', {})
            import threading
            stop = threading.Event()
            def writer():
                i = 0
                while not stop.is_set():
                    os.write(master, payload_fn(i)); i += 1; time.sleep(0.05)
            th = threading.Thread(target=writer, daemon=True); th.start()
            try:
                rep = lst.identify(t, journal, seconds)
            finally:
                stop.set(); th.join(1.0); t.close(); journal.close()
            echoed = select.select([master], [], [], 0.1)[0]
            return rep, echoed
        finally:
            os.close(slave); os.close(master)

    def test_type150_frames_identified_without_tx(self):
        board = sim.BoardModel(self.lib, 66, 512, -1)
        try:
            def frame(i):
                ms = 5000 + i * 50; board.tick(ms); f = board.status(ms)
                return f or b''
            rep, echoed = self.feed_and_identify(frame)
        finally:
            board.destroy()
        self.assertEqual(rep['verdict'], 'type150_identified'); self.assertEqual(rep['boot_ids'], [66])
        self.assertGreaterEqual(rep['type150_frames'], 3); self.assertEqual(rep['tx_bytes'], 0)
        self.assertTrue(rep['all_outputs_and_requests_zero']); self.assertEqual(rep['states'], [DISARMED])
        self.assertEqual(echoed, [], 'bytes went back to the board side')

    def test_ascii_board_reported_without_tx(self):
        rep, echoed = self.feed_and_identify(lambda i: ASCII_LINES[i % 2])
        self.assertEqual((rep['verdict'], rep['type150_frames'], rep['tx_bytes']), ('ascii_firmware', 0, 0))
        self.assertEqual(rep['ascii_marker'], 'F8 PID HILL HOLD READY'); self.assertEqual(echoed, [])

    def test_identify_cli_requires_output_dir_and_refuses_non_mega(self):
        with self.assertRaises(SystemExit), redirect_stderr(io.StringIO()):
            lst.main(['--identify', '--port', '/dev/null'])
        master, slave = os.openpty()
        try:
            out = Path(self.tmp) / 'idcli'
            with redirect_stderr(io.StringIO()):
                rc = lst.main(['--identify', '--port', os.ttyname(slave), '--output-dir', str(out), '--seconds', '0.2'])
            self.assertEqual(rc, lst.EXIT_CONN)           # identity check: a PTY is not the recorded Mega
            meta = json.loads((out / 'metadata.json').read_text()); self.assertEqual(meta['status'], 'error')
        finally:
            os.close(slave); os.close(master)


class ReviewFixes(Base):
    """Codex 05:27 review: public target API validation, stop-ack scope, library binary hash, terminal output."""

    def moving_harness(self):
        hz = self.harness(max_hold_s=3.0)
        hz.run(0.8, keys={300: 'p'}); self.assertTrue(hz.trial.prepared())
        self.assertTrue(hz.trial.command_target(400, hz.ms / 1000, 3.0)); hz.run(0.2)
        self.assertNotEqual(hz.board.adc, 600); return hz

    def assert_stopped_and_locked(self, hz, before_frames):
        t = hz.trial
        self.assertIsNone(t.target_cdeg); self.assertIsNone(t.hold_until); self.assertTrue(t.adapter.locked_by_fault)
        self.assertEqual(hz.tx()[-1]['action'], DISARM)
        adc_at_stop = hz.board.adc; hz.run(0.5)
        self.assertEqual([d for d in hz.tx()[before_frames:] if d.get('steer')], [], 'non-zero steer after rejection')
        self.assertAlmostEqual(hz.board.adc, adc_at_stop, delta=0.5)
        self.assertFalse(t.command_target(400, hz.ms / 1000, 3.0))          # no resume without prepare
        self.assertIsNone(t.target_cdeg)

    def test_out_of_range_target_while_moving_cancels_stops_locks(self):
        hz = self.moving_harness(); n = len(hz.transport.tx_frames)
        self.assertFalse(hz.trial.command_target(1600, hz.ms / 1000, 3.0))   # adc 830 > usable 820
        self.assertIn('out of range', hz.trial.last_stop_reason)
        self.assert_stopped_and_locked(hz, n)
        hz.run(0.6, keys={hz.ms + 5: 'p'}); self.assertTrue(hz.trial.prepared())   # explicit prepare resumes
        self.assertTrue(hz.trial.command_target(400, hz.ms / 1000, 3.0)); hz.invariants(self)

    def test_malformed_requests_cancel_stop_lock(self):
        cases = {'nan_hold': (400, None, float('nan')), 'inf_hold': (400, None, float('inf')), 'zero_hold': (400, None, 0.0),
                 'neg_hold': (400, None, -1.0), 'bool_angle': (True, None, 3.0), 'float_angle': (400.0, None, 3.0),
                 'wrap_angle': (65536 + 400, None, 3.0), 'nan_now': (400, float('nan'), 3.0), 'neg_now': (400, -1.0, 3.0),
                 'bool_hold': (400, None, True)}
        for name, (angle, now, hold) in cases.items():
            with self.subTest(name):
                hz = Harness(self.lib, tempfile.mkdtemp(dir=self.tmp), max_hold_s=3.0)
                try:
                    hz.run(0.8, keys={300: 'p'}); self.assertTrue(hz.trial.command_target(400, hz.ms / 1000, 3.0)); hz.run(0.2)
                    n = len(hz.transport.tx_frames)
                    self.assertFalse(hz.trial.command_target(angle, hz.ms / 1000 if now is None else now, hold))
                    self.assertIn('invalid target request', hz.trial.last_stop_reason)
                    self.assert_stopped_and_locked(hz, n)
                    self.assertEqual(hz.trial.core.last_sequence(), hz.trial.seq)    # nothing malformed reached the core
                finally:
                    hz.close()

    def test_nan_hold_never_keeps_a_target_alive(self):
        hz = self.harness(); hz.run(0.8, keys={300: 'p'})
        self.assertFalse(hz.trial.command_target(400, hz.ms / 1000, float('nan'))); hz.run(4.0)
        self.assertIsNone(hz.trial.target_cdeg); self.assertTrue(hz.trial.adapter.locked_by_fault)
        self.assertAlmostEqual(hz.board.adc, 600, delta=0.5)

    def test_valid_target_before_prepare_is_refused_without_stop_attempt(self):
        hz = self.harness(); hz.run(0.3)
        self.assertFalse(hz.trial.command_target(400, hz.ms / 1000, 3.0))
        self.assertEqual(hz.trial.stop_attempts, 0); self.assertIsNone(hz.trial.last_stop_reason)

    def test_old_stop_ack_is_history_not_current_after_reprepare_and_disconnect(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'}); hz.trial.stop_motion('first stop', hz.ms / 1000); hz.run(0.2)
        self.assertTrue(hz.trial.stop_ack_observed); first = dict(hz.trial.current_stop)
        hz.run(0.5, keys={hz.ms + 10: 'p'}); self.assertTrue(hz.trial.prepared())
        self.assertIsNone(hz.trial.current_stop); self.assertFalse(hz.trial.stop_ack_observed)
        self.assertEqual(hz.trial.stop_history[0]['ack_req'], first['ack_req']); self.assertIn('re-prepared', hz.trial.stop_history[0]['retired_by'])
        self.assertTrue(hz.trial.command_target(400, hz.ms / 1000, 3.0)); hz.run(0.25)
        hz.transport.dead = True; hz.run(0.2)
        self.assertEqual(hz.trial.exit_code, lst.EXIT_CONN); self.assertFalse(hz.trial.stop_ack_observed)
        self.assertIsNone(hz.trial.current_stop); self.assertEqual(len(hz.trial.stop_history), 1)
        self.assertIn('no stop ack', hz.trial.exit_reason)

    def test_repeated_stops_and_reboot_keep_ack_per_attempt(self):
        hz = self.harness()
        hz.run(0.8, keys={300: 'p'}); hz.trial.stop_motion('stop A', hz.ms / 1000); hz.run(0.2)
        a = hz.trial.current_stop; self.assertIsNotNone(a['ack_req'])
        hz.run(0.5, keys={hz.ms + 10: 'p'}); hz.trial.stop_motion('stop B', hz.ms / 1000)
        b = hz.trial.current_stop; self.assertIsNone(b['ack_req']); self.assertFalse(hz.trial.stop_ack_observed)
        self.assertEqual([h_['reason'] for h_ in hz.trial.stop_history], ['stop A'])
        hz.run(0.2); self.assertTrue(hz.trial.stop_ack_observed); self.assertGreater(b['ack_req'], a['ack_req'])
        # stop C, board reboots before any ack -> this attempt can never be confirmed
        hz.run(0.5, keys={hz.ms + 10: 'p'}); hz.trial.stop_motion('stop C', hz.ms / 1000)
        hz.restart_board(58); hz.run(0.3)
        self.assertFalse(hz.trial.stop_ack_observed); self.assertIn('rebooted', hz.trial.current_stop.get('note', ''))
        hz.run(0.5, keys={hz.ms + 10: 'q'})
        self.assertEqual(hz.trial.exit_code, lst.EXIT_OK)   # quit's own stop got its own DISARMED ack from boot 58
        self.assertEqual([h_['reason'] for h_ in hz.trial.stop_history], ['stop A', 'stop B', 'stop C'])
        self.assertEqual(hz.trial.current_stop['reason'], 'operator quit')

    def test_preflight_rejects_library_binary_that_differs_from_build_info(self):
        import shutil
        d = Path(self.tmp) / 'lib'; d.mkdir()
        shutil.copy(h.LIB_PATH, d / 'libsteer_host.so'); shutil.copy(h.LIB_PATH.parent / 'build_info.json', d / 'build_info.json')
        real = h.LIB_PATH
        try:
            h.LIB_PATH = d / 'libsteer_host.so'
            self.assertEqual(lst.preflight(SYNTH, 5.0, allow_synthetic=True)['status'], 'READY')   # faithful copy
            with open(d / 'libsteer_host.so', 'ab') as f: f.write(b'\0')                             # tampered COPY only
            r = lst.preflight(SYNTH, 5.0, allow_synthetic=True)
            self.assertEqual(r['status'], 'BLOCKED'); self.assertTrue(any('differs from build_info.library_sha256' in m for m in r['missing']))
        finally:
            h.LIB_PATH = real
        self.assertEqual(lst.preflight(SYNTH, 5.0, allow_synthetic=True)['status'], 'READY')   # real .so untouched

    def test_terminal_shows_phase_adc_target_and_errors(self):
        out = Path(self.tmp) / 'term'; buf = io.StringIO()
        with redirect_stdout(buf):
            rc = lst.main(['--model', '--script', '0.3:p,1.0:l,1.6:q', '--max-hold-s', '1.0', '--output-dir', str(out)])
        text = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn('ready', text); self.assertIn('adc=', text); self.assertIn('target +', text); self.assertIn('ARMED', text)
        self.assertIn('exit 0', text)
        meta = json.loads((out / 'metadata.json').read_text())
        self.assertIn('--model', meta['argv']); self.assertEqual(meta['final_stop_attempt']['reason'], 'operator quit')
        self.assertIsNotNone(meta['final_stop_attempt']['ack_req']); self.assertEqual(meta['stop_history'], [])


class StallGuard(Base):
    """움직임 없음 보호 (2026-09-18). 물리적으로 바퀴를 붙잡지 않고 보드 모형의 랙을 정지시켜 확인한다."""

    def test_normal_move_and_arrival_do_not_trip_the_guard(self):
        """정상 이동과 도달은 오검출하지 않는다. 도달 후 출력 0으로 오래 머물러도 차단되지 않는다."""
        hz = self.harness(max_hold_s=3.0)
        hz.run(4.0, keys={300: 'p', 600: 'l'}, until_finished=False)
        self.assertIsNone(hz.trial.stall_blocked, '정상 이동에서 정체로 오판')
        self.assertIn('AT_TARGET', hz.core_reasons())
        hz.invariants(self)
        hz.close()

    def test_frozen_rack_with_output_active_is_blocked(self):
        """출력이 나가는데 ADC가 멈춰 있으면 감시창 후 정지·DISARM·잠금."""
        hz = self.harness(max_hold_s=10.0)
        hz.run(0.8, keys={300: 'p'}, until_finished=False)
        hz.freeze()
        hz.run(4.0, keys={900: 'l'}, until_finished=False)
        b = hz.trial.stall_blocked
        self.assertIsNotNone(b, '정체인데 차단되지 않음')
        self.assertLess(b['progress_adc'], hz.trial.stall_min_progress_adc)
        self.assertGreaterEqual(b['waited_s'], hz.trial.stall_window_s)
        self.assertTrue(hz.trial.adapter.locked_by_fault, '차단 후 잠기지 않음')
        self.assertIsNone(hz.trial.target_cdeg, '차단 후 목표가 남아 있음')
        self.assertTrue(any('stall' in r for r in [hz.trial.last_stop_reason or '']))
        n = len(hz.tx())
        hz.run(1.0, until_finished=False)          # 차단 이후 구간에 비영점 조향 출력이 없어야 한다
        after = [d for d in hz.tx()[n:] if d.get('action') == CONTROL and d.get('steer')]
        self.assertEqual(after, [], '차단 후 비영점 조향 출력이 나감')
        hz.invariants(self)
        hz.close()

    def test_small_sensor_noise_does_not_count_as_progress(self):
        """정지 상태의 작은 잡음(±2)은 진행으로 인정하지 않는다."""
        hz = self.harness(max_hold_s=10.0)
        hz.run(0.8, keys={300: 'p'}, until_finished=False)
        hz.freeze(noise=2)
        hz.run(4.0, keys={900: 'l'}, until_finished=False)
        self.assertIsNotNone(hz.trial.stall_blocked, '잡음만 있는데 진행으로 인정')
        hz.close()

    def test_sequence_refresh_does_not_restart_the_window(self):
        """같은 목표의 sequence 갱신으로 감시창이 초기화되지 않는다."""
        hz = self.harness(max_hold_s=10.0)
        hz.run(0.8, keys={300: 'p'}, until_finished=False)
        hz.freeze()
        seq_before = hz.trial.seq
        hz.run(4.0, keys={900: 'l'}, until_finished=False)
        self.assertGreater(hz.trial.seq, seq_before + 1, '갱신이 일어나지 않아 시험 전제가 성립하지 않음')
        self.assertIsNotNone(hz.trial.stall_blocked, 'sequence 갱신으로 차단을 회피함')
        hz.close()

    def test_blocked_state_needs_prepare_not_just_a_new_target(self):
        """차단 후 새 목표만으로는 재가동되지 않는다. 'p' 이후에만 다시 움직인다."""
        hz = self.harness(max_hold_s=10.0)
        hz.run(0.8, keys={300: 'p'}, until_finished=False)
        hz.freeze()
        hz.run(4.0, keys={900: 'l'}, until_finished=False)
        self.assertIsNotNone(hz.trial.stall_blocked)
        n_before = len(hz.trial.adapter.trace)
        hz.run(1.0, keys={hz.ms + 100: 'r'}, until_finished=False)
        self.assertIsNone(hz.trial.target_cdeg, '차단 상태에서 새 목표가 수락됨')
        self.assertTrue(hz.trial.adapter.locked_by_fault)
        self.assertEqual([s for s in hz.steers()[n_before:] if s], [], '차단 후 비영점 출력이 나감')
        hz.close()

    def test_wrong_direction_travel_is_not_progress(self):
        """목표 반대 방향으로 가는 것은 진행으로 인정하지 않는다(부호가 뒤집힌 배선 모형)."""
        hz = self.harness(max_hold_s=10.0)
        hz.board.sign = -hz.board.sign          # 보드가 명령과 반대로 움직인다
        hz.run(0.8, keys={300: 'p'}, until_finished=False)
        hz.run(4.0, keys={900: 'l'}, until_finished=False)
        self.assertIsNotNone(hz.trial.stall_blocked, '반대 방향 이동을 진행으로 인정')
        self.assertLess(hz.trial.stall_blocked['progress_adc'], 0)
        hz.close()

    def test_rx_loss_still_ends_through_the_existing_protection(self):
        """수신이 끊기면 정체 보호가 아니라 기존 통신 보호로 끝난다."""
        hz = self.harness(max_hold_s=10.0)
        hz.run(0.8, keys={300: 'p'}, until_finished=False)
        hz.frames_on = False
        hz.run(6.0, keys={900: 'l'}, until_finished=False)
        self.assertTrue(hz.trial.finished or hz.trial.adapter.locked_by_fault)
        hz.close()

    def test_thresholds_come_from_the_recorded_evidence(self):
        """임계값 기본값과 근거가 코드에 남아 있고 metadata로 기록된다."""
        self.assertEqual(lst.LiveTrial.STALL_WINDOW_S, 1.5)
        self.assertEqual(lst.LiveTrial.STALL_MIN_PROGRESS_ADC, 8)
        src = (ROOT / 'tools/live_steering_trial.py').read_text()
        self.assertIn('105..346 ADC/s', src)
        self.assertIn('longest run with output active and NO motion', src)
        meta = lst.build_metadata('live', SYNTH, {'validated': None}, 400, {}, argv=[],
                                  stall={'window_s': 1.5, 'min_progress_adc': 8})
        self.assertEqual(meta['stall_guard'], {'window_s': 1.5, 'min_progress_adc': 8})

    def test_block_is_written_into_the_run_metadata(self):
        """차단이 일어나면 그 사실과 수치가 metadata에 남는다(2026-09-18 20:47 실차에서 누락됐던 항목)."""
        out = Path(self.tmp) / 'stallmeta'
        with redirect_stdout(io.StringIO()):
            rc = lst.main(['--model', '--script', '0.3:p,1.0:l,9:q', '--max-hold-s', '8.0',
                           '--stall-window-s', '0.4', '--stall-min-progress-adc', '9000', '--output-dir', str(out)])
        meta = json.loads((out / 'metadata.json').read_text())
        self.assertIsNotNone(meta.get('stall_blocked'), 'metadata에 차단 기록이 없음')
        self.assertIn('progress_adc', meta['stall_blocked'])
        self.assertEqual(meta['stall_guard'], {'window_s': 0.4, 'min_progress_adc': 9000})


if __name__ == '__main__':
    unittest.main()
