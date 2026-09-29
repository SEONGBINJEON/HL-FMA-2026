"""2026-09-19 field defect: with the drive axis active (DriveGate PWM>0) the board reports reason ACTIVE(10)
for steer=0 statuses, so the host reversal gate (which accepted only READY/NEUTRAL) never confirmed the
neutral and a correct steering sign reversal was held at output 0 until the stall guard stopped the run.
Real adapter + real firmware core board model, fake clock; no device."""
import sys, unittest, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tests')]
import steer_target_host as h, live_path_follow as live
from test_live_steering_trial import Harness


class ReversalWhileDriving(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.lib = h.load_library()

    def _run(self, drive_pwm):
        with tempfile.TemporaryDirectory() as d:
            x = Harness(self.lib, d, adc0=600, angle_cdeg=400)
            try:
                gate = live.DriveGate(drive_pwm, x.trial.adapter, clock=lambda: x.fake.now); x.trial.client.encoder = gate
                x.run(.5, keys={100: 'p'}); self.assertTrue(x.trial.prepared())
                # left target; keep the gate open every tick so drive stays active like the follower does
                for _ in range(40):
                    if not x.trial.adapter.locked_by_fault: x.trial.command_target(400, x.ms / 1000, 3); gate.open(x.ms / 1000)
                    x.run(.05)
                first_sign = next((s for s in x.steers() if s), None); self.assertIsNotNone(first_sign)
                # now the opposite direction: a real sign reversal while drive is active
                for _ in range(80):
                    if not x.trial.adapter.locked_by_fault: x.trial.command_target(-400, x.ms / 1000, 3); gate.open(x.ms / 1000)
                    x.run(.05)
                steers = x.steers(); drives = [c.get('drive') for c in x.controls()]
                return dict(first=first_sign, steers=steers, drives=drives, locked=x.trial.adapter.locked_by_fault,
                            stall=x.trial.stall_blocked, reasons=set(x.board.reasons_seen), fault=x.trial.adapter.first_fault,
                            notes=[t.note for t in x.trial.adapter.trace if t.note.startswith('reversal')])
            finally: x.close()

    def test_reversal_completes_with_drive_active(self):
        r = self._run(50)
        self.assertIn(1, r['drives'], 'drive frames must have been sent (ACTIVE reason path)')
        self.assertIn(-r['first'], r['steers'], f"opposite steer never sent: {r['fault']} {r['notes'][:3]}")
        self.assertFalse(r['locked'], r['fault']); self.assertIsNone(r['stall'])
        self.assertNotIn(7, r['reasons'], 'board must never see a direct REVERSAL error')
        self.assertTrue(any(n.startswith('reversal allowed') for n in r['notes']))
        # no direct sign flip in the byte stream: at least one steer=0 frame between opposite signs
        nz = [(i, s) for i, s in enumerate(r['steers']) if s]
        for (i, a), (j, b) in zip(nz, nz[1:]):
            if a != b: self.assertGreater(j - i, 1)

    def test_reversal_still_completes_steering_only(self):
        r = self._run(0)
        self.assertNotIn(1, r['drives']); self.assertIn(-r['first'], r['steers']); self.assertFalse(r['locked'])
        self.assertNotIn(7, r['reasons'])


if __name__ == '__main__': unittest.main()
