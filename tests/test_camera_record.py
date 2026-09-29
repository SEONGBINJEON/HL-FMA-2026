"""Hardware-free lifecycle and failure tests for the D455 recorder."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

SPEC = importlib.util.spec_from_file_location("camera_record", Path(__file__).parents[1] / "tools" / "camera_record.py")
camera = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(camera)


class Profile:
    def __init__(self, stream="color"): self.stream = stream
    def stream_type(self): return self.stream
    def stream_index(self): return 0
    def format(self): return "rgb8" if self.stream == "color" else "z16"
    def fps(self): return 30
    def as_video_stream_profile(self): return self
    def is_video_stream_profile(self): return True
    def width(self): return 640
    def height(self): return 480
    def get_intrinsics(self):
        return NS(width=640, height=480, fx=600., fy=601., ppx=320., ppy=240., model="brown", coeffs=[0.] * 5)
    def get_extrinsics_to(self, other):
        return NS(rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.], translation=[.01,0.,0.])


class Device:
    def __init__(self, name="Intel RealSense D455", serial="D455-TEST"):
        self.values = dict(name=name, serial_number=serial, firmware_version="test", usb_type_descriptor="3.2")
    def supports(self, key): return key in self.values
    def get_info(self, key): return self.values[key]
    def query_sensors(self): return [NS(get_stream_profiles=lambda: [Profile()])]
    def first_depth_sensor(self): return NS(get_depth_scale=lambda: .001)


class Frame:
    def __init__(self, number, stream="color"): self.number, self.stream = number, stream
    def get_profile(self): return Profile(self.stream)
    def get_frame_number(self): return self.number
    def get_timestamp(self): return self.number * 33.3
    def get_frame_timestamp_domain(self): return "hardware_clock"


class FakeSDK:
    camera_info = NS(name="name", serial_number="serial_number", firmware_version="firmware_version", usb_type_descriptor="usb_type_descriptor")
    stream = NS(color="color", depth="depth")
    format = NS(rgb8="rgb8", z16="z16")
    def __init__(self, frames=(1, 3), error=None, name="Intel RealSense D455", resolve=True, start_error=False):
        self.devices = [Device(name)]
        self.clock = 0.
        self.frames = iter(frames)
        self.error = error
        self.resolve = resolve
        self.start_error = start_error
        self.stops = 0
        self.starts = 0
        self.serial = None
        self.bag = None
        self.enabled = []
        self.omit_depth = False
    def context(self): return NS(query_devices=lambda: self.devices)
    def pipeline(self, ctx): return self
    def config(self): return self
    def enable_device(self, serial): self.serial = serial
    def disable_all_streams(self): pass
    def enable_stream(self, *args): self.enabled.append(args[0])
    def pipeline_wrapper(self, pipeline): return pipeline
    def can_resolve(self, wrapper): return self.resolve
    def enable_record_to_file(self, path):
        self.bag = Path(path)
        # SDK 2.58 rejects .bag before starting; mirror the observed API contract.
        if self.bag.suffix != ".db3":
            raise RuntimeError("Output file must have .db3 extension")
    def start(self, config):
        self.starts += 1
        if self.start_error: raise RuntimeError("start failed")
        self.bag.write_bytes(b"fake bag")
        return NS(get_device=lambda: self.devices[0], get_streams=lambda: [Profile(s) for s in self.enabled],
                  get_stream=lambda stream: Profile(stream))
    def try_wait_for_frames(self, timeout_ms):
        self.clock += timeout_ms / 1000.
        if self.error is not None: raise self.error
        try:
            number = next(self.frames)
            return True, [Frame(number, s) for s in self.enabled if not (s == "depth" and self.omit_depth)]
        except StopIteration: return False, []
    def stop(self): self.stops += 1
    def monotonic(self): return self.clock


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "new_session"
    def run_record(self, sdk, duration=.5):
        return camera.record(sdk, serial="D455-TEST", output_dir=self.output,
                             duration=duration, monotonic=sdk.monotonic, wall_ns=lambda: 123456789)
    def test_finite_session_records_calibration_and_observed_gaps(self):
        sdk = FakeSDK()
        result = self.run_record(sdk)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["elapsed_seconds"], .5)
        self.assertEqual(sdk.stops, 1)
        self.assertEqual(sdk.serial, "D455-TEST")
        self.assertEqual(result["bag_file"], "camera.db3")
        self.assertEqual(result["recording_format"], "realsense_sdk_rosbag2_db3")
        persisted = json.loads((self.output / "metadata.json").read_text())
        self.assertEqual(persisted["profiles"][0]["intrinsics"]["fy"], 601.)
        self.assertEqual(persisted["observed_streams"]["color"]["frame_number_gaps"], 1)
        frames = [json.loads(line) for line in (self.output / "frames.jsonl").read_text().splitlines()]
        self.assertEqual([f["frame_number"] for f in frames], [1, 3])
        self.assertEqual(frames[0]["host_unix_ns"], 123456789)
        self.assertEqual(frames[0]["timestamp_domain"], "hardware_clock")
    def test_existing_output_is_untouched_and_never_starts(self):
        self.output.mkdir()
        sentinel = self.output / "camera.bag"
        sentinel.write_bytes(b"original")
        sdk = FakeSDK()
        with self.assertRaises(camera.RecordingError): self.run_record(sdk)
        self.assertEqual(sentinel.read_bytes(), b"original")
        self.assertEqual(sdk.starts, 0)
    def test_wrong_model_does_not_create_files_or_start(self):
        sdk = FakeSDK(name="Intel RealSense D455f")
        with self.assertRaises(camera.RecordingError): self.run_record(sdk)
        self.assertFalse(self.output.exists())
        self.assertEqual(sdk.starts, 0)
    def test_unsupported_profile_is_rejected_before_creating_session(self):
        sdk = FakeSDK(resolve=False)
        with self.assertRaisesRegex(camera.RecordingError, "640x480@30"):
            self.run_record(sdk)
        self.assertFalse(self.output.exists())
    def test_capture_failure_stops_and_preserves_failed_metadata(self):
        sdk = FakeSDK(error=RuntimeError("disconnected"))
        with self.assertRaisesRegex(camera.RecordingError, "disconnected"):
            self.run_record(sdk)
        self.assertEqual(sdk.stops, 1)
        self.assertEqual(json.loads((self.output / "metadata.json").read_text())["status"], "failed")
    def test_start_failure_attempts_cleanup_and_saves_error(self):
        sdk = FakeSDK(start_error=True)
        with self.assertRaisesRegex(camera.RecordingError, "start failed"):
            self.run_record(sdk)
        self.assertEqual(sdk.stops, 1)
        self.assertEqual(json.loads((self.output / "metadata.json").read_text())["bag_bytes"], 0)
    def test_ctrl_c_stops_and_marks_partial_recording(self):
        sdk = FakeSDK(error=KeyboardInterrupt())
        result = self.run_record(sdk)
        self.assertEqual(result["status"], "interrupted")
        self.assertEqual(sdk.stops, 1)
    def test_no_frames_is_not_reported_as_success(self):
        with self.assertRaises(camera.RecordingError): self.run_record(FakeSDK(frames=()))
    def test_missing_native_bag_is_not_reported_as_success(self):
        sdk = FakeSDK()
        def stop_without_bag():
            sdk.stops += 1
            sdk.bag.unlink()
        sdk.stop = stop_without_bag
        with self.assertRaisesRegex(camera.RecordingError, "camera.db3"):
            self.run_record(sdk)
        self.assertEqual(json.loads((self.output / "metadata.json").read_text())["status"], "failed")
    def test_nonfinite_or_nonpositive_duration_never_starts(self):
        for duration in (0., -1., float("nan"), float("inf")):
            with self.subTest(duration=duration):
                sdk = FakeSDK()
                with self.assertRaises(camera.RecordingError): self.run_record(sdk, duration)
                self.assertEqual(sdk.starts, 0)
    def test_initial_frame_then_stall_fails_and_reports_stream_gap(self):
        sdk = FakeSDK(frames=(1,))
        with self.assertRaisesRegex(camera.RecordingError, "수신 중단"):
            self.run_record(sdk, duration=3.)
        metadata = json.loads((self.output / "metadata.json").read_text())
        counters = metadata["observed_streams"]["color"]
        self.assertEqual(metadata["status"], "failed")
        self.assertEqual(metadata["stalled_stream"], "color")
        self.assertEqual(counters["last_seen_elapsed_seconds"], .25)
        self.assertGreaterEqual(counters["max_observed_gap_seconds"], 2.)
        self.assertGreater(counters["effective_fps"], 0.)
        self.assertEqual(sdk.stops, 1)
    def test_depth_records_both_streams_scale_and_extrinsics(self):
        sdk = FakeSDK()
        metadata = camera.record(sdk, serial="D455-TEST", output_dir=self.output,
                                 duration=.5, with_depth=True, monotonic=sdk.monotonic)
        self.assertEqual(metadata["depth_scale_meters"], .001)
        self.assertEqual(metadata["depth_to_color_extrinsics"]["translation_meters"], [.01,0.,0.])
        self.assertEqual(metadata["observed_streams"]["depth"]["unique_frames"], 2)
    def test_depth_stall_detected_even_while_color_continues(self):
        sdk = FakeSDK(frames=range(30))
        sdk.omit_depth = True
        with self.assertRaisesRegex(camera.RecordingError, "depth"):
            camera.record(sdk, serial="D455-TEST", output_dir=self.output,
                          duration=3., with_depth=True, monotonic=sdk.monotonic)
        self.assertEqual(sdk.stops, 1)
    def test_gap_threshold_must_allow_two_frame_periods(self):
        sdk = FakeSDK()
        with self.assertRaises(camera.RecordingError):
            camera.record(sdk, serial="D455-TEST", output_dir=self.output,
                          fps=1, max_frame_gap=1., monotonic=sdk.monotonic)
        self.assertEqual(sdk.starts, 0)


if __name__ == "__main__":
    unittest.main()
