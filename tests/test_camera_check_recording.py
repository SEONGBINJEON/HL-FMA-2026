import importlib.util
from datetime import timedelta
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock
import zlib


SPEC = importlib.util.spec_from_file_location("camera_check_recording", Path(__file__).resolve().parents[1] / "tools/camera_check_recording.py")
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


def fake_sdk(bag, timestamps):
    sdk = MagicMock()
    sdk.stream.color = "color"
    sdk.playback_status.stopped = "stopped"
    pipeline = sdk.pipeline.return_value
    active = pipeline.start.return_value
    device = active.get_device.return_value
    sdk.config.return_value.resolve.return_value.get_device.return_value = device
    device.is_playback.return_value = True
    playback = device.as_playback.return_value
    playback.file_name.return_value = str(bag)
    playback.get_duration.return_value = timedelta(seconds=5)
    playback.get_position.return_value = 5_000_000_000
    playback.current_status.return_value = "stopped"
    profile = MagicMock()
    profile.stream_type.return_value = "color"
    profile.stream_index.return_value = 0
    profile.format.return_value = "format.rgb8"
    profile.fps.return_value = 30
    profile.is_video_stream_profile.return_value = True
    profile.as_video_stream_profile.return_value.width.return_value = 1
    profile.as_video_stream_profile.return_value.height.return_value = 1
    active.get_streams.return_value = [profile]
    batches = []
    for number, timestamp in enumerate(timestamps, 1):
        frame = MagicMock()
        frame.get_profile.return_value = profile
        frame.get_frame_number.return_value = number
        frame.get_timestamp.return_value = timestamp
        frame.get_frame_timestamp_domain.return_value = "hardware_clock"
        video = frame.as_video_frame.return_value
        video.get_width.return_value = video.get_height.return_value = 1
        video.get_stride_in_bytes.return_value = 3
        video.get_data.return_value = bytes([number, 20, 30])
        batches.append((True, [frame]))
    pipeline.try_wait_for_frames.side_effect = batches + [(False, None)]
    return sdk, pipeline, playback


class PlaybackTests(unittest.TestCase):
    def test_sample_is_file_only_and_preview_uses_later_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "camera.bag"
            bag.write_bytes(b"fixture")
            preview = Path(folder) / "preview.png"
            sdk, pipeline, playback = fake_sdk(bag, [100, 1100, 2200, 2300])
            report = checker.check_recording(sdk, bag=bag, preview=preview, max_frames=3)
            self.assertEqual(report["status"], "passed")
            self.assertEqual(report["verification_scope"], "sampled")
            self.assertEqual(report["preview"]["frame_number"], 3)
            self.assertTrue(report["preview"]["at_least_two_seconds_after_first_color"])
            sdk.config.return_value.enable_device_from_file.assert_called_once_with(str(bag), repeat_playback=False)
            sdk.context.assert_not_called()
            playback.set_real_time.assert_called_with(False)
            pipeline.stop.assert_called_once()
            png = preview.read_bytes()
            self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")
            data_start = png.index(b"IDAT") + 4
            data_length = int.from_bytes(png[data_start - 8:data_start - 4], "big")
            self.assertEqual(zlib.decompress(png[data_start:data_start + data_length]), b"\x00\x03\x14\x1e")

    def test_eof_with_frames_is_complete_but_zero_frames_fails(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "camera.bag"
            bag.write_bytes(b"fixture")
            for frames, status in [([100], "passed"), ([], "failed")]:
                sdk, pipeline, _ = fake_sdk(bag, frames)
                report = checker.check_recording(sdk, bag=bag)
                self.assertEqual(report["status"], status)
                self.assertEqual(report["verification_scope"], "complete")
                pipeline.stop.assert_called_once()

    def test_timeout_fails_and_stops_pipeline(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "camera.bag"
            bag.write_bytes(b"fixture")
            sdk, pipeline, playback = fake_sdk(bag, [])
            playback.current_status.return_value = "playing"
            times = iter([0, 16, 16])
            report = checker.check_recording(sdk, bag=bag, monotonic=lambda: next(times))
            self.assertEqual(report["status"], "failed")
            self.assertEqual(report["error_type"], "TimeoutError")
            pipeline.stop.assert_called_once()

    def test_existing_preview_is_refused_before_sdk_start(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "camera.bag"
            bag.write_bytes(b"fixture")
            preview = Path(folder) / "preview.png"
            preview.write_bytes(b"keep")
            sdk = MagicMock()
            with self.assertRaises(ValueError):
                checker.check_recording(sdk, bag=bag, preview=preview)
            sdk.pipeline.assert_not_called()
            self.assertEqual(preview.read_bytes(), b"keep")

    def test_sdk_read_error_is_not_mistaken_for_end_of_file(self):
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "camera.bag"
            bag.write_bytes(b"fixture")
            sdk, pipeline, playback = fake_sdk(bag, [])
            playback.current_status.return_value = "stopped"
            pipeline.try_wait_for_frames.side_effect = RuntimeError("Corrupt recording")
            report = checker.check_recording(sdk, bag=bag)
            self.assertEqual(report["status"], "failed")
            self.assertEqual(report["error"], "Corrupt recording")
            self.assertFalse(report["end_of_file_observed"])
            pipeline.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
