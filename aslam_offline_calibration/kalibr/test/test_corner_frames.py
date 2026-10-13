"""Tests for kalibr_common/CornerFrames.py (stdlib only: run with `python3 -m unittest` from this dir)."""

import importlib.util
import os
import unittest

# Loaded by path: importing the kalibr_common package pulls in the compiled aslam/numpy_eigen modules.
_MODULE_PATH = os.path.join(os.path.dirname(__file__), "..", "python", "kalibr_common", "CornerFrames.py")
_spec = importlib.util.spec_from_file_location("CornerFrames", _MODULE_PATH)
CornerFrames = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(CornerFrames)


def _frame(frame_id, cam_id):
    return {"frame_id": frame_id, "cam_id": cam_id, "corner_ids": [], "corners": []}


def _frame_ids(frames):
    return [fr["frame_id"] for fr in frames]


class SelectCameraFramesTest(unittest.TestCase):
    def test_keeps_only_the_requested_camera_sorted_by_frame_id(self):
        frames = [_frame(30, 0), _frame(10, 1), _frame(10, 0), _frame(20, 0)]

        selected = CornerFrames.selectCameraFrames(frames, cam_id=0, every=1)

        self.assertEqual(_frame_ids(selected), [10, 20, 30])
        self.assertTrue(all(fr["cam_id"] == 0 for fr in selected))

    def test_keeps_every_nth_timestamp(self):
        frames = [_frame(t, 0) for t in range(0, 60, 10)]

        selected = CornerFrames.selectCameraFrames(frames, cam_id=0, every=2)

        self.assertEqual(_frame_ids(selected), [0, 20, 40])

    def test_cameras_keep_the_same_timestamps_when_one_misses_a_frame(self):
        timestamps = range(0, 80, 10)
        frames = [_frame(t, 0) for t in timestamps] + [_frame(t, 1) for t in timestamps if t != 10]

        cam0 = CornerFrames.selectCameraFrames(frames, cam_id=0, every=2)
        cam1 = CornerFrames.selectCameraFrames(frames, cam_id=1, every=2)

        self.assertEqual(_frame_ids(cam0), [0, 20, 40, 60])
        self.assertEqual(_frame_ids(cam1), [0, 20, 40, 60])

    def test_returns_empty_when_camera_is_absent(self):
        frames = [_frame(10, 0), _frame(20, 0)]

        self.assertEqual(CornerFrames.selectCameraFrames(frames, cam_id=1, every=1), [])


if __name__ == "__main__":
    unittest.main()
