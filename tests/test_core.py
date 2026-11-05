import sys
from pathlib import Path
import unittest
from unittest.mock import patch
from collections import deque
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from board import RingBoard, GRID, ANCHORS, project, estimate_pose, Observation
from calibration import solve, inv, delta, unpack
from robot_adapter import Tianji


def fixture_board():
    image = np.full((720, 800), 30, np.uint8)
    image[60:660, 100:700] = 220
    for y in range(10):
        for x in range(10):
            image[110+y*50:110+(y+1)*50, 150+x*50:150+(x+1)*50] = 220 if (x+y)%2 == 0 else 20
    image[260:460, 300:500] = 20
    # Deliberately arbitrary central artwork. Never decoded or used as corners.
    rng = np.random.default_rng(9)
    image[290:430, 330:470] = cv2.resize((rng.random((28,28)) > .5).astype(np.uint8)*200, (140,140), interpolation=cv2.INTER_NEAREST)
    return cv2.GaussianBlur(image, (3, 3), .6)


def fixture_calibration(n=25, noise=False):
    rng = np.random.default_rng(12)
    X = unpack(np.r_[.2, -.5, .3, .7, -.15, .6])
    Y = unpack(np.r_[.3, .1, -.2, .04, .02, .12])
    samples = []
    for i in range(n):
        A = unpack(np.r_[rng.normal(0, .5, 3), rng.uniform(-.3, .5, 3)])
        B = inv(X) @ A @ Y
        if noise:
            B = B @ unpack(np.r_[rng.normal(0, np.deg2rad(.02), 3), rng.normal(0, .00005, 3)])
        samples.append({'T_base_flange': A.tolist(), 'T_camera_board': B.tolist()})
    return samples, X, Y


class CalibrationTests(unittest.TestCase):
    def test_eye_to_hand_direction_and_units(self):
        s, X, Y = fixture_calibration()
        out = solve(s)
        t, r = delta(X, np.array(out['T_base_camera']))
        self.assertLess(t, 1e-8)
        self.assertLess(r, 1e-6)
        self.assertTrue(out['quality_passed'])
        self.assertEqual(out['validation']['count'], 5)
        np.testing.assert_allclose(out['T_camera_base'], inv(X), atol=1e-8)
        np.testing.assert_allclose(out['T_flange_board'], Y, atol=1e-8)

    def test_realistic_noise(self):
        s, X, _ = fixture_calibration(noise=True)
        out = solve(s)
        t, r = delta(X, np.array(out['T_base_camera']))
        self.assertLess(t, .001)
        self.assertLess(r, .1)
        self.assertTrue(out['quality_passed'])

    def test_holdout_not_used_and_bad_sample_rejected(self):
        s, X, _ = fixture_calibration()
        bad = np.array(s[4]['T_camera_board'])
        bad[:3, 3] += [.1, .1, 0]
        s[4]['T_camera_board'] = bad.tolist()
        out = solve(s)
        np.testing.assert_allclose(out['T_base_camera'], X, atol=1e-8)
        self.assertFalse(out['quality_passed'])
        self.assertGreater(out['validation']['translation_max_mm'], 100)

    def test_insufficient_rotation_rejected(self):
        s, _, _ = fixture_calibration()
        for sample in s:
            sample['T_base_flange'] = np.eye(4).tolist()
        with self.assertRaisesRegex(ValueError, 'rotation diversity'):
            solve(s)

    def test_single_axis_rejected(self):
        s, _, _ = fixture_calibration()
        for i, sample in enumerate(s):
            sample['T_base_flange'] = unpack([0,0,i*.04,0,0,0]).tolist()
        with self.assertRaisesRegex(ValueError, 'rotation diversity'):
            solve(s)


class BoardTests(unittest.TestCase):
    def test_auto_detect_56_and_no_central_points(self):
        b = RingBoard()
        ob = b.initialize(fixture_board())
        self.assertIsNotNone(ob)
        self.assertEqual(len(ob.ids), 56)
        self.assertFalse(np.any(np.all((GRID[ob.ids] > 2) & (GRID[ob.ids] < 8), axis=1)))

    def test_tracking_and_lost_state(self):
        im = fixture_board()
        b = RingBoard()
        old = b.initialize(im)
        affine = np.float32([[1,0,4],[0,1,-3]])
        moved = cv2.warpAffine(im, affine, (im.shape[1], im.shape[0]))
        new = b.update(moved)
        self.assertIsNotNone(new)
        np.testing.assert_allclose(project(new.H, ANCHORS), project(old.H, ANCHORS)+[4,-3], atol=.2)
        self.assertIsNone(b.update(np.zeros_like(im)))
        self.assertTrue(b.lost)
        with self.assertRaises(ValueError):
            b.initialize(im)
        self.assertIsNotNone(b.initialize(im, project(old.H, ANCHORS)))

    def test_central_artwork_does_not_change_corners(self):
        im = fixture_board()
        b = RingBoard()
        old = b.initialize(im)
        im[280:440,320:480] = 255
        new = b.initialize(im, project(old.H, ANCHORS))
        self.assertEqual(len(new.ids), 56)
        np.testing.assert_allclose(old.pixels, new.pixels, atol=.05)

    def test_partial_huge_occlusion_rejected(self):
        im = fixture_board()
        b = RingBoard()
        ob = b.initialize(im)
        im[:, 100:400] = 30
        self.assertIsNone(b.initialize(im, project(ob.H, ANCHORS)))

    def test_pnp_metric_scale_and_rotation(self):
        K = np.array([[1100.,0,640],[0,1100,360],[0,0,1]])
        T = unpack([.3,-.4,.15,-.022,-.02,.32])
        obj = np.column_stack((GRID*.005,np.zeros(len(GRID))))
        pixels = cv2.projectPoints(obj, cv2.Rodrigues(T[:3,:3])[0], T[:3,3],K,np.zeros(5))[0].reshape(-1,2)
        ob = Observation(np.eye(3),np.arange(56),pixels,0,17)
        out, error = estimate_pose(ob,K)
        np.testing.assert_allclose(T, out, atol=1e-6)
        self.assertLess(error, 1e-4)


class FeedbackTests(unittest.TestCase):
    def mock_robot(self):
        r = object.__new__(Tianji)
        r.history = deque({'host_monotonic': float(t), 'frame_serial': i+1, 'joints_deg': [0.]*7,
                           'velocities_deg_s': [0.]*7, 'state': 3, 'error_code': 0}
                          for i,t in enumerate(np.linspace(9.4,10.02,32)))
        r.read = lambda: None
        r.fk = lambda q: unpack([0,0,0,q[0]*.001,0,0])
        return r

    def test_stationary_feedback_accepted(self):
        r = self.mock_robot()
        with patch('robot_adapter.time.monotonic',return_value=10.03):
            T, data = r.stationary_pose(10.)
        np.testing.assert_allclose(T,np.eye(4))
        self.assertGreater(len(data['feedback_window']),8)

    def test_stale_and_moving_feedback_rejected(self):
        r = self.mock_robot()
        with patch('robot_adapter.time.monotonic',return_value=10.5):
            with self.assertRaises(ValueError): r.stationary_pose(10.)
        r.history[-1]['velocities_deg_s'][0] = 1
        with patch('robot_adapter.time.monotonic',return_value=10.03):
            with self.assertRaisesRegex(ValueError,'moving'): r.stationary_pose(10.)


if __name__ == '__main__':
    unittest.main()
