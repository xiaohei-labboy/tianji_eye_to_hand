"""Conventions: T_A_B maps points in B into A; all translations in metres."""
import itertools
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def validate_T(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError('Transform must be finite 4x4')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError('Invalid homogeneous row')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or abs(np.linalg.det(R) - 1) > 1e-5:
        raise ValueError('Rotation is not in SO(3)')
    return T


def inv(T):
    T = validate_T(T)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def pack(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def unpack(v):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rotation.from_rotvec(v[:3]).as_matrix(), v[3:]
    return T


def delta(a, b):
    d = inv(a) @ b
    return float(np.linalg.norm(d[:3, 3])), float(np.rad2deg(Rotation.from_matrix(d[:3, :3]).magnitude()))


def average(Ts):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_matrix(np.array([T[:3, :3] for T in Ts])).mean().as_matrix()
    out[:3, 3] = np.mean([T[:3, 3] for T in Ts], axis=0)
    return out


def excitation(As):
    vectors = [Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec()
               for a, b in itertools.combinations(As, 2)]
    s = np.linalg.svd(vectors, compute_uv=False)
    max_angle = float(np.rad2deg(np.max(np.linalg.norm(vectors, axis=1))))
    ratio = float(s[1] / max(s[0], 1e-12))
    if max_angle < 15 or ratio < .1:
        raise ValueError(f'Insufficient rotation diversity: max={max_angle:.1f}deg, second/first={ratio:.3f}. '
                         'Collect rotations about at least two nonparallel axes.')
    return {'max_pair_rotation_deg': max_angle, 'rotation_axis_singular_ratio': ratio}


def residuals(X, Y, As, Bs):
    rows = []
    for A, B in zip(As, Bs):
        E = inv(A @ Y) @ X @ B
        rows.extend(E[:3, 3] / .001)
        rows.extend(Rotation.from_matrix(E[:3, :3]).as_rotvec() / np.deg2rad(1))
    return np.asarray(rows)


def metrics(X, Y, As, Bs, indices):
    records = []
    for i in indices:
        t, r = delta(As[i] @ Y, X @ Bs[i])
        records.append({'sample_index': int(i), 'translation_mm': t * 1000, 'rotation_deg': r})
    t = np.array([v['translation_mm'] for v in records])
    r = np.array([v['rotation_deg'] for v in records])
    return {'count': len(records), 'translation_rms_mm': float(np.sqrt(np.mean(t*t))),
            'translation_max_mm': float(t.max()), 'rotation_rms_deg': float(np.sqrt(np.mean(r*r))),
            'rotation_max_deg': float(r.max()), 'per_sample': records}


def solve(samples, max_mm=5, max_deg=2):
    """X B_i = A_i Y. Hold out every fifth sample, never use it for fitting."""
    if len(samples) < 12:
        raise ValueError('Need at least 12 samples; 20-30 varied, stationary samples recommended.')
    As = [validate_T(s['T_base_flange']) for s in samples]
    Bs = [validate_T(s['T_camera_board']) for s in samples]
    validation = [i for i in range(len(samples)) if i % 5 == 4]
    train = [i for i in range(len(samples)) if i not in validation]
    aa, bb = [As[i] for i in train], [Bs[i] for i in train]
    diversity = excitation(aa)
    ainv = [inv(a) for a in aa]
    methods = [('PARK', cv2.CALIB_HAND_EYE_PARK), ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
               ('TSAI', cv2.CALIB_HAND_EYE_TSAI)]
    candidates = []
    for name, method in methods:
        try:
            R, t = cv2.calibrateHandEye([a[:3, :3] for a in ainv], [a[:3, 3] for a in ainv],
                                       [b[:3, :3] for b in bb], [b[:3, 3] for b in bb], method=method)
            X = np.eye(4)
            X[:3, :3], X[:3, 3] = R, t.ravel()
            validate_T(X)
            Y = average([inv(a) @ X @ b for a, b in zip(aa, bb)])
            fit = least_squares(lambda v: residuals(unpack(v[:6]), unpack(v[6:]), aa, bb),
                                np.r_[pack(X), pack(Y)], loss='soft_l1', f_scale=1,
                                max_nfev=300, x_scale='jac')
            if not fit.success or not np.all(np.isfinite(fit.x)):
                continue
            X, Y = unpack(fit.x[:6]), unpack(fit.x[6:])
            candidates.append((float(fit.cost), name, X, Y))
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            continue
    if not candidates:
        raise ValueError('All solvers failed: inspect transforms, corner numbering, and rotation diversity.')
    _, method, X, Y = min(candidates, key=lambda v: v[0])
    training = metrics(X, Y, As, Bs, train)
    heldout = metrics(X, Y, As, Bs, validation)
    all_data = metrics(X, Y, As, Bs, range(len(samples)))
    passed = all_data['translation_max_mm'] <= max_mm and all_data['rotation_max_deg'] <= max_deg
    return {'convention': 'T_A_B maps B coordinates into A; translation unit: metre',
            'camera_frame': 'RealSense color optical frame (x right, y down, z forward)',
            'T_base_camera': X.tolist(), 'T_camera_base': inv(X).tolist(),
            'T_flange_board': Y.tolist(), 'initializer': method,
            'rotation_diversity': diversity, 'training': training, 'validation': heldout,
            'all_samples': all_data, 'thresholds': {'max_mm': max_mm, 'max_deg': max_deg},
            'quality_passed': bool(passed),
            'note': 'Consistency check, not a guarantee of absolute robot/camera accuracy. '
                    'Validation samples were excluded from fitting.'}
