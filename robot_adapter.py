"""TJ SDK feedback and local FK only. No servo/motion/drag/error-clear commands."""
import ctypes
import hashlib
import importlib
import ipaddress
from pathlib import Path
import sys
import time
from collections import deque
import numpy as np
from calibration import validate_T, delta

DEFAULT_SDK = Path('/media/zhh/D44E3D434E3D2022/TJ_FX_ROBOT_CONTRL_SDK')
DEFAULT_CONFIG = DEFAULT_SDK / 'CommonConfig/ccs_m6_40.MvKDCfg'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Tianji:
    def __init__(self, arm, sdk_root=DEFAULT_SDK, model_config=DEFAULT_CONFIG):
        if arm not in ('A', 'B'):
            raise ValueError('Arm must be A (left) or B (right)')
        self.index = 0 if arm == 'A' else 1
        self.arm = arm
        self.config = Path(model_config).resolve()
        self.sdk_root = Path(sdk_root).resolve()
        if not self.config.is_file():
            raise FileNotFoundError(self.config)
        sys.path.insert(0, str(self.sdk_root))
        kine_module = importlib.import_module('SDK_PYTHON.fx_kine')
        robot_module = importlib.import_module('SDK_PYTHON.fx_robot')
        self.kine = kine_module.Marvin_Kine()
        cfg = self.kine.load_config(self.index, str(self.config))
        if cfg is None or not self.kine.initial_kine(cfg['TYPE'][self.index], cfg['DH'][self.index],
                                                   cfg['PNVA'][self.index], cfg['BD'][self.index]):
            raise RuntimeError('SDK kinematics initialization failed')
        if not self.kine.remove_tool_kine():
            raise RuntimeError('Cannot select flange as local FK endpoint')
        self.robot = robot_module.Marvin_Robot()
        status, endian = self.robot.check_sdk_type_compat()
        if status < 0:
            raise RuntimeError(f'SDK ABI mismatch: {status}')
        self.dcss = robot_module.DCSS()
        # Native signatures explicitly match the maintained C++ header.
        lib = self.robot.robot
        lib.OnLinkTo.argtypes = [ctypes.c_ubyte] * 4
        lib.OnLinkTo.restype = ctypes.c_bool
        lib.OnRelease.argtypes, lib.OnRelease.restype = [], ctypes.c_bool
        lib.OnGetBuf.argtypes = [ctypes.POINTER(robot_module.DCSS)]
        lib.OnGetBuf.restype = ctypes.c_bool
        lib.OnGetIntPara.argtypes = [ctypes.POINTER(ctypes.c_char), ctypes.POINTER(ctypes.c_long)]
        lib.OnGetIntPara.restype = ctypes.c_long
        self.connected = False
        self.connect_attempted = False
        self.history = deque(maxlen=240)
        self.metadata = {'arm': arm, 'base_frame': f'arm_{arm}_SDK_kinematic_base',
                         'endpoint_frame': f'arm_{arm}_flange', 'model_config': str(self.config),
                         'model_config_sha256': digest(self.config), 'robot_type': cfg['TYPE'][self.index],
                         'sdk_version': int(self.robot.SDK_version()),
                         'libKine_sha256': digest(self.sdk_root / 'SDK_PYTHON/libKine.so'),
                         'libMarvinSDK_sha256': digest(self.sdk_root / 'SDK_PYTHON/libMarvinSDK.so')}

    def fk(self, joints_deg):
        joints = np.asarray(joints_deg, dtype=float)
        if joints.shape != (7,) or not np.all(np.isfinite(joints)):
            raise ValueError('Expected seven finite joint angles in degrees')
        T = np.asarray(self.kine.fk(joints.tolist()), dtype=float)
        T = validate_T(T).copy()
        T[:3, 3] *= .001  # SDK millimetres -> metres; do not scale R.
        return T

    def get_int(self, name):
        buf = ctypes.create_string_buffer(name.encode('ascii'), 30)
        value = ctypes.c_long()
        status = self.robot.robot.OnGetIntPara(buf, ctypes.byref(value))
        if status != 0:
            raise RuntimeError(f'Cannot read {name}: SDK status {status}')
        return int(value.value)

    def connect(self, ip):
        address = ipaddress.IPv4Address(ip)
        if address.is_unspecified or address.is_loopback or address.is_multicast or str(address) == '255.255.255.255':
            raise ValueError('Use the actual unicast robot controller address')
        self.connect_attempted = True
        try:
            if not self.robot.connect(ip):
                raise RuntimeError('Cannot connect. Check exclusive UDP port 4730 and robot address.')
            self.connected = True
            version = self.get_int('VERSION')
            if version < 100343:
                raise RuntimeError(f'Controller firmware {version} is incompatible with 100343 SDK')
            self.metadata['controller_firmware'] = version
            actual_type = self.get_int(f'R.A{self.index}.BASIC.Type')
            if actual_type != self.metadata['robot_type']:
                raise RuntimeError(f'Robot type {actual_type} != selected config type {self.metadata["robot_type"]}')
            self.metadata['robot_ip'] = ip
            deadline = time.monotonic() + 3
            serials = set()
            while time.monotonic() < deadline:
                snapshot = self.read()
                if snapshot is not None:
                    serials.add(snapshot['frame_serial'])
                if len(serials) >= 3:
                    return
                time.sleep(.01)
            raise RuntimeError('No advancing robot feedback frames')
        except BaseException:
            self.close()
            raise

    def read(self):
        if not self.connected or not self.robot.robot.OnGetBuf(ctypes.byref(self.dcss)):
            return None
        out = self.dcss.m_Out[self.index]
        serial = int(out.m_OutFrameSerial)
        if serial == 0 or (self.history and serial == self.history[-1]['frame_serial']):
            return None
        state = self.dcss.m_State[self.index]
        s = {'host_monotonic': time.monotonic(), 'frame_serial': serial,
             'joints_deg': list(out.m_FB_Joint_Pos), 'velocities_deg_s': list(out.m_FB_Joint_Vel),
             'state': int(state.m_CurState), 'error_code': int(state.m_ERRCode)}
        self.history.append(s)
        return s

    def stationary_pose(self, image_host_time, duration=.5):
        self.read()
        now = time.monotonic()
        rows = [r for r in self.history if r['host_monotonic'] >= image_host_time - duration]
        if len(rows) < 8 or rows[0]['host_monotonic'] > image_host_time - duration + .1:
            raise ValueError('Need at least 0.5 s of stationary feedback; wait and save again')
        stamps = np.array([r['host_monotonic'] for r in rows])
        if now - stamps[-1] > .1 or now - image_host_time > .3 or np.max(np.diff(stamps)) > .15:
            raise ValueError('Stale/gapped robot or camera data; sample rejected')
        if stamps[-1] < image_host_time:
            raise ValueError('Need fresh feedback after the image arrival')
        if any(r['state'] not in (0, 1, 2, 3, 4) or r['error_code'] != 0 for r in rows):
            raise ValueError('Robot fault or transitioning state; sample rejected')
        joints = np.array([r['joints_deg'] for r in rows])
        vel = np.array([r['velocities_deg_s'] for r in rows])
        if not np.all(np.isfinite(joints)) or not np.all(np.isfinite(vel)):
            raise ValueError('Nonfinite robot feedback')
        if np.max(np.abs(vel)) > .15 or np.max(np.ptp(joints, axis=0)) > .06:
            raise ValueError('Robot moving; wait until stationary')
        poses = [self.fk(q) for q in joints]
        ref = poses[np.argmin(np.abs(stamps - image_host_time))]
        differences = [delta(ref, T) for T in poses]
        if max(t for t, r in differences) > .0005 or max(r for t, r in differences) > .1:
            raise ValueError('Flange pose not stable within 0.5 mm / 0.1 deg')
        nearest = rows[int(np.argmin(np.abs(stamps - image_host_time)))]
        return ref, {'nearest_feedback': nearest, 'feedback_window': rows,
                     'sync_method': 'stationary capture, host arrival bracketing; no hardware sync'}

    def close(self):
        if self.connect_attempted:
            self.robot.release_robot()
            self.connect_attempted = self.connected = False
