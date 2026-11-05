#!/usr/bin/env python3
"""Eye-to-hand calibration: TJ MARVIN + fixed RealSense + center-obscured board."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
from board import RingBoard, draw, estimate_pose, project, ANCHORS
from calibration import solve, delta
from robot_adapter import Tianji, DEFAULT_CONFIG, DEFAULT_SDK


def save_json(path, data):
    path = Path(path)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temp.replace(path)


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(f'Could not write image: {path}')


def pick_anchors(image):
    """Select four real inner intersections, NOT the plate outline."""
    scale = min(1., 1280 / image.shape[1], 800 / image.shape[0])
    shown = cv2.resize(image, None, fx=scale, fy=scale)
    clicks = []
    name = 'Select inner corners 1,2,3,4 | Enter accept | Esc cancel | Backspace undo'
    cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
    def callback(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x / scale, y / scale))
    cv2.setMouseCallback(name, callback)
    try:
        while True:
            preview = shown.copy()
            for i, (x, y) in enumerate(clicks):
                pt = (int(x * scale), int(y * scale))
                cv2.circle(preview, pt, 6, (0, 150, 255), 2)
                cv2.putText(preview, str(i + 1), pt, cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 150, 255), 2)
            cv2.imshow(name, preview)
            key = cv2.waitKey(30) & 255
            if key in (10, 13) and len(clicks) == 4:
                return np.float32(clicks)
            if key == 27 or cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
                return None
            if key in (8, 127) and clicks:
                clicks.pop()
    finally:
        cv2.destroyWindow(name)


def sdk(args):
    return Tianji(args.arm, args.sdk_root, args.model_config)


def detect(args):
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError(f'Cannot read {args.image}')
    scale = min(1., 1280 / max(image.shape[:2]))
    image = cv2.resize(image, None, fx=scale, fy=scale)
    board = RingBoard()
    anchors = pick_anchors(image) if args.manual else None
    if args.manual and anchors is None:
        return 1
    ob = board.initialize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), anchors)
    if ob is None:
        raise ValueError('Automatic detection failed. Try --manual and select the four extreme inner intersections.')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    save_image(args.out, draw(image, ob))
    print(json.dumps({'detected_corners': len(ob.ids), 'grid_rms_px': ob.grid_rms_px,
                      'median_cell_px': ob.cell_px, 'image_scale': scale,
                      'anchors_in_preview_pixels': project(ob.H, ANCHORS).tolist(),
                      'preview': str(args.out.resolve()),
                      'note': 'Detection diagnostic; grid RMS is not calibration accuracy.'}, indent=2))
    return 0


def inspect_sdk(args):
    robot = sdk(args)
    print(json.dumps(robot.metadata, ensure_ascii=False, indent=2))
    T = robot.fk([21.8, -41., -4.74, -63.67, 10.15, 14.72, 7.68])
    print('Offline FK at example angles; NO connection or robot motion. T_base_flange in metres:')
    print(T)
    return 0


def capture(args):
    from camera import RealSenseColor
    if args.session.exists():
        raise ValueError('Session directory already exists. Use a new directory to preserve IDs and avoid mixed data.')
    robot, camera = sdk(args), None
    samples = []
    try:
        camera = RealSenseColor(args.serial, args.width, args.height, args.fps)
        robot.connect(args.ip)
        args.session.mkdir(parents=True, exist_ok=False)
        meta = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                'robot': robot.metadata, 'camera': camera.metadata,
                'board': {'squares': [10, 10], 'square_m': .005, 'center_ignored': True},
                'transform_convention': 'T_A_B maps B into A; metres',
                'motion_mode': 'Externally prepared manual positioning; this program issues no motion commands'}
        save_json(args.session / 'metadata.json', meta)
        save_json(args.session / 'samples.json', samples)
        board = RingBoard()
        next_auto = 0.
        status = 'Show whole board. S save | I select same four anchors | Q quit'
        window = 'TJ eye-to-hand | S save | I manual anchors | Q quit'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, args.width, args.height)
        while True:
            robot.read()
            image, frame = camera.read()
            robot.read()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ob = None
            if not board.started and time.monotonic() >= next_auto:
                ob = board.initialize(gray)
                next_auto = time.monotonic() + 1
                if ob is not None:
                    save_image(args.session / 'initial_reference.png', draw(image, ob))
                    status = 'Tracking initialized. Hold still, press S to save.'
            elif board.started:
                ob = board.update(gray)
            if board.lost:
                status = 'TRACK LOST: I to select SAME physical corners as initial_reference.png'
            pnp = None
            if ob is not None:
                try:
                    T, reproj = estimate_pose(ob, camera.K)
                    pnp = T, reproj
                except ValueError as e:
                    status = str(e)
            preview = draw(image, ob)
            info = f'samples={len(samples)} corners={0 if ob is None else len(ob.ids)}'
            if pnp:
                info += f' reproj={pnp[1]:.3f}px'
            cv2.putText(preview, info, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .65, (40, 230, 40), 2)
            cv2.putText(preview, status[:125], (12, 54), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 180, 255), 1)
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 255
            if key in (ord('q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord('i'):
                anchors = pick_anchors(image)
                if anchors is not None:
                    recovered = board.initialize(gray, anchors)
                    if recovered is None:
                        # Never retain a valid tracking state after failed manual relabeling.
                        board.lost = True
                        status = 'Manual initialization failed; select four extreme INNER intersections.'
                    else:
                        status = 'Anchor IDs set. Hold still before sampling.'
                        if not (args.session / 'initial_reference.png').exists():
                            save_image(args.session / 'initial_reference.png', draw(image, recovered))
                        # Paused frames do not satisfy a fresh stability window.
                        robot.history.clear()
            if key == ord('s'):
                try:
                    if ob is None or pnp is None:
                        raise ValueError('No valid board pose; sample rejected')
                    A, feedback = robot.stationary_pose(frame['host_monotonic'])
                    if any((lambda d: d[0] < .01 and d[1] < 3)(delta(np.array(s['T_base_flange']), A)) for s in samples):
                        raise ValueError('Pose too close to an existing sample (<10 mm and <3 deg)')
                    index = len(samples)
                    name = f'color_{index:04d}.png'
                    record = {'index': index, 'image': name, 'frame': frame, 'robot_feedback': feedback,
                              'T_base_flange': A.tolist(), 'T_camera_board': pnp[0].tolist(),
                              'corner_ids': ob.ids.tolist(), 'image_points': ob.pixels.tolist(),
                              'object_points_m': ob.object_points().tolist(), 'reprojection_rms_px': pnp[1]}
                    save_image(args.session / name, image)
                    samples.append(record)
                    save_json(args.session / 'samples.json', samples)
                    status = f'Saved sample {len(samples)}; move slowly, then hold still.'
                    print(status, flush=True)
                except ValueError as e:
                    status = str(e)
                    print('Not saved:', status, flush=True)
        print(f'Saved {len(samples)} samples in {args.session.resolve()}')
        return 0
    finally:
        try:
            if camera is not None:
                camera.close()
        finally:
            try:
                robot.close()
            finally:
                cv2.destroyAllWindows()


def solve_session(args):
    samples = json.loads((args.session / 'samples.json').read_text())
    meta = json.loads((args.session / 'metadata.json').read_text())
    result = solve(samples, args.max_mm, args.max_deg)
    result['base_frame'] = meta['robot']['base_frame']
    result['session_metadata'] = meta
    target = args.out or args.session / 'calibration_result.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    save_json(target, result)
    print('T_base_camera (metres):\n', np.array(result['T_base_camera']))
    print('Held-out validation:', json.dumps(result['validation'], ensure_ascii=False, indent=2))
    print('QUALITY:', 'PASS' if result['quality_passed'] else 'FAIL; inspect per-sample errors before use')
    print('Result:', target.resolve())
    return 0 if result['quality_passed'] else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('detect', help='Test the provided board image without hardware')
    p.add_argument('--image', type=Path, required=True)
    p.add_argument('--out', type=Path, default=Path('detection_preview.jpg'))
    p.add_argument('--manual', action='store_true')
    p.set_defaults(run=detect)
    for name in ('inspect-sdk', 'capture'):
        p = commands.add_parser(name)
        p.add_argument('--sdk-root', type=Path, default=DEFAULT_SDK)
        p.add_argument('--model-config', type=Path, default=DEFAULT_CONFIG)
        p.add_argument('--arm', choices=('A', 'B'), required=True)
        if name == 'inspect-sdk':
            p.set_defaults(run=inspect_sdk)
        else:
            p.add_argument('--ip', required=True, help='Robot controller IPv4 address')
            p.add_argument('--serial', help='RealSense serial; required if more than one is connected')
            p.add_argument('--width', type=int, default=1280)
            p.add_argument('--height', type=int, default=720)
            p.add_argument('--fps', type=int, default=30)
            p.add_argument('--session', type=Path, required=True, help='New session directory')
            p.set_defaults(run=capture)
    p = commands.add_parser('solve')
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--out', type=Path)
    p.add_argument('--max-mm', type=float, default=5.)
    p.add_argument('--max-deg', type=float, default=2.)
    p.set_defaults(run=solve_session)
    args = parser.parse_args(argv)
    try:
        return args.run(args)
    except KeyboardInterrupt:
        print('Interrupted; completed samples remain saved.', file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, cv2.error) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
