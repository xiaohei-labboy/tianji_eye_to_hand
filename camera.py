"""RealSense RGB frames rectified into the same color optical coordinate system."""
import time
import cv2
import numpy as np


class RealSenseColor:
    def __init__(self, serial=None, width=1280, height=720, fps=30):
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise RuntimeError('pyrealsense2 is missing; install requirements.txt in a virtual environment') from e
        self.rs, self.pipe = rs, rs.pipeline()
        self.started = False
        self.last_number = None
        config = rs.config()
        if serial:
            config.enable_device(serial)
        else:
            devices = rs.context().query_devices()
            if len(devices) != 1:
                raise RuntimeError(f'Found {len(devices)} RealSense devices; connect one or specify --serial')
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            profile = self.pipe.start(config)
            self.started = True
            vp = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = vp.get_intrinsics()
            self.K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], float)
            self.maps = None
            size = (intr.width, intr.height)
            if intr.model == rs.distortion.brown_conrady:
                self.maps = cv2.initUndistortRectifyMap(self.K, np.array(intr.coeffs), None,
                                                      self.K, size, cv2.CV_32FC1)
            elif intr.model == rs.distortion.inverse_brown_conrady:
                raise RuntimeError('Inverse Brown-Conrady color stream is not supported by this rectifier')
            elif intr.model != rs.distortion.none:
                # Use the SDK forward projection for other models, e.g. modified
                # Brown-Conrady; their coefficients must NOT be passed to OpenCV.
                mx, my = np.empty(size[::-1], np.float32), np.empty(size[::-1], np.float32)
                for y in range(intr.height):
                    for x in range(intr.width):
                        ray = [(x - intr.ppx) / intr.fx, (y - intr.ppy) / intr.fy, 1.0]
                        mx[y, x], my[y, x] = rs.rs2_project_point_to_pixel(intr, ray)
                self.maps = mx, my
            dev = profile.get_device()
            self.metadata = {'device_name': dev.get_info(rs.camera_info.name),
                             'serial': dev.get_info(rs.camera_info.serial_number),
                             'firmware': dev.get_info(rs.camera_info.firmware_version),
                             'width': intr.width, 'height': intr.height, 'fps': fps,
                             'raw_distortion_model': str(intr.model), 'raw_distortion_coeffs': list(intr.coeffs),
                             'K': self.K.tolist(), 'saved_images': 'rectified color',
                             'saved_distortion_coeffs': [0.] * 5,
                             'frame': 'color optical (x right, y down, z forward)'}
            for sensor in dev.query_sensors():
                if sensor.supports(rs.option.frames_queue_size):
                    sensor.set_option(rs.option.frames_queue_size, 1)
            for _ in range(30):
                self.pipe.wait_for_frames(5000)
        except BaseException:
            self.close()
            raise

    def read(self):
        frames = self.pipe.wait_for_frames(3000)
        frame = frames.get_color_frame()
        if not frame:
            raise RuntimeError('No RGB frame')
        arrival = time.monotonic()
        number = int(frame.get_frame_number())
        if number == self.last_number:
            raise RuntimeError('Repeated RGB frame')
        self.last_number = number
        image = np.asanyarray(frame.get_data()).copy()
        if self.maps is not None:
            image = cv2.remap(image, self.maps[0], self.maps[1], cv2.INTER_LINEAR)
        return image, {'host_monotonic': arrival, 'frame_number': number,
                       'device_timestamp_ms': float(frame.get_timestamp()),
                       'timestamp_domain': str(frame.get_frame_timestamp_domain())}

    def close(self):
        if self.started:
            self.pipe.stop()
            self.started = False
