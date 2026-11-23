"""The ZED X Mini through the ZED SDK (pyzed) for calibration: left rectified image, depth along the optical axis in metres, factory intrinsics.

The camera interface the runner needs (`app.run`), so a test or another camera can stand in:
    .K (3x3 float), .size (width, height), .grab(settle_frames=8) -> (header dict, {"rgb": uint8 HxWx3, "depth": float32 HxW metres, 0 = missing}), .hello() -> dict, .close()
pyzed is imported when the camera is opened, not when this module is.
"""
import time

import numpy as np


class CameraError(RuntimeError):
    pass


class ZedCamera:
    def __init__(self, resolution="HD1200", fps=15, depth_mode="ULTRA", depth_min_m=0.15, depth_max_m=2.5):
        try:
            import pyzed.sl as sl
        except ImportError as error:
            raise CameraError(f"pyzed is not importable: {error}") from None
        self.sl, self.depth_min_m, self.depth_max_m = sl, depth_min_m, depth_max_m
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, resolution)
        init.camera_fps = fps
        init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        init.coordinate_units = sl.UNIT.METER
        init.depth_minimum_distance = depth_min_m
        init.depth_maximum_distance = depth_max_m
        self.camera = sl.Camera()
        status = self.camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise CameraError(f"open failed: {status}")
        self.runtime = sl.RuntimeParameters()
        info = self.camera.get_camera_information()
        left = info.camera_configuration.calibration_parameters.left_cam
        self.K = np.array([[left.fx, 0, left.cx], [0, left.fy, left.cy], [0, 0, 1]], dtype=float)
        self.size = (int(left.image_size.width), int(left.image_size.height))
        self.meta = {"serial": int(info.serial_number), "sdk": sl.Camera.get_sdk_version(), "model": str(info.camera_model),
                     "resolution": list(self.size), "fps": fps, "depth_mode": depth_mode, "K": self.K.tolist(),
                     "distortion": [float(v) for v in left.disto]}
        self._image, self._depth = sl.Mat(), sl.Mat()

    def hello(self):
        return dict(self.meta)

    def grab(self, settle_frames=8):
        """Never moves anything; the caller checks the arm was still."""
        sl, good = self.sl, 0
        for _ in range(int(settle_frames) + 30):
            if self.camera.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                good += 1
                if good > int(settle_frames):
                    break
        else:
            raise CameraError("no frames from the camera")
        self.camera.retrieve_image(self._image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(self._depth, sl.MEASURE.DEPTH)
        rgb = np.ascontiguousarray(self._image.get_data()[:, :, :3][:, :, ::-1])         # BGRA -> RGB
        depth = np.nan_to_num(self._depth.get_data().astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        depth[(depth < self.depth_min_m) | (depth > self.depth_max_m)] = 0.0
        return {"K": self.K.tolist(), "width": self.size[0], "height": self.size[1], "t": time.monotonic()}, {"rgb": rgb, "depth": depth}

    def close(self):
        self.camera.close()
