from queue import Queue
import numpy as np
import sys
import glob
import pygame as pg

try:
    sys.path.append(
        glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
    pass
import carla
# from carla import ColorConverter as cc


class CameraManager:
    def __init__(self, cameras, synchronous_mode=False):
        self.synchronous_mode = synchronous_mode
        self.cams = cameras
        self.surfaces = dict.fromkeys(self.cams.keys())
        self.queues = []

        for c in self.cams.values():
            self._make_queue(self.queues, c.listen)

    def _make_queue(self, queues, register_event):
        q = Queue()
        register_event(q.put)
        queues.append(q)

    def _retrieve_data(self, sensor_queue, frame_id, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == frame_id:
                return data

    def tick(self, frame_id, timeout=2):
        data = [self._retrieve_data(q, frame_id, timeout) for q in self.queues]
        return data

    def _process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        return array

    def fetch_image(self, frame_id, agents_id=None):
        rawImages = self.tick(frame_id)
        idx = self.cams.keys() if agents_id is None else agents_id

        for idx, rawImage in zip(idx, rawImages):
            image = self._process_image(rawImage)
            self.surfaces[idx] = pg.surfarray.make_surface(
                image.swapaxes(0, 1))
