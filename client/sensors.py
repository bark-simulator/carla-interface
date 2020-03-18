from queue import Queue
import numpy as np
import sys
import glob
import pygame
# from collections import defaultdict

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla
from carla import ColorConverter as cc

class CameraManager:
  def __init__(self,carla_cameras,synchronous_mode=False):
    self.synchronous_mode=synchronous_mode
    self.cameras=carla_cameras

    self.ids=self.cameras.keys()

    self.surfaces = dict.fromkeys(self.ids)
    # self.image_queues={k:Queue() for k in self.ids}
    self._queues = []

    def make_queue(register_event):
        q = Queue()
        register_event(q.put)
        self._queues.append(q)

    for c in self.cameras.values():
        make_queue(c.listen)
  
  def tick(self,frame_id, timeout=2):
    data = [self._retrieve_data(q, frame_id,timeout) for q in self._queues]
    return data
  
  def _retrieve_data(self, sensor_queue,frame_id, timeout):
    while True:
        data = sensor_queue.get(timeout=timeout)
        if data.frame == frame_id:
            return data

  def processImage(self,image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array

  def fetchImage(self,frame_id,agents_id=None):
    rawImages=self.tick(frame_id)
    ids=self.ids if agents_id is None else agents_id
    for id,rawImage in zip(ids,rawImages):
      image=self.processImage(rawImage)
      self.surfaces[id] = pygame.surfarray.make_surface(image.swapaxes(0, 1))

  def RGBcamToImage(self, image,cam_id):
  #   if self.synchronous_mode:
  #       for id in self.ids:
  #         image = self.image_queues[id].get(timeout=2.0)
  #         print(image)
  #         if image.frame == world_frame_id:
  #           array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
  #           array = np.reshape(array, (image.height, image.width, 4))
  #           array = array[:, :, :3]
  #           array = array[:, :, ::-1]
  #           self.surfaces[id] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            
          
  #   elif image is not None:
    image.convert(cc.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    self.surfaces[cam_id] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
      