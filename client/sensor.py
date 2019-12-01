import queue
import numpy as np
import sys
import glob
import pygame

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla
from carla import ColorConverter as cc

class SensorData:
  def __init__(self):
    # displaying camera's image
    self.cam_surface = None
    self.image_queue = queue.Queue()

  def RGBcamToImage_sync(self, world_frame_id):
    while True:
      image = self.image_queue.get(timeout=2.0)
      if image.frame == world_frame_id:
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.cam_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        return

  def RGBcamToImage(self, image=None):
    image.convert(cc.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    self.cam_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))