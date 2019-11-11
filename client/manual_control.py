import pygame
from pygame.locals import K_UP
from pygame.locals import K_DOWN
from pygame.locals import K_q
from pygame.locals import K_c
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_s
from pygame.locals import K_w
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_SPACE

import queue
import numpy as np
import sys
import glob

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla
from carla import ColorConverter as cc


class KeyboardControl:
  def __init__(self, world):
    self._control = carla.VehicleControl()
    self._steer_cache = 0.0
    self.world = world
    # create and move to World class later
    # self.weather = WeatherControl(self.world)

  def control(self, vehicle, clock):
    for event in pygame.event.get():
      if event.type == pygame.KEYUP:
        if event.key == K_q:
          self._control.gear = 1 if self._control.reverse else -1
        # elif event.key == K_c:
        #   self.weather.change()

    self.parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
    self._control.reverse = self._control.gear < 0
    vehicle.apply_control(self._control)

  def parse_vehicle_keys(self, keys, milliseconds):
    self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
    steer_increment = 5e-4 * milliseconds
    if keys[K_LEFT] or keys[K_a]:
      self._steer_cache -= steer_increment
    elif keys[K_RIGHT] or keys[K_d]:
      self._steer_cache += steer_increment
    else:
      self._steer_cache = 0.0
    self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
    self._control.steer = round(self._steer_cache, 1)
    self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
    self._control.hand_brake = keys[K_SPACE]


class SensorsData:
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
