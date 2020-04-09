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

import sys
import glob

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla


class KeyboardControl:
  def __init__(self, world):
    self.control = carla.VehicleControl()
    self.steer_cache = 0.0
    # TODO: create and move to World class later
    # self.world = world
    # self.weather = WeatherControl(self.world)

  def control(self, vehicle, clock):
    for event in pygame.event.get():
      if event.type == pygame.KEYUP:
        if event.key == K_q:
          self.control.gear = 1 if self.control.reverse else -1
        # elif event.key == K_c:
        #   self.weather.change()

    self.parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
    self.control.reverse = self.control.gear < 0
    vehicle.applycontrol(self.control)

  def parse_vehicle_keys(self, keys, milliseconds):
    self.control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
    steer_increment = 5e-4 * milliseconds
    if keys[K_LEFT] or keys[K_a]:
      self.steer_cache -= steer_increment
    elif keys[K_RIGHT] or keys[K_d]:
      self.steer_cache += steer_increment
    else:
      self.steer_cache = 0.0
    self.steer_cache = min(0.7, max(-0.7, self.steer_cache))
    self.control.steer = round(self.steer_cache, 1)
    self.control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
    self.control.hand_brake = keys[K_SPACE]
