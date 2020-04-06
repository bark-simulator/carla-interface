import sys
import glob
import math
import numpy as np
import logging

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla


class CarlaClient():
  """Class for connecting server and managing carla's world"""

  def __init__(self, map='Town02'):
    self.client = None
    self.world = None
    self.bp_lib = None
    self.map = map
    self.active_actors = dict()

  # def __del__(self):
  #   if self.active_actors:
  #     for actor in self.active_actors.values():
  #       actor.destroy()
  #   self.set_synchronous_mode(False)

  def connect(self, host='localhost', port=2000, timeout=2):
    self.client = carla.Client(host, port)
    self.client.set_timeout(timeout)
    self.client.load_world(self.map)

    self.world = self.client.get_world()
    self.bp_lib = self.world.get_blueprint_library()
    logging.info("Connected to Carla Server")

  def set_synchronous_mode(self, mode, delta_seconds=0.05):
    self.world.apply_settings(carla.WorldSettings(
        synchronous_mode=mode,
        fixed_delta_seconds=delta_seconds))

  def tick(self):
    return self.world.tick()

  def get_world(self):
    return self.world

  def get_blueprint_library(self):
    return self.bp_lib

  def get_spawn_points(self):
    return self.world.get_map().get_spawn_points()

  def spawn_actor(self, blueprint, transform):
      actor = self.world.try_spawn_actor(blueprint, transform)

      if actor is not None:
        self.active_actors[actor.id] = actor
        return actor.id
      else:
        return None

  def spawn_sensor(self, attach_to_id: int, sensor_type, location, rotation):
      sensor_bp = self.bp_lib.find(sensor_type)
      sensor_transform = carla.Transform(carla.Location(*location),carla.Rotation(*rotation))

      sensor = self.world.try_spawn_actor(sensor_bp, sensor_transform, attach_to=self.active_actors[attach_to_id])
      
      if sensor is not None:
        self.active_actors[sensor.id] = sensor
        return sensor.id
      else:
        return None

  def get_actors(self, actor_id):
    return self.active_actors[actor_id]

  def set_autopilot(self, actor_id, mode=True):
    self.active_actors[actor_id].set_autopilot(mode)

  """
      [TIME_POSITION, X_POSITION, Y_POSITION, THETA_POSITION, VEL_POSITION, ...]

      Notes:
      The y-axis location and orientation angle returned from Carla need to be 
      mirror in order to display the vehicles correctly on the map
      -t.rotation.yaw & -t.location.y

  """

  def get_vehicle_state(self, id):
    if id in self.active_actors:
      snapshot = self.world.get_snapshot()
      vehicle = self.active_actors[id]

      t = vehicle.get_transform()
      v = vehicle.get_velocity()
      # c = vehicle.get_control()
      # a = vehicle.get_acceleration()
      # av = vehicle.get_angular_velocity()

      return np.array([snapshot.timestamp.elapsed_seconds, t.location.x, -t.location.y,
                       math.radians(-t.rotation.yaw),math.sqrt(v.x**2 + v.y**2 + v.z**2)])
    else:
      logging.error("Actor {} not found".format(id))
      return None

  def get_vehicles_state(self, carla_2_bark_idx):
    # carla_2_bark_idx: convert carla actor id to bark agent id
    # Should be called in synchronous mode

    actor_state_map = dict()
    for id, vehicle in self.active_actors.items():
      if vehicle.type_id.split(".")[0] == "vehicle":
        actor_state_map[carla_2_bark_idx[id]] = self.get_vehicle_state(id)

    return actor_state_map
