import sys
import glob
import math
import numpy as np

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
    # TODO: set/dict/list?
    self.actors = dict()

  def __del__(self):
    if self.actors:
      for id, actor in self.actors.items():
        actor.destroy()
    self.set_synchronous_mode(False)

  def connect(self, host='localhost', port=2000, timeout=2):
    self.client = carla.Client(host, port)
    self.client.set_timeout(timeout)  # in second
    self.client.load_world(self.map)
    self.world = self.client.get_world()
    self.bp_lib = self.world.get_blueprint_library()
    print("Connected to Carla Server")

  def set_synchronous_mode(self, mode, delta_seconds=0.05):
    self.world.apply_settings(carla.WorldSettings(
        synchronous_mode=mode,
        fixed_delta_seconds=delta_seconds))

  def tick(self):
    self.world.tick()

  def get_world(self):
    return self.world

  def get_blueprint_library(self):
    return self.bp_lib

  def get_spawn_points(self):
    return self.world.get_map().get_spawn_points()

  def spawn_actor(self, blueprint, transform):
    try:
      actor = self.world.spawn_actor(blueprint, transform)
      self.actors[actor.id] = actor
      return actor.id, actor
    except Exception as e:
      print(e)
      return None, None

  def spawn_sensor(self, attach_to_id: int, sensor_type='sensor.camera.rgb', relative_location=(1.5, 0.0, 2.4)):
    try:
      sensor_bp = self.bp_lib.find(sensor_type)
      sensor_transform = carla.Transform(carla.Location(*relative_location))
      sensor = self.world.spawn_actor(sensor_bp, sensor_transform, attach_to=self.actors[attach_to_id])
      self.actors[sensor.id] = sensor
      return sensor.id, sensor
    except Exception as e:
      print(e)
      return None, None

  def get_actors_id(self):
    return self.actors.keys()

  def set_autopilot(self, id, mode):
    self.actors[id].set_autopilot(mode)

  """
  
      [TIME_POSITION, X_POSITION, Y_POSITION, THETA_POSITION, VEL_POSITION, ...]

      Notes:
      The y-axis location and orientation angle returned from Carla need to be 
      mirror in order to display the vehicles correctly on the map
      -t.rotation.yaw & -t.location.y

      TODO:
      Look for potential bug in Carla/BARK's source code
  
  """

  def get_vehicle_state(self, id):
    if id in self.actors:
      snapshot = self.world.get_snapshot()
      # vehicle = snapshot.find(id)
      vehicle = self.actors[id]

      t = vehicle.get_transform()
      v = vehicle.get_velocity()
      # c = vehicle.get_control()
      # a = vehicle.get_acceleration()
      # av = vehicle.get_angular_velocity()

      return np.array([snapshot.timestamp.elapsed_seconds, t.location.x, -t.location.y,
                       math.radians(-t.rotation.yaw),
                       math.sqrt(v.x**2 + v.y**2 + v.z**2)])
    else:
      print("Actor {} not found".format(id))
      return None

  def get_vehicles_state(self, id_convertion):
    # id_convertion: convert carla actor id to bark agent id
    # Should be called in synchronous mode
    actor_state_map = dict()
    snapshot = self.world.get_snapshot()
    for id, vehicle in self.actors.items():
      if vehicle.type_id.split(".")[0] == "vehicle":
        actor_state_map[id_convertion[id]] = self.get_vehicle_state(id)

    return actor_state_map
