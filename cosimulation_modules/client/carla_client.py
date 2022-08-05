import sys
import glob
import math
import numpy as np
import logging
import random
import time

try:
    sys.path.append(
        glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
    pass
import carla


class CarlaClient():
    """Class for connecting server and managing carla's world"""

    def __init__(self):
        self.client = None
        self.world = None
        self.bp_lib = None
        self.active_actors = dict()

    def __del__(self):
        print('destroying actors')
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.active_actors.values()])
        self.active_actors.clear()
        # if self.active_actors:
        #     for actor in self.active_actors.values():
        #         if self.world.get_actor(actor.id) is not None:
        #             actor.destroy()
        if self.world is not None:
            self.set_synchronous_mode(False)

    def connect(self, carla_map='Town02', host='localhost',
                port=2000, timeout=2, num_retries=10):
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)

        for _ in range(num_retries):
            self.client.load_world(carla_map)
            self.world = self.client.get_world()

            if self.world is not None:
                break
            else:
                time.sleep(1)

        if self.world is None:
            raise Exception("Cannot connect to retrieve Carla's world")

        self.bp_lib = self.world.get_blueprint_library()

        self.traffic_manager = self.client.get_trafficmanager()

        logging.info("Connected to Carla Server")

    def set_synchronous_mode(self, mode, delta_seconds=0.05):
        self.world.apply_settings(carla.WorldSettings(
            synchronous_mode=mode,
            fixed_delta_seconds=delta_seconds))
        self.traffic_manager.set_synchronous_mode(mode)

    def tick(self):
        return self.world.tick()
    def get_current_time(self):
        curr_snapshot =  self.world.get_snapshot()
        return curr_snapshot.timestamp.elapsed_seconds
    def get_world(self):
        return self.world

    def get_blueprint_library(self):
        return self.bp_lib

    def get_spawn_points(self):
        ava_spawn_pts = self.world.get_map().get_spawn_points()
        if not ava_spawn_pts:
            print("Spawn Point List is empty! Generating way points")
            waypoint_list = self.world.get_map().generate_waypoints(10.0)
            # ava_spawn_pts = [wp.transform for wp in waypoint_list]
            for wp in waypoint_list:
                tmp_tf = wp.transform
                tmp_tf.location.z += 0.3
                ava_spawn_pts.append(tmp_tf)
        return ava_spawn_pts

    def spawn_random_vehicle(self, num_retries=10, transform=None):
        vehicle_bp = self.bp_lib.filter('vehicle')
        blueprint = random.choice(vehicle_bp)

        id = None
        for _ in range(num_retries):
            if transform is None:
                transform = random.choice(self.get_spawn_points())
            id = self.spawn_actor(blueprint, transform)
            if id is not None:
                return id

    def spawn_actor(self, blueprint, transform):
        actor = self.world.try_spawn_actor(blueprint, transform)

        if actor is not None:
            self.active_actors[actor.id] = actor
            return actor.id
        else:
            return None

    def spawn_sensor(self, attach_to_id: int, sensor_type, location, rotation):
        sensor_bp = self.bp_lib.find(sensor_type)
        sensor_transform = carla.Transform(
            carla.Location(
                *location),
            carla.Rotation(
                *rotation))

        sensor = self.world.try_spawn_actor(
            sensor_bp,
            sensor_transform,
            attach_to=self.active_actors[attach_to_id])

        if sensor is not None:
            self.active_actors[sensor.id] = sensor
            return sensor.id
        else:
            return None

    def get_actor(self, actor_id):
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
            vehicle = self.active_actors[id]
            t = vehicle.get_transform()
            v = vehicle.get_velocity()
            # c = vehicle.get_control()
            # a = vehicle.get_acceleration()
            # av = vehicle.get_angular_velocity()
            snapshot = self.world.get_snapshot()
            return np.array([snapshot.timestamp.elapsed_seconds, t.location.x, -t.location.y,
                                math.radians(-t.rotation.yaw), math.sqrt(v.x**2 + v.y**2 + v.z**2)])
        else:
            logging.error("Actor {} not found".format(id))
            return None

    def get_vehicles_state(self, carla_2_bark_idx):
        # carla_2_bark_idx: convert carla actor id to bark agent id
        # Should be called in synchronous mode

        actor_state_map = dict()
        for id in self.active_actors.copy():
            if self.active_actors[id].is_alive:
                if self.active_actors[id].type_id.split(".")[0] == "vehicle":
                    actor_state_map[carla_2_bark_idx[id]
                                    ] = self.get_vehicle_state(id)
            else:
                del self.active_actors[id]
                logging.error("Actor {} is already destroyed!".format(id))
            

        return actor_state_map

    @staticmethod
    def generate_tranformation(x=0, y=0, z=0, pitch=0, yaw=0, roll=0):
        return carla.Transform(carla.Location(float(x), -float(y), float(z)),
                               carla.Rotation(float(pitch), -float(yaw), float(roll)))
