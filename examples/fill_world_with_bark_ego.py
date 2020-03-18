from client.carla_client import CarlaClient
from client.raw_control import RawControl
from client.sensors import CameraManager
from client.viewer import Viewer

from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.pygame_viewer import PygameViewer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser

import subprocess
import sys
import os
import glob
import time
import numpy as np
import random
import math
import pygame

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla


BARK_PATH = "external/com_github_bark_simulator_bark/"
BARK_MAP = "Town01"
CARLA_MAP = "Town01"
CARLA_PORT = 2000
DELTA_SECOND = 1
NUM_BARK_TRAJECTORY_STEP = 10
SYNCHRONOUS_MODE = True
BARK_SIM_FACTOR = 10
BARK_SCREEN_DIMS = (600, 600)
EGO_CAMERA_SIZE = (800, 600)
CARLA_LOW_QUALITY = True


CARLA_DELTA = DELTA_SECOND
# CARLA_DELTA = DELTA_SECOND/NUM_BARK_TRAJECTORY_STEP
# assert(1/10 >= CARLA_DELTA >= 1/60)


param_server = ParameterServer(filename=BARK_PATH+"examples/params/od8_const_vel_one_agent.json")

# World Definition
bark_world = World(param_server)

# Model Definitions
behavior_model = BehaviorConstantVelocity(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel(param_server)

# Map Definition
xodr_parser = XodrParser(BARK_PATH+"modules/runtime/tests/data/"+BARK_MAP+".xodr")
map_interface = MapInterface()
map_interface.set_open_drive_map(xodr_parser.map)
bark_world.set_map(map_interface)
# Agent Definition
agent_2d_shape = CarLimousine()
init_state = np.array([0, -2, -304, 3.14*-0.5, 20/3.6]) # [0, -2, -300, 3.14*-1, 20/3.6]
goal_polygon = Polygon2d([0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
goal_polygon = goal_polygon.translate(Point2d(352, -331))

# open Carla simulation server
try:
  args = ["external/carla/CarlaUE4.sh", "-quality-level=Low"]
  server = subprocess.Popen(args[0] if not CARLA_LOW_QUALITY else args)
  # Connect to Carla server
  client = CarlaClient(CARLA_MAP)
  time.sleep(12)  # Wait for carla
  client.connect(port=CARLA_PORT, timeout=5)
  client.set_synchronous_mode(SYNCHRONOUS_MODE, CARLA_DELTA)

  rc = RawControl(client.client)

  blueprint_library = client.get_blueprint_library()

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()

  agent_carla_id = None
  while agent_carla_id == None:
    # create agent (actor) in Carla
    bp = random.choice(blueprint_library.filter('vehicle.dodge_charger.police'))
    transform = carla.Transform(carla.Location(x=float(init_state[1]), y=-float(init_state[2]), z=0.3),
                                carla.Rotation(yaw=math.degrees(init_state[3])))
    agent_carla_id, agent_carla = client.spawn_actor(bp, transform)

    if agent_carla_id != None:
      cam_carla_id, cam = client.spawn_sensor(agent_carla_id, "sensor.camera.rgb",
                                              location=(0, 0, 30), rotation=(270, 0, 0))

    # create agent object in BARK
    agent_params = param_server.addChild("agent{}".format(1))
    bark_agent = Agent(init_state,
                       behavior_model,
                       dynamic_model,
                       execution_model,
                       agent_2d_shape,
                       agent_params,
                       GoalDefinitionPolygon(goal_polygon),  # goal_lane_id
                       map_interface)
    bark_world.add_agent(bark_agent)

    carla_to_bark_id[agent_carla_id] = bark_agent.id

  # bark viewer
  bark_viewer = PygameViewer(params=param_server,
                             follow_agent_id=bark_agent.id,
                             x_range=[-50, 50],
                             y_range=[-50, 50],
                             screen_dims=BARK_SCREEN_DIMS)

  # 800*600 is the default size of carla's camera
  interface_viewer = Viewer(EGO_CAMERA_SIZE, BARK_SCREEN_DIMS)
  sensors = CameraManager()

  if SYNCHRONOUS_MODE:
    cam.listen(sensors.image_queue.put)
  else:
    cam.listen(lambda image: sensors.RGBcamToImage(image))

  # main loop
  for i in range(10000):
    bark_viewer.clear()

    # TODO: use time different in carla if synchronous mode is off
    if i == 0:
      agent_state_map = client.get_vehicles_state(carla_to_bark_id)
      bark_world.fill_world_from_carla(0, agent_state_map)

    plan = bark_world.plan_agents(DELTA_SECOND, [bark_agent.id])[bark_agent.id]
    rc.control(agent_carla, plan[-2][1:3], plan[-1][1:3], plan[-1][4], plan[-1][3])
    bark_world.fill_world_from_carla(CARLA_DELTA, agent_state_map)
    # fr r ion range(1, NUM_BARK_TRAJECTORY_STEP):
    #   rc.control(agent_carla, plan[r-1][1:3], plan[r][1:3], plan[r][4], plan[r][3])

    interface_viewer.tick()

    if SYNCHRONOUS_MODE:
      frame_id = client.tick()
      sensors.RGBcamToImage_sync(frame_id)

    if sensors.cam_surface != None:
      interface_viewer.update(sensors.cam_surface, (0, 0) + EGO_CAMERA_SIZE)

    agent_state_map = client.get_vehicles_state(carla_to_bark_id)
    bark_world.fill_world_from_carla(CARLA_DELTA, agent_state_map)

    # if BARK_SIM_FACTOR == None or i % BARK_SIM_FACTOR == 0:
    bark_viewer.drawWorld(bark_world, eval_agent_ids=[bark_agent.id], show=False)
    interface_viewer.update(bark_viewer.screen_surface, (EGO_CAMERA_SIZE[0], 0) + BARK_SCREEN_DIMS)

except (KeyboardInterrupt, SystemExit):
  raise
finally:
  # kill the child of subprocess
  os.system("fuser {}/tcp -k".format(CARLA_PORT))
