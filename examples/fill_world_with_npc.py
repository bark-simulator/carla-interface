from client.carla_client import CarlaClient
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
import pygame as pg


BARK_PATH = "external/com_github_bark_simulator_bark/"
BARK_MAP = "Town02"
CARLA_MAP = "Town02"
CARLA_PORT = 2000
DELTA_SECOND = 0.1
SYNCHRONOUS_MODE = True
BARK_SCREEN_DIMS = (600, 600)
CARLA_LOW_QUALITY = False


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

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time in simulation",
                                           0.05]
# sim_real_time_factor = param_server["simulation"]["real_time_factor",
#                                                   "execution in real-time or faster",
#                                                   100]

# Open Carla simulation server
try:
  args = ["external/carla/CarlaUE4.sh", "-quality-level=Low"]
  server = subprocess.Popen(args[0] if not CARLA_LOW_QUALITY else args)
  time.sleep(6)  # Wait for carla

  # Connect to Carla server
  client = CarlaClient(CARLA_MAP)
  client.connect(port=CARLA_PORT,timeout=10)
  client.set_synchronous_mode(SYNCHRONOUS_MODE, DELTA_SECOND)

  blueprint_library = client.get_blueprint_library()

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()

  carla_cams=dict()

  for i in range(15):
    # spawn agent (actor) in Carla
    bp = random.choice(blueprint_library.filter('vehicle'))
    transform = random.choice(client.get_spawn_points())
    agent_carla_id, _ = client.spawn_actor(bp, transform)

    if agent_carla_id != None:
      _, cam = client.spawn_sensor(agent_carla_id, "sensor.camera.rgb",
                                              location=(0, 0, 30), rotation=(270, 0, 0))
      carla_cams[agent_carla_id]=cam
    else:
      continue

    client.set_autopilot(agent_carla_id, True)

    # spawn agent object in BARK
    agent_params = param_server.addChild("agent{}".format(i))
    agent = Agent(np.empty(5),
                  behavior_model,
                  dynamic_model,
                  execution_model,
                  agent_2d_shape,
                  agent_params,
                  None,  # goal_lane_id
                  map_interface)
    bark_world.add_agent(agent)

    carla_to_bark_id[agent_carla_id] = agent.id

  # bark viewer
  bark_viewer = PygameViewer(params=param_server,
                        follow_agent_id=agent.id,
                        x_range=[-100, 100],
                        y_range=[-100, 100],
                        screen_dims=BARK_SCREEN_DIMS)
  # 800*600 is the default size of carla's camera
  interface_viewer = Viewer(1, BARK_SCREEN_DIMS)
  # cam_man = CameraManager(carla_cams,synchronous_mode=True)
  cam_man = CameraManager({agent_carla_id:cam},synchronous_mode=True)

  # for id,cam in carla_cams.items():
  #   if SYNCHRONOUS_MODE:
  #     cam.listen(cam_man.image_queues[id].put)
  #   else:
  #     cam.listen(lambda image: cam_man.RGBcamToImage(image=image,cam_id=id))

  # main loop
  print("START")
  while True:
    bark_viewer.clear()
    interface_viewer.tick()
    if SYNCHRONOUS_MODE:
      frame_id=client.tick()
      cam_man.fetchImage(frame_id,agents_id=[agent_carla_id])

    interface_viewer.update_cameras(cam_man.surfaces,agents_id=[agent_carla_id])

    agent_state_map = client.get_vehicles_state(carla_to_bark_id)
    # TODO: use time different in carla if synchronous mode is off
    bark_world.fill_world_from_carla(DELTA_SECOND, agent_state_map)
    bark_viewer.drawWorld(bark_world,eval_agent_ids=[agent.id])
    interface_viewer.update(bark_viewer.screen, (0, 0), BARK_SCREEN_DIMS)
    interface_viewer.flip()
    
except (KeyboardInterrupt, SystemExit):
  raise
finally:
  # kill the child of subprocess if it is not yet killed
  os.system("fuser {}/tcp -k".format(CARLA_PORT))
  pg.display.quit()
  pg.quit()
