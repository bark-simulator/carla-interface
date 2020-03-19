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

# Open Carla simulation server
try:
  args = ["external/carla/CarlaUE4.sh", "-quality-level=Low"]
  server = subprocess.Popen(args[0] if not CARLA_LOW_QUALITY else args)
  time.sleep(6)  # Wait for carla

  # Connect to Carla server
  client = CarlaClient(CARLA_MAP)
  client.connect(port=CARLA_PORT,timeout=10)
  client.set_synchronous_mode(SYNCHRONOUS_MODE, DELTA_SECOND)

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()
  carla_cams=dict()

  # spawn agent (actor) in Carla
  blueprint_library = client.get_blueprint_library()
  for i in range(15):
    bp = random.choice(blueprint_library.filter('vehicle'))
    transform = random.choice(client.get_spawn_points())
    carla_agent_id, _ = client.spawn_actor(bp, transform)

    if carla_agent_id == None:
      continue

    client.set_autopilot(carla_agent_id, True)
    
    # spawn camera on an agent
    if i==14:
        _, cam = client.spawn_sensor(carla_agent_id, 
                                    "sensor.camera.rgb",
                                    location=(0, 0, 30), 
                                    rotation=(270, 0, 0))
        carla_cams[carla_agent_id]=cam

    # spawn agent object in BARK
    agent_params = param_server.addChild("agent{}".format(i))
    bark_agent = Agent(np.empty(5),
                  behavior_model,
                  dynamic_model,
                  execution_model,
                  agent_2d_shape,
                  agent_params,
                  None,  # goal_lane_id
                  map_interface)
    bark_world.add_agent(bark_agent)
    carla_to_bark_id[carla_agent_id] = bark_agent.id

  # bark viewer
  bark_viewer = PygameViewer(params=param_server,
                        follow_agent_id=bark_agent.id,
                        x_range=[-100, 100],
                        y_range=[-100, 100],
                        screen_dims=BARK_SCREEN_DIMS)

  viewer = Viewer(1, BARK_SCREEN_DIMS)
  cam_manager = CameraManager(carla_cams,synchronous_mode=True)

  # main loop
  print("START")
  while True:
    bark_viewer.clear()
    viewer.tick()

    if SYNCHRONOUS_MODE:
      frame_id=client.tick()
      cam_manager.fetch_image(frame_id,agents_id=[carla_agent_id])

    viewer.update_cameras(cam_manager.surfaces,agents_id=[carla_agent_id])

    # get agents' state in carla, and fill the state into bark
    # TODO: use time different in carla if synchronous mode is off
    carla_agent_states = client.get_vehicles_state(carla_to_bark_id)
    bark_world.fill_world_from_carla(DELTA_SECOND, carla_agent_states)

    bark_viewer.drawWorld(bark_world,eval_agent_ids=[bark_agent.id])
    viewer.update(bark_viewer.screen, (0, 0), BARK_SCREEN_DIMS)

    viewer.flip()
    
except (KeyboardInterrupt, SystemExit):
  raise
finally:
  pg.display.quit()
  pg.quit()
  os.system("fuser {}/tcp -k".format(CARLA_PORT))# kill the child of the subprocess
  
