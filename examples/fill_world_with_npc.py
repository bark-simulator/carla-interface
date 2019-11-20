from client.carla_client import CarlaClient

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


BARK_PATH = "external/com_github_bark-simulator_bark/"
CARLA_MAP = "Town02"
CARLA_PORT = 2000
DELTA_SECOND = 0.1
SYNCHRONOUS_MODE = True


param_server = ParameterServer(filename=BARK_PATH+"examples/params/od8_const_vel_one_agent.json")

# World Definition
bark_world = World(param_server)

# Model Definitions
behavior_model = BehaviorConstantVelocity(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel(param_server)

# Map Definition
xodr_parser = XodrParser(BARK_PATH+"modules/runtime/tests/data/"+CARLA_MAP+".xodr")
map_interface = MapInterface()
map_interface.set_open_drive_map(xodr_parser.map)
bark_world.set_map(map_interface)
# Agent Definition
agent_2d_shape = CarLimousine()

init_state = np.empty(5)

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time in simulation",
                                           0.05]
# sim_real_time_factor = param_server["simulation"]["real_time_factor",
#                                                   "execution in real-time or faster",
#                                                   100]

# Open Carla simulation server
try:
  server = subprocess.Popen("external/carla/CarlaUE4.sh")
  time.sleep(7)  # Wait for carla

  # Connect to Carla server
  client = CarlaClient(CARLA_MAP)
  client.connect(port=CARLA_PORT)
  client.set_synchronous_mode(SYNCHRONOUS_MODE, DELTA_SECOND)

  blueprint_library = client.get_blueprint_library()

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()

  for i in range(40):
    # create agent (actor) in Carla
    bp = random.choice(blueprint_library.filter('vehicle'))
    transform = random.choice(client.get_spawn_points())
    agent_carla_id, _ = client.spawn_actor(bp, transform)

    if agent_carla_id == None:
      continue

    client.set_autopilot(agent_carla_id, True)

    # create agent object in BARK
    agent_params = param_server.addChild("agent{}".format(i))
    agent = Agent(init_state,
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
  viewer = PygameViewer(params=param_server,
                        follow_agent_id=agent.id,
                        x_range=[-100, 100],
                        y_range=[-100, 100],
                        screen_dims=[1000, 1000])

  # main loop
  print("START")
  while True:
    if SYNCHRONOUS_MODE:
      client.tick()
    viewer.clear()

    agent_state_map = client.get_vehicles_state(carla_to_bark_id)
    # TODO: use time different in carla if synchronous mode if off
    bark_world.fill_world_from_carla(DELTA_SECOND, agent_state_map) 
    viewer.drawWorld(bark_world, eval_agent_ids=[agent.id])
    viewer.show(block=False)

except (KeyboardInterrupt, SystemExit):
  raise
finally:
  # kill the child of subprocess if it is not killed
  os.system("fuser {}/tcp -k".format(CARLA_PORT))
