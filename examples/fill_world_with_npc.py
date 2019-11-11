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

init_state = np.zeros(5)
goal_polygon = Polygon2d([0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
goal_polygon = goal_polygon.translate(Point2d(-191.789, -50.1725))

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time in simulation",
                                           0.05]
# sim_real_time_factor = param_server["simulation"]["real_time_factor",
#                                                   "execution in real-time or faster",
#                                                   100]

# Open Carla simulation server
with subprocess.Popen("external/carla/CarlaUE4.sh") as server:
  time.sleep(10)  # Wait for carla

  # Connect to Carla server
  client = CarlaClient(CARLA_MAP)
  client.connect()
  # TODO: synchronous mode
  # client.set_synchronous_mode(True, sim_step_time)

  blueprint_library = client.get_blueprint_library()

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()

  for i in range(10):
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
                  GoalDefinitionPolygon(goal_polygon),  # goal_lane_id
                  map_interface)
    bark_world.add_agent(agent)

    carla_to_bark_id[agent_carla_id] = agent.id

  # bark viewer
  viewer = PygameViewer(params=param_server,
                        x_range=[-300, 300],
                        y_range=[-300, 300],
                        screen_dims=[1000, 1000])

  # TODO: error if not calling this first
  bark_world.step(sim_step_time)

  # main loop
  for _ in range(0, 100):
    # TODO: synchronous mode
    viewer.clear()
    # world.step(sim_step_time)
    agent_state_map = client.get_vehicles_state(carla_to_bark_id)
    bark_world.fill_world_from_carla(sim_step_time, agent_state_map)
    viewer.drawWorld(bark_world)
    viewer.show(block=True)
