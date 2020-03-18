from client.carla_client import CarlaClient
from client.keyboard_control import KeyboardControl
from client.sensor import SensorData

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
import pygame

BARK_PATH = "external/com_github_bark_simulator_bark/"
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
  time.sleep(8)  # Wait for carla

  # Connect to Carla server
  client = CarlaClient(CARLA_MAP)
  client.connect()
  # TODO: synchronous mode
  # client.set_synchronous_mode(True, sim_step_time)

  blueprint_library = client.get_blueprint_library()

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()

  while True:
    ego_bp = random.choice(blueprint_library.filter('vehicle'))
    ego_transform = random.choice(client.get_spawn_points())
    ego_carla_id, ego = client.spawn_actor(ego_bp, ego_transform)

    if ego_carla_id != None:
      cam_carla_id, cam = client.spawn_sensor(ego_carla_id, 'sensor.camera.rgb', location=(1.5, 0.0, 2.4),rotation=(0,0,0))
      break

  # create agent object in BARK
  agent_params = param_server.addChild("agent1")
  agent = Agent(init_state,
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                GoalDefinitionPolygon(goal_polygon),  # goal_lane_id
                map_interface)
  bark_world.add_agent(agent)

  carla_to_bark_id[ego_carla_id] = agent.id

  # # bark viewer
  # viewer = PygameViewer(params=param_server,
  #                       x_range=[-300, 300],
  #                       y_range=[-300, 300],
  #                       screen_dims=[1000, 1000])
  viewer = MPViewer(params=param_server)

  pygame.init()
  display = pygame.display.set_mode((800, 600), pygame.DOUBLEBUF)
  clock = pygame.time.Clock()

  sensors = SensorData()
  cam.listen(lambda image: sensors.RGBcamToImage(image))
  controller = KeyboardControl(client.get_world())

  while True:
    frame_start = time.time()
    # show bark world
    viewer.clear()
    agent_state_map = client.get_vehicles_state(carla_to_bark_id)
    bark_world.fill_world_from_carla(sim_step_time, agent_state_map)
    viewer.drawWorld(bark_world)
    viewer.show(block=False)

    clock.tick()
    # TODO: synchronous mode
    controller.control(ego, clock)

    # show ego camera
    if sensors.cam_surface is not None:
      display.blit(sensors.cam_surface, (0, 0))
    pygame.display.flip()
    print("FPS: {:.2f}".format(1/(time.time()-frame_start)))
