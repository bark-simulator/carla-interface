from client.carla_client import CarlaClient
from client.raw_control import RawControl

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

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla


BARK_PATH = "external/com_github_bark_simulator_bark/"
BARK_MAP = "Town02"
CARLA_MAP = "Town02"
CARLA_PORT = 2000
DELTA_SECOND = 0.033
NUM_BARK_TRAJECTORY_STEP = 10
SYNCHRONOUS_MODE = True


CARLA_DELTA = DELTA_SECOND
# CARLA_DELTA = DELTA_SECOND/NUM_BARK_TRAJECTORY_STEP
assert(1/10 >= CARLA_DELTA >= 1/60)


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
init_state = np.array([0, -3.2, -167, 0, 1])
goal_polygon = Polygon2d([0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
goal_polygon = goal_polygon.translate(Point2d(190, -124))

# World Simulation
# sim_step_time = param_server["simulation"]["step_time",
#                                            "Step-time in simulation",
#                                            0.05]
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
  client.set_synchronous_mode(SYNCHRONOUS_MODE, CARLA_DELTA)

  rc = RawControl(client.client)

  blueprint_library = client.get_blueprint_library()

  # use for converting carla actor id to bark agent id
  carla_to_bark_id = dict()

  agent_carla_id = None
  while agent_carla_id == None:
    # create agent (actor) in Carla
    bp = random.choice(blueprint_library.filter('vehicle'))
    transform = carla.Transform(carla.Location(x=float(init_state[1]), y=-float(init_state[2]), z=0.3),
                                carla.Rotation(yaw=math.degrees(init_state[3])))
    agent_carla_id, agent_carla = client.spawn_actor(bp, transform)

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
  viewer = PygameViewer(params=param_server,
                        follow_agent_id=bark_agent.id,
                        x_range=[-100, 100],
                        y_range=[-100, 100],
                        screen_dims=[1000, 1000])

  # main loop
  print("START")
  for i in range(99999):
    if SYNCHRONOUS_MODE:
      client.tick()
    viewer.clear()

    agent_state_map = client.get_vehicles_state(carla_to_bark_id)
    # TODO: use time different in carla if synchronous mode is off
    if i == 0:
      bark_world.fill_world_from_carla(0, agent_state_map)

    plan = bark_world.plan_agents(DELTA_SECOND, [bark_agent.id])[bark_agent.id]
    # for r in range(1, NUM_BARK_TRAJECTORY_STEP):
    #   rc.control(agent_carla, plan[r-1][1:3], plan[r][1:3], plan[r][4], plan[r][3])
    #   bark_world.fill_world_from_carla(CARLA_DELTA, agent_state_map)
    rc.control(agent_carla, plan[-2][1:3], plan[-1][1:3], plan[-1][4], plan[-1][3])
    bark_world.fill_world_from_carla(CARLA_DELTA, agent_state_map)

    viewer.drawWorld(bark_world, eval_agent_ids=[bark_agent.id])
    viewer.show(block=False)

except (KeyboardInterrupt, SystemExit):
  raise
finally:
  # kill the child of subprocess if it is not yet killed
  os.system("fuser {}/tcp -k".format(CARLA_PORT))
