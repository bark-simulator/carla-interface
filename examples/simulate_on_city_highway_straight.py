from cosimulation_modules.client.carla_client import CarlaClient
from cosimulation_modules.client.sensors import CameraManager
from cosimulation_modules.client.viewer import CosimulationViewer
from cosimulation_modules.client.controller import Controller

from bark.core.world.agent import Agent
from bark.core.models.behavior import BehaviorIDMClassic
from bark.core.world import World
from bark.core.world.map import MapInterface
from bark.core.models.dynamic import SingleTrackModel, StateDefinition
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.geometry.standard_shapes import CarLimousine
from bark.core.geometry import Point2d, Polygon2d
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.pygame_viewer import PygameViewer
from bark.runtime.commons.xodr_parser import XodrParser
from bark.core.world.goal_definition import GoalDefinitionPolygon

import subprocess
import os
import glob
import numpy as np
import pygame as pg
import time
import logging
import math


EG_PATH = os.path.dirname(__file__)
BARK_MAP = "city_highway_straight"
OPENDRIVE_MAP = "city_highway_straight"
CARLA_PORT = 2000
DELTA_SECOND = 0.05
SYNCHRONOUS_MODE = True
BARK_SCREEN_DIMS = (800, 800)
CARLA_LOW_QUALITY = False
NUM_CAMERAS = 1


class Cosimulation:
    def __init__(self):
        self.carla_server = None
        self.carla_client = None
        self.carla_controller = None
        self.bark_viewer = None
        self.cosimulation_viewer = None
        self.launch_args = ["external/carla/CarlaUE4.sh", "-quality-level=Low"]

        # Bark parameter server
        self.param_server = ParameterServer(
            filename=os.path.join(EG_PATH,
            "eg_params/od8_const_vel_one_agent.json"))

        # World Definition
        self.bark_world = World(self.param_server)

        # Model Definitions
        self.behavior_model = BehaviorIDMClassic(self.param_server)
        self.execution_model = ExecutionModelInterpolate(self.param_server)
        self.dynamic_model = SingleTrackModel(self.param_server)

        # Map Definition
        xodr_parser = XodrParser(EG_PATH + "/eg_params/" +
                                 BARK_MAP + ".xodr")
        self.map_interface = MapInterface()
        self.map_interface.SetOpenDriveMap(xodr_parser.map)
        self.bark_world.SetMap(self.map_interface)

        # Bark agent definition
        self.agent_2d_shape = CarLimousine()

        # use for converting carla actor id to bark agent id
        self.carla_2_bark_id = dict()
        # store the camera id attached to an agent
        self.carla_agents_cam = dict()

    def initialize_viewer(self):
        # Viewer of Bark simulation, the pygame surface will be extracted
        self.bark_viewer = PygameViewer(params=self.param_server,
                                        use_world_bounds=True,
                                        screen_dims=BARK_SCREEN_DIMS,
                                        screen_map_ratio = 1.5)

        # Viewer of cosimulation
        # Set the number of cameras to show both simulation side by side
        # windows from Bark simulation & camera image from Carla simulation
        self.cosimulation_viewer = CosimulationViewer(
            BARK_SCREEN_DIMS, num_cameras=NUM_CAMERAS)

    def close(self):
        pg.display.quit()
        pg.quit()

        # kill the child of the subprocess
        # sometimes the socket is not killed, blocking the launch of carla
        # server
        os.system("fuser {}/tcp -k".format(CARLA_PORT))
        exit()

    def launch_carla_server(self):
        self.carla_server = subprocess.Popen(
            self.launch_args[0] if not CARLA_LOW_QUALITY else self.launch_args)
        # Wait for launching carla
        time.sleep(8)

    def connect_carla_server(self):
        """
        create a carla client and try connect to carla server
        """
        self.carla_client = CarlaClient()
        self.carla_client.connect(
            port=CARLA_PORT,
            timeout=10)

        with open("cosimulation_modules/maps/" + OPENDRIVE_MAP + ".xodr") as f:
            try:
                xodr_file = f.read()
            except OSError:
                raise ValueError("file could not be readed.")

            self.carla_client.client.generate_opendrive_world(xodr_file)

        self.carla_client.set_synchronous_mode(SYNCHRONOUS_MODE, DELTA_SECOND)
        self.carla_controller = Controller(self.carla_client)

    def spawn_npc_agents(self, num_agents):
        """spawn npc agents in both Carla and Bark

        Arguments:
            num_agents {int} -- number of agents to be spawned
        """
        tf = self.carla_client.generate_tranformation(
            x=5114, y=5300, z=0.3, yaw=90)
        for i in range(num_agents):
            carla_agent_id = self.carla_client.spawn_random_vehicle(
                num_retries=5, transform=tf)
            if carla_agent_id is not None:
                self.carla_client.set_autopilot(carla_agent_id, True)

                # spawn agent object in BARK
                agent_params = self.param_server.addChild("agent{}".format(i))
                carla_agent_state = self.carla_client.get_vehicle_state(carla_agent_id)
                if carla_agent_state is None:
                    carla_agent_state = np.empty(5)
                
                bark_agent = Agent(carla_agent_state,
                                   self.behavior_model,
                                   self.dynamic_model,
                                   self.execution_model,
                                   self.agent_2d_shape,
                                   agent_params,
                                   None,  # goal_lane_id
                                   self.map_interface)
                self.bark_world.AddAgent(bark_agent)
                self.carla_2_bark_id[carla_agent_id] = bark_agent.id

        if len(self.carla_2_bark_id) != num_agents:
            logging.warning("Some agents cannot be spawned due to collision in the spawning location, {} agents are spawned".format(
                len(self.carla_2_bark_id)))
        else:
            logging.info("{} agents spawned sucessfully.".format(num_agents))
        return carla_agent_state[0]

    def initialize_camera_manager(self, surfaces):
        """create object for fetching image from carla

        Arguments:
            surfaces {list} -- list of pygame surfaces
        """
        self.cam_manager = CameraManager(
            surfaces, synchronous_mode=SYNCHRONOUS_MODE)

    def simulation_loop(self, carla_ego_id):
        bark_ego_id = self.carla_2_bark_id[carla_ego_id]

        self.bark_viewer.clear()
        self.cosimulation_viewer.tick()

        agent_state_map = self.carla_client.get_vehicles_state(
            self.carla_2_bark_id)
        self.bark_world.UpdateAgentStateFromExtern(DELTA_SECOND, agent_state_map)

        self.bark_world.PlanAndExecuteAgentsWithID(
            DELTA_SECOND, [bark_ego_id])

        ego_state, ego_action = self.bark_world.agents[bark_ego_id].history[-1]
        print("Ego State: ", ego_state)
        print("Ego Action: ", ego_action)
        self.carla_controller.control(self.carla_client.get_actor(
            carla_ego_id), [ego_state[int(StateDefinition.X_POSITION)],ego_state[int(StateDefinition.Y_POSITION)], 0.0], ego_state[int(StateDefinition.VEL_POSITION)], ego_state[int(StateDefinition.THETA_POSITION)])

        if SYNCHRONOUS_MODE:
            frame_id = self.carla_client.tick()
            self.cam_manager.fetch_image(frame_id)

        self.cosimulation_viewer.update_cameras(self.cam_manager.surfaces)

        self.bark_viewer.drawWorld(
            self.bark_world, show=False,
            eval_agent_ids=[bark_ego_id])
        self.cosimulation_viewer.update_bark(self.bark_viewer.screen_surface)


        self.cosimulation_viewer.show()


sim = Cosimulation()

try:
    sim.launch_carla_server()
    sim.connect_carla_server()

    sim_t0 = sim.spawn_npc_agents(1)
    print("Sim Time: ",sim_t0)
    sim.bark_world.time = sim_t0
    # [TIME_POSITION, X_POSITION, Y_POSITION, THETA_POSITION, VEL_POSITION, ...]
    ego_initial = np.array([sim_t0, 5123, 5250, 0, 0])
    goal_polygon = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon = goal_polygon.Translate(Point2d(5118, 5300))

    bp_lib = sim.carla_client.get_blueprint_library()
    bp = bp_lib.filter("vehicle.dodge.charger_police")[0]
    tf = sim.carla_client.generate_tranformation(
        x=ego_initial[1], y=ego_initial[2], z=0.3, pitch=0, yaw=math.degrees(ego_initial[3]), roll=0)

    carla_ego_id = sim.carla_client.spawn_actor(bp, tf)

    if carla_ego_id is None:
        raise Exception("ego agent cannot be spawned")

    agent_params = sim.param_server.addChild("agent 1")
    bark_ego = Agent(ego_initial,
                     sim.behavior_model,
                     sim.dynamic_model,
                     sim.execution_model,
                     sim.agent_2d_shape,
                     agent_params,
                     GoalDefinitionPolygon(goal_polygon),
                     sim.map_interface)
    sim.bark_world.AddAgent(bark_ego)
    sim.carla_2_bark_id[carla_ego_id] = bark_ego.id

    cam_id = sim.carla_client.spawn_sensor(carla_ego_id,
                                           "sensor.camera.rgb",
                                           location=(0, 0, 15),
                                           rotation=(270, 0, 0))
    sim.carla_agents_cam[carla_ego_id] = sim.carla_client.get_actor(cam_id)
    sim.initialize_camera_manager(sim.carla_agents_cam)

    logging.info("Start simulation")
    sim.initialize_viewer()

    sim.carla_client.tick()
    while True:
        sim.simulation_loop(carla_ego_id)

except (KeyboardInterrupt, SystemExit):
    logging.info("Simulation canceled by user")
except Exception as e:
    logging.exception(str(e))
finally:
    sim.close()
