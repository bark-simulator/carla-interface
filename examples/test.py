#from modules.runtime.commons.xodr_parser import XodrParser
import subprocess
import sys
import os
import glob
import time

try:
  sys.path.append(glob.glob('external/carla/PythonAPI/carla/dist/carla-*.egg')[0])
except IndexError:
  pass

import carla


# print(glob.glob("external/carla/*"))

with subprocess.Popen("external/carla/CarlaUE4.sh") as server:

  time.sleep(5)

  client = carla.Client('localhost', 2000)
  client.set_timeout(2)  # in second
  client.load_world("Town01")
  world = client.get_world()

  print("end")
