import math
import numpy as np
import sys
import glob

try:
  sys.path.append(glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
  pass
import carla


class RawControl:
  def __init__(self, client):
    # self.control = carla.VehicleControl()
    self.client = client

  def control(self, vehicle, last_location, location, velocity, steer_dir):
    # TODO: use raw control instead of setting the state directly

    # self.control.reverse = self.control.gear < 0
    # vehicle.applycontrol(self.control)
    transform = carla.Transform(carla.Location(x=float(location[0]), y=-float(location[1])),
                                carla.Rotation(yaw=math.degrees(-steer_dir)))
    v = np.append(location, 0)-np.append(last_location, 0)
    v = v / (np.linalg.norm(v) + 1e-16)*velocity
    velocity_vec = carla.Vector3D(x=v[0], y=-v[1], z=v[2])
    # print(transform)

    vehicle.set_transform(transform)
    # vehicle.set_velocity(velocity_vec)
