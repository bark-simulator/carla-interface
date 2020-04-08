import math
import numpy as np
import sys
import glob

try:
    sys.path.append(
        glob.glob("external/carla/PythonAPI/carla/dist/carla-*.egg")[0])
except IndexError:
    pass
import carla


def rotate_2d_vector(x, y, radians):
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)
    return xx, yy


class Controller:
    def __init__(self, client):
        self.client = client

    def control(self, vehicle, last_location, location, velocity, steer_dir):
        # TODO: use raw control instead of setting the state directly
        transform = carla.Transform(carla.Location(x=float(location[0]), y=-float(location[1])),
                                    carla.Rotation(yaw=math.degrees(-steer_dir)))
        vehicle.set_transform(transform)
        # velocity=10

        v_x, v_y = rotate_2d_vector(0, 1, -steer_dir)
        v_x *= velocity
        v_y *= velocity
        velocity_vec = carla.Vector3D(x=v_x, y=v_y, z=0)
        vehicle.set_velocity(velocity_vec)
