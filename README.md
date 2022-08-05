# carla-interface
The carla-interface is adapted to the latest release of carla (0.9.13) and master branch of BARK.
## Screenshot

![example](https://github.com/bark-simulator/carla-interface/blob/master/doc/npc_example.png)

## Requirement
This repository has the same environment with BARK. If you haven't yet used BARK, check here to [install BARK](https://github.com/bark-simulator/bark/blob/master/docs/source/installation.md)
- Ubuntu 18.04 or later
- Nvidia driver
- OpenGL (to run with CPU only)
- Bazel 
- virtualenv

## Installation
[bazel install instruction](https://docs.bazel.build/versions/master/install-ubuntu.html)
[Install carla latest version or (0.9.13) using deb installation](https://carla.readthedocs.io/en/latest/start_quickstart/#a-debian-carla-installation)
Install dependencies
```python
sudo apt-get update
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install python3.7 python3.7-dev python3.7-tk
pip3 install virtualenv==16.7.8 
```

We then call the install script, that will install a virtual environment with all required dependencies. (Only once!)
```python
bash install.sh
```

## Run examples
We then go into the virtual environment (Execute every time after openning a new terminal!)
```python
source dev_into.sh
```

```python
# Execute at project root directory
bazel run //examples:fill_world_with_npc
```

### Examples
- fill_world_with_npc: spawn npc agents in Carla and Bark simultaneously, which controlled by Carla autopilot, the states/actions are mirrored into Bark
- fill_world_with_bark_ego: spawn npc agents and one ego agent simultaneously, ego agent is controlled/palnned from Bark
- simulate_on_Crossing8Course : simulate on custom opendrive map
- simulate_on_city_highway_straight: simulate on custom opendrive map, with limited range

## Known issue
- Ego agent cannot drive through some junction
