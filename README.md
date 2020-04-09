# carla-interface

## Requirement
- Ubuntu 16.04 or later (recommended)
- Nvidia driver v384.11 or later
- OpenGL (to run with CPU only)
- Bazel 0.25.0 or later (requires Java)
- virtualenv

## Installation
[bazel install instruction](https://docs.bazel.build/versions/master/install-ubuntu.html)

Install dependencies
```python
sudo apt-get update
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install python3.5-dev
sudo apt-get install python3-pip
sudo pip3.5 install virtualenv 
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
# Carla will be downloaded at the first time
bazel run //examples:fill_world_with_npc
```

### Examples
- fill_world_with_npc: spawn npc agents in Carla and Bark simultaneously, which controlled by Carla autopilot, the actions are mirrored into Bark
- fill_world_with_bark_ego: spawn npc agents and one ego agent simultaneously, ego agent is controlled from Bark
- simulate_on_Crossing8Course & : simulate on custom opendrive map

## TODO
- Install Carla using deb installation

## Known issue
- Simulation crash when driving into some junction (possible bug releated to Bark)
- Carla autopilot mode does not support custom opendrive map without junction (simulate_on_city_highway_straight)
- Spawn agents with default spawn location on custom opendrive may lead to error
