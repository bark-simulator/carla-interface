# carla-interface

### Requirement
Linux:
- Ubuntu 16.04 or later (recommended)
- Python 3.5
- Nvidia driver v384.11 or later
- OpenGL (to run with CPU only)
- Bazel 0.25.0 or later (requires Java)
- virtualenv

### Run examples

Run `bazel run //examples:fill_world_with_npc` at root directory.\
It will download Carla at the first time, it will be slow as the size of Carla is quite large.

# Run Carla out of bazel

Using Carla 0.9.6 (3.1GB, pre-compiled): [link](https://github.com/carla-simulator/carla/releases/tag/0.9.6)

### Run Carla server
Extract the file and run `./CarlaUE4.sh` at the Carla's root directory with a display card.\
Run `./CarlaUE4.sh -opengl` with CPU only

### Run Carla client script
See examples in `./PythonAPI/examples`, run the script with Python 3.5

### Setting quality level:
Only two levels can be selected: `Low` and `Epic` (default)

`./CarlaUE4.sh -quality-level=Low`

### Troubleshooting
If the window crashs, try to run with `./CarlaUE4.sh -opengl`. If it can be executed successfully, then it would be the problem related to display card driver.
