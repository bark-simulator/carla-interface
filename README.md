# carla-interface

Download Carla 0.9.6 (pre-compiled): [link](https://github.com/carla-simulator/carla/releases/tag/0.9.6)

### Requirement
Linux:
- Ubuntu 16.04 or later (recommendded)
- Python 3.5
  - pygame
  - numpy
- Nvidia driver v384.11 or later
- OpenGL (to run with CPU only)

### Run Carla server
Extract the file and run `./CarlaUE4.sh` at the Carla's root directory with a display card.\
Run `./CarlaUE4.sh -opengl` with CPU only

### Run Carla client
See examples in `./PythonAPI/examples`, run the script with Python 3.5

### Setting quality level:
Only two levels can be selected: `Low` and `Epic` (default)

`./CarlaUE4.sh -quality-level=Low`

### Troubleshooting
If the window crashs, try to run with `./CarlaUE4.sh -opengl`. If it can be executed successfully, then it would be the problem related too display card driver.
