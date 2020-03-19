#!/bin/bash
virtualenv -p python3.5 ./python/venv # CARLA provides only egg file with python 2.7 and 3.5
source ./python/venv/bin/activate && pip install -r tools/installers/requirements.txt
