#!/bin/bash

# CARLA provides only egg file with python 2.7 and 3.5 in their binary release
virtualenv -p python3.7 ./python/venv
source ./python/venv/bin/activate && pip3 install -r tools/installers/requirements.txt
