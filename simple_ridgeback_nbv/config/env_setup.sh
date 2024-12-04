#!/bin/bash

# Define workspace directory as a variable
WORKSPACE_DIR=~/catkin_ws

# Source the workspace setup file
source $WORKSPACE_DIR/devel/setup.bash

# Add SuperGluePretrainedNetwork to the PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$WORKSPACE_DIR/src/SuperGluePretrainedNetwork

# Ridgeback extra to put on 
export RIDGEBACK_URDF_EXTRAS="$WORKSPACE_DIR/src/StickyNAV/simple_ridgeback_nbv/urdf/fridgeback.urdf.xacro"

# Enable the Hokuyo laser
export RIDGEBACK_FRONT_HOKUYO_LASER=1