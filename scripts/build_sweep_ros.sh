#!/bin/bash

# Get the script directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

source $SCRIPT_DIR/utils.sh

pushd $SCRIPT_DIR/..

rosdep update
rosdep install --from-paths . -r -y
colcon build

popd