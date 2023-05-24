#!/bin/bash

# start aliases for pushd/popd so they don't echo to console
pushd () {
    command pushd "$@" > /dev/null
}

popd () {
    command popd "$@" > /dev/null
}
# end pushd/popd aliases


# Get the root directory for the git repo
reporoot=$(git rev-parse --show-toplevel)

pushd $reporoot/sweep-sdk/libsweep

# create and enter a build directory
mkdir -p build
cd build

# build and install the libsweep library
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --build . --target install
sudo ldconfig

popd