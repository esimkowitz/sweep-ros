#!/bin/bash

# start aliases for pushd/popd so they don't echo to console
pushd () {
    command pushd "$@" > /dev/null
}

popd () {
    command popd "$@" > /dev/null
}
# end pushd/popd aliases