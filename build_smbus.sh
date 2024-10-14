#!/bin/bash

if command -v ccache > /dev/null; then
    CMAKE_ARGS="$CMAKE_ARGS \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
fi

if [ "$1" = "debug" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-O0" -DCMAKE_C_FLAGS="-O0" -DBUILD_TESTING=OFF"
else
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_CXX_FLAGS="-O3" -DCMAKE_C_FLAGS="-O3" -DBUILD_TESTING=OFF"
fi

echo $CMAKE_ARGS

source /opt/ros/humble/setup.bash
colcon build --parallel-workers 4 --symlink-install --cmake-args $CMAKE_ARGS --packages-up-to csmbus