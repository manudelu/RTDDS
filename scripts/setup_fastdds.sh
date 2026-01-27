#!/bin/bash

export LD_LIBRARY_PATH="$HOME/Fast-DDS/install/lib:$LD_LIBRARY_PATH"
export PATH="$HOME/Fast-DDS/install/bin:$PATH"
export CMAKE_PREFIX_PATH="$HOME/Fast-DDS/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

echo "Fast-DDS environment loaded."