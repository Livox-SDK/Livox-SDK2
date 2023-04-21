#!/bin/bash

echo "Building in new directory:" `pwd`"/build"

rm -rf build
mkdir build
cd build
cmake .. && make -j
sudo make install

echo "Finished build and install."
