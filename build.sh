#!/bin/bash

echo "Building in new directory:" `pwd`"/build"

rm -rf build
mkdir build
cd build
cmake .. && make -j12
cpack -G DEB

echo "Finished build and created deb package."
