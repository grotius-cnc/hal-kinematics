#!/bin/bash

# For users that don't use qt, i made a single wrapper.cpp file. This is a combined next.h and next.cpp file.
# You need to install the orocos library first.

# Edit your linuxcnc path to the /rtlib
RTLIB="/opt/linuxcnc/rtlib/"

# Remove previous attempts from this dir.
rm -rf libnex.so

# This will produce a file wrapper.o
g++ -c -Wall -Werror -fPIC wrapper.cpp -I/usr/local/include/kdl -I/usr/include/eigen3 -lorocos-kdl

# Create shared lib, the libnext.so file
g++ -shared wrapper.o -o libnext.so

# Copy the libnext.so file to your linuxcnc/rtlib/
cp -rf libnext.so $RTLIB

# Tell linux where to find your so library
sudo ldconfig $RTLIB

