# dream-multi-mapper


1-Requirements:

-ROS Melodic
sudo apt install ros-melodic-geodesy


2-Resolve FLANN library conflict:

sudo nano /usr/include/flann/util/serialization.h

replace:

#include "flann/ext/lz4.h"
#include "flann/ext/lz4hc.h"

by:

#include "lz4.h"
#include "lz4hc.h"

3-Compile

mkdir build
cd build
cmake ..
make -j
