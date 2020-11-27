# dream-multi-mapper


1-Requirements:

-ROS Melodic

-sudo apt install ros-melodic-geodesy

-libpointmatcher with semanticOutlierFilter fork: git clone https://github.com/georges-chahine/libpointmatcher

2-Resolve FLANN library conflict:

-sudo nano /usr/include/flann/util/serialization.h

-replace:

-#include "flann/ext/lz4.h"
-AND
-#include "flann/ext/lz4hc.h"

-by:

-#include "lz4.h"
-#include "lz4hc.h"

3-Compile

-mkdir build
-cd build
-cmake ..
-make -j
