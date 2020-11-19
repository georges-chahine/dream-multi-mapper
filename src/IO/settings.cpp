#include "IO/settings.h"

namespace mapper
{

bool tfInit=false;

std::vector<std::vector<int>> palette          { {128,  64, 128},
                                                 {244,  35, 232 },
                                                 {70,  70,  70 },
                                                 {102, 102, 156 },
                                                 {190, 153, 153 },
                                                 {153, 153, 153 },
                                                 {250, 170,  30 },
                                                 {220, 220,   0 },
                                                 {107, 142,  35 },
                                                 {152, 251, 152 },
                                                 {70, 130, 180 },
                                                 {220,  20, 60 },
                                                 {255, 0, 0 },
                                                 {0, 0, 142 },
                                                 {0, 0,  70 },
                                                 {0, 60, 100 },
                                                 {0, 80, 100 },
                                                 {0, 0, 230 },
                                                 {119,  11,  32 },
                                                 {213, 213, 213 },
                                                 {0, 0, 0}};

std::vector<std::string> labels { "Ground",
                                               "Sidewalk",
                                               "Building",
                                              "Wall",
                                               "Fence",
                                               "Build",
                                               "Pole",
                                               "Traffic sign",
                                               "Vegetation",
                                               "Terrain",
                                               "Sky",
                                               "Person",
                                               "Rider",
                                               "Car",
                                               "Truck",
                                               "Bus",
                                               "Train",
                                               "Motorcyle",
                                               "Bicycle",
                                               "Labelless",
                                               "Other"};


}
