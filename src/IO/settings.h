#include "vector"
#include <iostream>

namespace mapper
{

extern bool tfInit;
extern std::vector<std::vector<int>> palette;

extern std::vector<std::string> labels;

extern std::vector<float> semanticWeights;

extern std::vector<std::vector<double>> travelledDistance;   //to align all surveys with the same reference, this has to be global

}
