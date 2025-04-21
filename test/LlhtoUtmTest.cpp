#include "input/converter.h"

int main(){
    auto [easting, northing, zone, height] = LLHtoUTM(109.030844, 34.348630, 50.0);
    std::cout << easting << " " << northing << " " << zone << " " << height << std::endl;
}