#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <vector>
#include <string>

// Class Remap
enum ClassType {
    BOAT = 2,
    CAR = 3,
    PERSON = 4,
    TRUNK = 6,
    OTHER = 100
};

const std::map<int, ClassType> CLS_MAP = {
    {0, OTHER},
    {1, OTHER},
    {2, BOAT},
    {3, CAR},
    {4, PERSON},
    {5, PERSON},
    {6, TRUNK},
    {30, PERSON},
    {31, PERSON},
    {32, TRUNK},
    {33, OTHER},
    {34, OTHER},
    {35, OTHER},
    {36, OTHER},
    {99, OTHER}
};

// Distortion Parameters
const std::vector<double> ultra_wide = {-0.010966, 0.0013977, -4.17E-05, 7.51E-06, -5.90E-10};
const std::vector<double> wide = {0.0016357, 0.000064971, -1.80E-06, -1.94E-05, -9.21E-10};
const std::vector<double> tele = {-0.0025488, 0.00010349, -9.92E-07, -2.08E-05, -5.40E-11};

const std::map<std::string, std::vector<double>> DISTORTION_MAP = {
    {"ultra-wide", ultra_wide},
    {"wide", wide},
    {"tele", tele}
};

// UAV Serial Numbers
const std::vector<std::string> UAV_LISTS = {
    "TH7923461373",
    "TH7923481017",
    "TH7923461368"
};

#endif // CONFIG_H
