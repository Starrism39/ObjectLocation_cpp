#include <vector>
#include <map>
#include <random>
#include <unordered_map>
#include <utility>
#include <string>
#include <Eigen/Dense>
#include <iostream>

#include "framework/difference.h"

class RTKDifference : public Difference
{

public:
    RTKDifference(double time_slice, int class_1, double x1, double y1, int max_queue_length = 0);
    void configureClass(int class_id, std::pair<double, double> pos);
    std::vector<Package> process(const std::vector<Package> &packages) override;

private:
    int class_1_;
    int max_queue_length;
    std::unordered_map<int, std::pair<double, double>> class_config;
    std::unordered_map<int, std::pair<double, double>> del_class_config_1;
    std::unordered_map<int, std::pair<double, double>> del_class_config_2; 
};
