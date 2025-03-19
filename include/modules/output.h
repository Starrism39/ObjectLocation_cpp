#pragma once

#include "framework/sink.h"
#include <vector>
#include <string>
#include <thread>



class Output : public Sink {
public:
    Output(double time_slice, int max_queue_length = 0);
    std::shared_ptr<DataPackage> process(const std::vector<Package>& packages) override;
private:
    void UTMToWGS84(const std::vector<double>& utm, double wgs84[3]);
};

    