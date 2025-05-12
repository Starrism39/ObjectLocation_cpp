#include "framework/module.h"
#include <vector>
class Difference : public Module
{
public:
    Difference(const std::string &name, double time_slice, size_t maxQueueLength = 0);

    virtual std::vector<Package> process(const std::vector<Package> &packages) = 0;

    void run() override;

protected:
    double timeSlice;

};