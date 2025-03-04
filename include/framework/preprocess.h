#include"framework/module.h"

class PreProcess : public Module{
public:
    PreProcess(const std::string& name, double time_slice, size_t maxQueueLength = 0);

    virtual std::vector<Package> process(std::vector<Package>& packages) = 0;
    void run() override;

protected:
    double timeSlice;
};