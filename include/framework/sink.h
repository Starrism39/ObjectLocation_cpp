#include"framework/module.h"

class Sink : public Module{
public:
    Sink(const std::string& name, double time_slice, size_t maxQueueLength = 0);
    virtual ~Sink() = default;

    virtual std::shared_ptr<DataPackage> process(const std::vector<Package>& packages) = 0;
    void run() override;

private:
    double timeSlice;
};