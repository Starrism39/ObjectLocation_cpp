#include "framework/module.h"
#include <vector>
class Filter : public Module{
public:
    Filter(const std::string& name, double time_slice, size_t maxQueueLength = 0);
    virtual ~Filter() = default;

    // 纯虚函数，由派生类实现
    virtual std::vector<Package> process(std::vector<Package>& packages) = 0;

    // 实现基类的run函数
    void run() override;

protected:
    double timeSlice;
};