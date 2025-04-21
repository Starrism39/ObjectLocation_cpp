#include "framework/module.h"
#include <vector>
class Arrange : public Module
{
public:
    Arrange(const std::string &name, size_t maxQueueLength = 0);

    virtual void process() = 0;

    void run() override;

};