#include"framework/module.h"

class Location : public Module{
public:
    Location(const std::string& name, size_t maxQueueLength = 0);
    virtual ~Location() = default;

    virtual void process(Package& packages) = 0;
    void run() override;
};