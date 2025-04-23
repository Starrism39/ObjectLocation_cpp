#include <atomic>
#include <unordered_map>
#include <string>
#include <vector>

#include "framework/arrange.h"
// #include "utils/mesh_raycast.h"  // TODO

class PkgArrange : public Arrange
{
public:
    PkgArrange(int max_queue_length = 0);
    ~PkgArrange() = default;

    void process() override;

private:
    int max_queue_length;
};