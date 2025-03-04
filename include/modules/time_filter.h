#include <vector>
#include <string>
#include <unordered_map>

#include"framework/preprocess.h"

class TimeFilter : public PreProcess {
public:
    TimeFilter(double time_slice, int max_queue_length);
    std::vector<Package> process(std::vector<Package>& data) override;
    
private:
    int max_queue_length;
};