#include <vector>
#include <string>
#include <unordered_map>

#include"framework/module.h"

class TimeFilter {
public:
    TimeFilter(int max_queue_length);
    std::vector<Package> process(const std::vector<Package>& data);
    
private:
    int max_queue_length;
};