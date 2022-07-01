#include <cstdio>
#include <algorithm>
#include <mutex>
#include "../car_detect/coordinate.h"
#include "../scheduler/schedulerRVO2/src/RVO.h"

enum sched_method {RVO2, vel_factor};

typedef struct DataPackageWithVel {
    AcrtTime time;
    RVO::Vector2 cars[3], vels[3];
} PackageWithVel;

class RVOScheduler {
public:
    RVOScheduler();
    ~RVOScheduler();

    mutex& getNewDataMutex();
    void setData(const PackageWithVel &);
    void setGoal(const std::vector<RVO::Vector2> &);

    // a monitor to fetch newest data
    const PackageWithVel& getNewData();
    void step();

private:
    std::mutex newDataMutex;
    PackageWithVel data, newData;

    RVO::RVOSimulator *sim;
    std::vector<RVO::Vector2> goals;
};