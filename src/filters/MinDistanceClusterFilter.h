/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_MINDISTANCECLUSTERFILTER_H
#define RAILROAD_MINDISTANCECLUSTERFILTER_H

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class MinDistanceClusterFilter : public SingleResultCloudProcessor
{
public:
    MinDistanceClusterFilter(double minDistance = 5.0, const std::string &name = "MinDistanceClusterFilter")
        : SingleResultCloudProcessor(name) {}

    MinDistanceClusterFilter(SeedHelper::SeedType _runOnSeed, double minDistance = 5.0, const std::string &name = "MinDistanceClusterFilter")
        : SingleResultCloudProcessor(name), _runOnSeed(_runOnSeed) {}

protected:
    double minDistance;
    PointCloudPtr process();   
    SeedHelper::SeedType _runOnSeed = SeedHelper::SeedType::NONE;
};

} // railroad

#endif //RAILROAD_MINDISTANCECLUSTERFILTER_H
