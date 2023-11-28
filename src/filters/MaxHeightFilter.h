/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_MAXHEIGHTFILTER_H
#define RAILROAD_MAXHEIGHTFILTER_H

#include <cmath>
#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class MaxHeightFilter : public SingleResultCloudProcessor
{
public:
    MaxHeightFilter(double offset = 0.0, bool keepBelow = true, const std::string &name = "MaxHeightFilter")
        : SingleResultCloudProcessor(name), offset(offset), keepBelow(keepBelow) {}

    MaxHeightFilter(SeedHelper::SeedType _runOnSeed, double offset = 0.0, bool keepBelow = true, const std::string &name = "MaxHeightFilter")
        : SingleResultCloudProcessor(name), offset(offset), keepBelow(keepBelow), _runOnSeed(_runOnSeed) {}

protected:
    double offset;
    bool keepBelow;
    SeedHelper::SeedType _runOnSeed = SeedHelper::SeedType::NONE;
    PointCloudPtr process();
};
} // railroad

#endif //RAILROAD_MAXHEIGHTFILTER_H
