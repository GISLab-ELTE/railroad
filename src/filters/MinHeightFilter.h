/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_MINHEIGHTFILTER_H
#define RAILROAD_MINHEIGHTFILTER_H

#include <cmath>
#include "../base/CloudProcessor.h"

namespace railroad
{

class MinHeightFilter : public CloudProcessor
{
public:
    MinHeightFilter(double offset = 0.0, bool keepAbove = true, const std::string &name = "MinHeightFilter")
        : CloudProcessor(name), offset(offset), keepAbove(keepAbove) {}

    MinHeightFilter(SeedHelper::SeedType _runOnSeed, double offset = 0.0, bool keepAbove = true, const std::string &name = "MinHeightFilter")
        : CloudProcessor(name), offset(offset), keepAbove(keepAbove), _runOnSeed(_runOnSeed) {}

protected:
    double offset;
    bool keepAbove;
    SeedHelper::SeedType _runOnSeed = SeedHelper::SeedType::NONE;
    PointCloudPtr process();
};
} // railroad

#endif //RAILROAD_MINHEIGHTFILTER_H
