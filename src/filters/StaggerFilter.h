/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_STAGGERFILTER_H
#define RAILROAD_STAGGERFILTER_H

#include <cmath>
#include "../base/CloudProcessor.h"

namespace railroad
{

class StaggerFilter : public CloudProcessor
{
public:
    StaggerFilter(double maxStaggerLimit = 0.3, double staggerThreshold = 0.01, double minStaggerLimit = 0.1, const std::string &name = "StaggerFilter")
        : CloudProcessor(name), maxStaggerLimit(maxStaggerLimit), minStaggerLimit(minStaggerLimit), staggerThreshold(staggerThreshold) {}

protected:
    PointCloudPtr process();
    double maxStaggerLimit;
    double minStaggerLimit;
    double staggerThreshold;
};
} // railroad

#endif //RAILROAD_STAGGERFILTER_H
