/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_BANDPASSFILTER_H
#define RAILROAD_BANDPASSFILTER_H

#include <cmath>
#include "../base/CloudProcessor.h"

namespace railroad
{

class BandPassFilter : public CloudProcessor
{
public:
    BandPassFilter(double height = 2.0, double offset = NAN, const std::string &name = "BandPassFilter")
        : CloudProcessor(name), height(height/2), offset(offset) {}

    BandPassFilter(SeedHelper::SeedType _runOnSeed, double height = 2.0, double offset = NAN, const std::string &name = "BandPassFilter")
        : CloudProcessor(name), height(height/2), offset(offset), _runOnSeed(_runOnSeed) {}

protected:
    double height;
    double offset;    
    SeedHelper::SeedType _runOnSeed = SeedHelper::SeedType::NONE;
    PointCloudPtr process();
};
} // railroad

#endif //RAILROAD_BANDPASSFILTER_H
