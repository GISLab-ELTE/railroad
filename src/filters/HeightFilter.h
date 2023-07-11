/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_HEIGHTFILTER_H
#define RAILROAD_HEIGHTFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{
class HeightFilter : public CloudProcessor
{
public:
    HeightFilter(const std::string &name = "HeightFilter", bool maxHeightLimit = false)
        : CloudProcessor(name), maxHeightLimit(maxHeightLimit) {}

    HeightFilter(SeedHelper::SeedType _runOnSeed, const std::string &name = "HeightFilter", bool maxHeightLimit = false)
        : CloudProcessor(name), _runOnSeed(_runOnSeed), maxHeightLimit(maxHeightLimit) {}

protected:
    PointCloudPtr process() override;
    SeedHelper::SeedType _runOnSeed = SeedHelper::NONE;
    bool maxHeightLimit = false; 
};
} // railroad

#endif //RAILROAD_HEIGHTFILTER_H
