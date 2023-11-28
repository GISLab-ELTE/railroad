/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_WIDTHFILTER_H
#define RAILROAD_WIDTHFILTER_H


#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{
class WidthFilter : public SingleResultCloudProcessor
{
public:
    WidthFilter(float maxDistance = 1.2, const std::string &name = "WidthFilter")
        : SingleResultCloudProcessor(name), _maxDistance(maxDistance) {}

    WidthFilter(SeedHelper::SeedType _runOnSeed, float maxDistance = 1.2, const std::string &name = "WidthFilter")
        : SingleResultCloudProcessor(name), _maxDistance(maxDistance), _runOnSeed(_runOnSeed) {}

protected:
    PointCloudPtr process() override;
    float _maxDistance;
    SeedHelper::SeedType _runOnSeed = SeedHelper::SeedType::NONE;
};
} // railroad


#endif //RAILROAD_WIDTHFILTER_H
