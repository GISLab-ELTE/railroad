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


#include "../base/CloudProcessor.h"

namespace railroad
{
class WidthFilter : public CloudProcessor
{
public:
    WidthFilter(float maxDistance = 1.2, const std::string &name = "WidthFilter")
        : CloudProcessor(name, true), _maxDistance(maxDistance) {}

protected:
    PointCloudPtr process() override;
    float _maxDistance;
};
} // railroad


#endif //RAILROAD_WIDTHFILTER_H
