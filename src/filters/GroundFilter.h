/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_GROUNDFILTER_H
#define RAILROAD_GROUNDFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{

class GroundFilter : public CloudProcessor
{
public:
    float threshold = 5.0f;

    GroundFilter(const std::string &name = "GroundFilter")
        : CloudProcessor(name) {}

protected:
    PointCloudPtr process();
};

} // railroad

#endif //RAILROAD_GROUNDFILTER_H
