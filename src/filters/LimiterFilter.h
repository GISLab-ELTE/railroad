/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_LIMITERFILTER_H
#define RAILROAD_LIMITERFILTER_H

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class LimiterFilter : public SingleResultCloudProcessor
{
public:
    LimiterFilter(const std::string &name = "LimiterFilter")
        : SingleResultCloudProcessor(name) {}

protected:
    PointCloudPtr process();
};

} // railroad

#endif //RAILROAD_LIMITERFILTER_H
