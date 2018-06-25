/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_OUTLIERFILTER_H
#define RAILROAD_OUTLIERFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{

class OutlierFilter : public CloudProcessor
{
public:
    double radius = 1.0;
    int minNeighbors = 10;

    OutlierFilter(const std::string &name = "OutlierFilter")
        : CloudProcessor(name) {}

protected:
    PointCloudPtr process();
};

} // railroad

#endif //RAILROAD_OUTLIERFILTER_H
