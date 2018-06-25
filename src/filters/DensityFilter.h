/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_DENSITYFILTER_H
#define RAILROAD_DENSITYFILTER_H

#include <opencv2/core/core.hpp>

#include "../base/CloudProcessor.h"

namespace railroad
{

class DensityFilter : public CloudProcessor
{
public:
    float resolution = 0.3f;
    int threshold = 5;

    DensityFilter(const std::string &name = "DensityFilter")
        : CloudProcessor(name) {}

protected:
    PointCloudPtr process();

private:
    void dump2DMap(cv::Mat density);
};

} // railroad

#endif //RAILROAD_DENSITYFILTER_H
