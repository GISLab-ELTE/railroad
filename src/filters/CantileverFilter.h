/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_CANTILEVERFILTER_H
#define RAILROAD_CANTILEVERFILTER_H

#include <cmath>
#include "../base/CloudProcessor.h"

namespace railroad
{

class CantileverFilter : public CloudProcessor
{
public:
    CantileverFilter(const std::string &name = "CantileverFilter")
        : CloudProcessor(name) {}

protected:
    PointCloudPtr process();
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeNearbyPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr seed, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius);
};
} // railroad

#endif //RAILROAD_CANTILEVERFILTER_H
