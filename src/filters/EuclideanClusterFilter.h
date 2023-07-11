/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_EUCLIDEANCLUSTERFILTER_H
#define RAILROAD_EUCLIDEANCLUSTERFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{

class EuclideanClusterFilter : public CloudProcessor
{
public:
    EuclideanClusterFilter(double clusterTolerance = 0.1, double minClusterSize = 10, double maxClusterSize = 2000, const std::string &name = "EucledianClusterFilter")
        : CloudProcessor(name), clusterTolerance(clusterTolerance), minClusterSize(minClusterSize), maxClusterSize(maxClusterSize) {}

protected:
    double clusterTolerance;
    double minClusterSize;    
    double maxClusterSize;    
    PointCloudPtr process();
};
} // railroad

#endif //RAILROAD_EUCLIDEANCLUSTERFILTER_H
