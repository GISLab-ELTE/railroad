/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_CYLINDERFINDER_H
#define RAILROAD_CYLINDERFINDER_H

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class CylinderFilter : public SingleResultCloudProcessor
{
public:
    CylinderFilter(double normalDistanceWeight = 0.5, double distanceThreshold = 2, double maximumRadius = 2,
                   const std::string &name = "CylinderFilter")
        : SingleResultCloudProcessor(name), normalDistanceWeight(normalDistanceWeight), distanceThreshold(distanceThreshold),
          maximumRadius(maximumRadius) {}

protected:
    PointCloudPtr process();

    double normalDistanceWeight;
    double distanceThreshold;
    double maximumRadius;

    pcl::PointCloud<pcl::PointXYZ>::Ptr findCylinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
};

} // railroad

#endif //RAILROAD_CYLINDERFINDER_H
