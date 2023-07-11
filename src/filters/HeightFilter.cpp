/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>

#include "HeightFilter.h"
#include "../helpers/LogHelper.h"

namespace railroad
{
pcl::PointCloud<pcl::PointXYZ>::Ptr HeightFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr seedCloudToUse;

    stopTimeMeasure();

    if(_runOnSeed != SeedHelper::SeedType::NONE) {
        seedCloudToUse = seedCloud(_runOnSeed);
        LOG(debug) << "Using seed cloud specified in Pipes.h in filter";
    } else {
        LOG(error) << "Seed type not specified in Pipes.h for HeightFilter";
    }

    startTimeMeasure();

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*seedCloudToUse, minPt, maxPt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double min = minPt.z - 1.0;
    double max = maxPt.z + 1.0;

    for (pcl::PointXYZ point : *_cloud) {
        if(maxHeightLimit) {
            if (point.z >= min && point.z <= max)
                cloud->push_back(point);
        } else {
            if (point.z >= min)
                cloud->push_back(point);
        }
    }

    return cloud;
}
}  // railroad

