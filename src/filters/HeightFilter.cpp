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

namespace railroad
{
pcl::PointCloud<pcl::PointXYZ>::Ptr HeightFilter::process()
{
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*_seedCloud, minPt, maxPt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double min = minPt.z - 1.0;

    for (pcl::PointXYZ point : *_cloud) {
        if (point.z >= min)
            cloud->push_back(point);
    }

    return cloud;
}
}  // railroad

