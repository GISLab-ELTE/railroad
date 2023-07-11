/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>

#include "BandPassFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr BandPassFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr seed;
    pcl::PointXYZ minPt, maxPt;

    stopTimeMeasure();

    if(_runOnSeed != SeedHelper::SeedType::NONE) {
        seed = seedCloud(_runOnSeed);
        pcl::getMinMax3D(*seed, minPt, maxPt);
        LOG(debug) << "Using seed cloud in filter";
    } else {
        pcl::getMinMax3D(*_cloud, minPt, maxPt);
    }

    startTimeMeasure();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double min = minPt.z;
    double max = maxPt.z;

    double midPoint = (min + max) / 2;

    if(!isnan(offset))
        midPoint = midPoint + offset;

    double passMin = midPoint - height;
    double passMax = midPoint + height;

    LOG(debug) << "Midpoint: " << midPoint << ", Pass min: " << passMin << ", Pass max: " << passMax;

    for (pcl::PointXYZ point : *_cloud) {
        if (point.z >= passMin && point.z <= passMax) {            
            cloud->push_back(point);
        }
    }

    return cloud;
}
}  // railroad

