/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>

#include "MinHeightFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr MinHeightFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud;

    stopTimeMeasure();

    if(_runOnSeed != SeedHelper::SeedType::NONE) {
        inputCloud = seedCloud(_runOnSeed);
        LOG(debug) << "Using seed cloud in filter";
    } else {
        inputCloud = _cloud;
        LOG(debug) << "Using input cloud in filter";
    }

    startTimeMeasure();

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double min = minPt.z + offset;

    if(keepAbove)
        LOG(debug) << "Keeping only points above Z value of: " << min;
    else
        LOG(debug) << "Keeping only points below Z value of: " << min;

    for (pcl::PointXYZ point : *inputCloud) {
        if (keepAbove && point.z >= min) {            
            cloud->push_back(point);
        } else if (!keepAbove && point.z <= min) {            
            cloud->push_back(point);
        }
    }

    return cloud;
}
}  // railroad

