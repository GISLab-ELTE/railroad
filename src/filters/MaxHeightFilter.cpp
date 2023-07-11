/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>

#include "MaxHeightFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr MaxHeightFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud;

    if(_runOnSeed != SeedHelper::SeedType::NONE) {
        inputCloud = seedCloud(_runOnSeed);
        LOG(debug) << "Using seed cloud in filter";
    } else {
        inputCloud = _cloud;
        LOG(debug) << "Using input cloud in filter";
    }

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double max = maxPt.z - offset;

    if(keepBelow)
        LOG(debug) << "Keeping only points below Z value of: " << max;
    else
        LOG(debug) << "Keeping only points above Z value of: " << max;

    for (pcl::PointXYZ point : *inputCloud) {
        if (keepBelow && point.z <= max) {            
            cloud->push_back(point);
        } else if (!keepBelow && point.z >= max) {            
            cloud->push_back(point);
        }
    }

    return cloud;
}
}  // railroad

