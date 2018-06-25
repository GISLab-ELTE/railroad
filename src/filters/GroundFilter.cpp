/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <algorithm>
#include <numeric>

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

#include "GroundFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr GroundFilter::process()
{
    LOG(debug) << "Threshold: " << threshold;

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*_cloud, min, max);

    double average = 0;
    auto sizeForDivide = (double) _cloud->points.size();
    for (auto &point : _cloud->points) {
        average += point.z / sizeForDivide;
    }
    LOG(trace) << "min:" << min.z << ", avg:" << average << ", max:" << max.z;


    pcl::PointCloud<pcl::PointXYZ>::Ptr noGround(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(average, max.z);
    pass.filter(*noGround);

    return noGround;
}

} // railroad
