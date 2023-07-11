/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include "VegetationDetectionFilter.h"


namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr
VegetationDetectionFilter::getVegetationCloud(pcl::PointXYZ middle, pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    double xmin = middle.x - _trackWidth + 0.05;
    double xmax = middle.x + _trackWidth - 0.15;
    double zmax = middle.z + _minDistance + 0.05;
    for (unsigned int i = 0; i < rotatedCloud->size(); i++) {
        pcl::PointXYZ point = rotatedCloud->at(i);
        if (point.z >= zmax && point.z <= zmax + 0.5 &&
            ((point.x <= xmax && point.x >= xmin) ||
             ((point.x >= xmax + 0.30 && point.x <= xmax + 1) ||
              (point.x <= xmin - 0.30 && point.x >= xmin - 1))
            ))
            cloud->push_back(_cloud->at(i));
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VegetationDetectionFilter::process()
{
    double angle = getRansac();
    pcl::PointXYZ middle = getMiddlePoint(getRotatedSeedCloud(angle));
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud = getRotatedCloud(angle);
    return getVegetationCloud(middle, rotatedCloud);
}
} // railroad