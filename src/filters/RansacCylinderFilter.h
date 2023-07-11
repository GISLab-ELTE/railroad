/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_RansacCylinderFilter_H
#define RAILROAD_RansacCylinderFilter_H

#include <string>

#include "../base/CloudProcessor.h"

namespace railroad
{

class RansacCylinderFilter : public CloudProcessor
{
public:
    RansacCylinderFilter(double normalDistanceWeight = 0.22,const std::string &name = "RansacCylinderFilter")
        : CloudProcessor(name), _normalDistanceWeight(normalDistanceWeight) {}

protected:
    PointCloudPtr process();
    double _normalDistanceWeight;

private:
    pcl::PointXYZ calcCircleCentreFromModel(pcl::PointXYZ mean, float pointZ, Eigen::Vector3f cylinderAxis);
    float roundToTwoDecimals(float num);
    void appendPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr original, pcl::PointCloud<pcl::PointXYZ>::Ptr append);
    static const uint MIN_POLE_POINTCOUNT = 450;
    static constexpr double POLE_CENTRE_SEARCH_RAD = 0.404;    

};

} // railroad

#endif // RAILROAD_RansacCylinderFilter_H
