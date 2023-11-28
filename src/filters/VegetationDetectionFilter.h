/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_VEGETATIONDETECTIONFILTER_H
#define RAILROAD_VEGETATIONDETECTIONFILTER_H


#include "../base/SingleResultCloudProcessor.h"
#include "../base/ErrorProcessor.h"

namespace railroad
{
class VegetationDetectionFilter: public ErrorProcessor
{
public:
    VegetationDetectionFilter(const std::string &name = "VegetationDeFilter",  float trackWidth = 1.435/2)
        : ErrorProcessor(SeedHelper::RAIL, name), _trackWidth(trackWidth) {}

protected:
    PointCloudPtr getVegetationCloud(pcl::PointXYZ middle, PointCloudPtr rotatedCloud);
    PointCloudPtr process() override;
    float _minDistance=0;
    float _trackWidth;
};
}  // railroad

#endif //RAILROAD_VEGETATIONDETECTIONFILTER_H