/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef RAILROAD_CABLEDISTANCEFILTER_H
#define RAILROAD_CABLEDISTANCEFILTER_H


#include "../base/CloudProcessor.h"

#include "../base/ErrorProcessor.h"

namespace railroad
{
class CableDistanceFilter: public ErrorProcessor
{
public:
    CableDistanceFilter(const std::string &name = "CableDistanceFilter", float maxDistance = 5.7, float maxCableWidth = 0.4)
        : ErrorProcessor(name, true), _maxDistance(maxDistance), _maxCableWidth(maxCableWidth) {}
    
protected:
    PointCloudPtr getCableCloud(pcl::PointXYZ middle, PointCloudPtr rotatedCloud);
       
    PointCloudPtr process() override;
    float _maxDistance;
    float _maxCableWidth;
};
}



#endif //RAILROAD_CABLEDISTANCEFILTER_H
