/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_STRUCTUREGAUGEFILTER_H
#define RAILROAD_STRUCTUREGAUGEFILTER_H

#include "../base/CloudProcessor.h"
#include "../base/ErrorProcessor.h"

namespace railroad
{
class StructureGaugeFilter: public ErrorProcessor
{
public:
    StructureGaugeFilter(const std::string &name = "StructureGaugeFilter", bool regionNational=true)
        : ErrorProcessor(SeedHelper::RAIL, name), _regionNational(regionNational) {}

protected:
    PointCloudPtr getStructureCloud(pcl::PointXYZ middle, PointCloudPtr rotatedCloud);
    PointCloudPtr process() override;
    bool _regionNational;
};
}



#endif //RAILROAD_STRUCTUREGAUGEFILTER_H
