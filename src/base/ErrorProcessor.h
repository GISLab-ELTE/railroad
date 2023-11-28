/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */


#ifndef RAILROAD_ERRORPROCESSOR_H
#define RAILROAD_ERRORPROCESSOR_H

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class ErrorProcessor: public SingleResultCloudProcessor
{
public:
    ErrorProcessor(SeedHelper::SeedType _runOnSeed, const std::string &name) : SingleResultCloudProcessor(name), _runOnSeed(_runOnSeed) {};
    virtual ~ErrorProcessor() {}
protected:
    double getAngle(const pcl::PointXYZ vector1, const pcl::PointXYZ vector2);
    pcl::PointXYZ getMiddlePoint(PointCloudPtr leftLine); 
    PointCloudPtr getRotatedSeedCloud(double angle);
    PointCloudPtr getRotatedCloud(double angle);
    PointCloudPtr separateLines();
    double getRansac();
    PointCloudConstPtr getSeedCloud();
    SeedHelper::SeedType _runOnSeed = SeedHelper::NONE;
};
}
#endif //RAILROAD_ERRORPROCESSOR_H