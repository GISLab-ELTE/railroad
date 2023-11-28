/*
 * BSD 3-Clause License
 * Copyright (c) 2018-2023, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_CLOUDPROCESSOR_H
#define RAILROAD_CLOUDPROCESSOR_H

#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "../helpers/SeedHelper.h"

namespace railroad
{

class CloudProcessor
{
public:
    typedef typename pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;

    PointCloudConstPtr baseCloud()
    {
        return _baseCloud;
    }

    void setBaseCloud(const PointCloudConstPtr &baseCloud)
    {
        _baseCloud = baseCloud;
    }

    PointCloudConstPtr inputCloud()
    {
        return _cloud;
    }

    void setInputCloud(const PointCloudConstPtr &cloud)
    {
        _cloud = cloud;
    }

    PointCloudConstPtr seedCloud(SeedHelper::SeedType seedType)
    {
        return _seedHelper.getSeedCloud(seedType);
    }

    void setSeedHelper(const SeedHelper &seedHelper)
    {
        _seedHelper = seedHelper;
    }

    std::string name()
    {
        return _name;
    }

    void startTimeMeasure()
    {
        if (_isClockRunning) {
            stopTimeMeasure();
        }
        _startClock = std::clock();
        _isClockRunning = true;
    }

    void stopTimeMeasure()
    {
        if (_isClockRunning) {
            _measuredTime += (double) (std::clock() - _startClock) / CLOCKS_PER_SEC;
            _isClockRunning = false;
        }
    }

    double getMeasuredTime() const
    {
        return _measuredTime;
    }

    int getNumberOfFilteredPoints() const
    {
        return _numberOfFilteredPoints;
    }

protected:
    std::string _name;
    PointCloudConstPtr _cloud;
    PointCloudConstPtr _baseCloud;
    SeedHelper _seedHelper;
    unsigned int _numberOfFilteredPoints;

    bool _isClockRunning;
    double _measuredTime;
    std::clock_t _startClock;

    CloudProcessor(const std::string &name) :
        _name(name), _numberOfFilteredPoints(0), _isClockRunning(false), _measuredTime(0.0f)
    {}

    virtual ~CloudProcessor() {}
};

} // railroad

#endif //RAILROAD_CLOUDPROCESSOR_H
