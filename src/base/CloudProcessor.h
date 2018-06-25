/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
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

#include "../helpers/PCLHelper.h"
#include "../helpers/LogHelper.h"

namespace railroad
{

class CloudProcessor
{
public:
    typedef typename pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;

    CloudProcessor(const std::string &name) : _name(name), _numberOfFilteredPoints(0),
                                              _isClockRunning(false), _measuredTime(0.0f) {}

    virtual ~CloudProcessor() {}

    PointCloudConstPtr inputCloud()
    {
        return _cloud;
    }

    void setInputCloud(const PointCloudConstPtr &cloud)
    {
        _cloud = cloud;
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

    PointCloudPtr execute()
    {
        LOG(debug) << "[" << _name << "] Started filter";
        startTimeMeasure();
        PointCloudPtr result = process();
        stopTimeMeasure();
        _numberOfFilteredPoints = _cloud->points.size() - result->points.size();

        LOG(info) << "[" << _name << "] "
                  << "Filtered points: " << (_cloud->size() - result->size())
                  << " (" << _cloud->size() << " -> " << result->size() << ")";
        return result;
    }

protected:
    std::string _name;
    PointCloudConstPtr _cloud;
    unsigned int _numberOfFilteredPoints;

    bool _isClockRunning;
    double _measuredTime;
    std::clock_t _startClock;

    virtual PointCloudPtr process() = 0;
};

} // railroad

#endif //RAILROAD_CLOUDPROCESSOR_H
