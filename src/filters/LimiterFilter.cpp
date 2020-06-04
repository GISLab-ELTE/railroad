/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <algorithm>

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

#include "LimiterFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

class Interval
{
public:
    float minVal;
    float maxVal;

    int counter;

    bool isIn(float val)
    {
        return minVal <= val && val <= maxVal;
    }

    bool isInWithPadding(float val, float padding)
    {
        return minVal - padding <= val && val <= maxVal + padding;
    }

    bool isIntersect(Interval *other)
    {
        return isIn(other->minVal) || isIn(other->maxVal) || other->isIn(minVal) || other->isIn(maxVal);
    }

    void annex(Interval *other)
    {
        minVal = min(minVal, other->minVal);
        maxVal = max(maxVal, other->maxVal);
        counter += other->counter;
        other->minVal = 0;
        other->maxVal = 0;
        other->counter = 0;
    }

    bool isEmpty()
    {
        return counter == 0;
    }
};


ostream &operator<<(ostream &out, const Interval &obj)
{
    return out << obj.minVal << " " << obj.maxVal << " " << obj.counter;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr LimiterFilter::process()
{
    float maxDistance = 0.1;
    vector<Interval> intervals;

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*_cloud, min, max);
    LOG(debug) << min << " " << max;
    for (auto point : _cloud->points) {
        bool found = false;
        for (unsigned int a = 0; a < intervals.size() && !found; a++) {
            Interval *actInterval = &intervals[a];
            if (actInterval->isInWithPadding(point.z, maxDistance)) {
                actInterval->counter++;

                if (point.z < actInterval->minVal) {
                    actInterval->minVal = point.z;
                }
                if (actInterval->maxVal < point.z) {
                    actInterval->maxVal = point.z;
                }

                for (unsigned int b = 0; b < intervals.size(); b++) {
                    Interval *otherInterval = &intervals[b];
                    if (a != b && actInterval->isIntersect(otherInterval)) {
                        actInterval->annex(otherInterval);
                    }
                }
                for (auto i = std::begin(intervals); i != std::end(intervals);) {
                    if (i->isEmpty()) {
                        i = intervals.erase(i);
                    } else {
                        ++i;
                    }
                }
                found = true;
            }
        }
        if (!found) {
            Interval newInterval;
            newInterval.minVal = newInterval.maxVal = point.z;
            newInterval.counter = 1;

            intervals.push_back(newInterval);
        }
    }

    int threshold = (int) round(_cloud->points.size() * 0.001);

    Interval finalInterval;
    finalInterval.minVal = 0;
    finalInterval.maxVal = 0;

    for (auto &interval : intervals) {
        if (interval.counter > threshold) {
            finalInterval.annex(&interval);
        }
    }
    LOG(debug) << "Final interval for valid z values: " << finalInterval;


    pcl::PointCloud<pcl::PointXYZ>::Ptr withoutOutliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(finalInterval.minVal, finalInterval.maxVal);
    pass.filter(*withoutOutliers);

    return withoutOutliers;
}

} // railroad
