/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_GROWTHFILTER_H
#define RAILROAD_GROWTHFILTER_H

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{
class GrowthFilter : public SingleResultCloudProcessor
{
public:
    GrowthFilter(SeedHelper::SeedType _runOnSeed, const std::string &name = "GrowthFilter", float seedBoxLength = 0.4f, int seedGridCount = 4, int seedMaxNumberOfPoints = 110)
        : SingleResultCloudProcessor(name), _runOnSeed(_runOnSeed), _seedBoxLength(seedBoxLength), _seedGridCount(seedGridCount), _seedMaxNumberOfPoints(seedMaxNumberOfPoints) {}

protected:
    PointCloudPtr process() override;
    SeedHelper::SeedType _runOnSeed = SeedHelper::NONE;

private:
    struct BoundingBox
    {
        BoundingBox(pcl::PointXYZ center, float size) : center(center), size(size)
        {
            max = pcl::PointXYZ(center.x + size, center.y + size, center.z + size);
            min = pcl::PointXYZ(center.x - size, center.y - size, center.z - size);
        }

        pcl::PointXYZ center;
        float size;
        pcl::PointXYZ max;
        pcl::PointXYZ min;

        bool containsPoint(pcl::PointXYZ point)
        {
            return point.x > min.x && point.y > min.y && point.z > min.z &&
                   point.x < max.x && point.y < max.y && point.z < max.z;

        }
    };

    pcl::PointXYZ calcSeed(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, size_t length);
    void findCable(pcl::PointXYZ s, float boxLength, std::vector<int>& insideBox, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& cable, float skip);
    float _seedBoxLength;
    int _seedGridCount;
    std::size_t _seedMaxNumberOfPoints;
};
}  // railroad

#endif //RAILROAD_GROWTHFILTER_H