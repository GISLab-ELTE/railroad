/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_ABOVEFILTER_H
#define RAILROAD_ABOVEFILTER_H

#include <opencv2/core.hpp>

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class AboveFilter : public SingleResultCloudProcessor
{
public:
    AboveFilter(float gridCellSize = 0.25, const std::string &name = "AboveFilter")
        : SingleResultCloudProcessor(name), gridCellSize(gridCellSize) {}

protected:
    PointCloudPtr process();
    pcl::PointCloud<pcl::PointXYZ>::Ptr processInner(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);


    float gridCellSize;
};

} // railroad

#endif //RAILROAD_ABOVEFILTER_H
