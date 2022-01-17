/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_RANSACFILTER_H
#define RAILROAD_RANSACFILTER_H

#include <utility>

#include <opencv2/core.hpp>

#include "../base/CloudProcessor.h"

namespace railroad
{
class RansacFilter : public CloudProcessor
{
public:
    RansacFilter(const std::string &name = "RansacFilter", int ransacMinSize = 6000)
        : CloudProcessor(name), _ransacMinSize(ransacMinSize) {}

protected:
    PointCloudPtr process() override;

private:
    static void compute(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &temp,
                        const Eigen::Vector3f &axis, double threshold, Eigen::VectorXf &coeff);
    int _ransacMinSize;
};
}  // railroad

#endif //RAILROAD_RANSACFILTER_H
