/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_HOUGH3DFILTER_H
#define RAILROAD_HOUGH3DFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{
class Hough3dFilter : public CloudProcessor
{
public:
    Hough3dFilter(const std::string &name = "Hough3dFilter", int houghLineCount = 5)
        : CloudProcessor(name), _houghLineCount(houghLineCount) {}

protected:
    PointCloudPtr process() override;

private:
    static constexpr const int num_directions[7] = {12, 21, 81, 321, 1281, 5121, 20481};


    static void pointsCloseToLine(const Eigen::Vector3f &a, const Eigen::Vector3f &b, float dx,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &Y,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &indices);
    static float orthogonal_LSQ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, Eigen::Vector3f *a, Eigen::Vector3f *b);
    static Eigen::Vector3f meanValue(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);
    int _houghLineCount;
};
} // railroad

#endif //RAILROAD_HOUGH3DFILTER_H
