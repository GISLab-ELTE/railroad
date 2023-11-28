/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Balázs Tábori
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <opencv2/core/core.hpp>

#include "FragmentationFilter.h"
#include "../helpers/LogHelper.h"
#include "../dim2/Projection.h"

using namespace std;

namespace railroad
{

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> FragmentationFilter::process()
{
    Projection projectionObj(_cloud, 0.25);
    cv::Mat projection = projectionObj.getImage();

    StraightTrackPartFinderProcessor finder(mode);
    finder.setInputImage(projection);
    finder.setFragmentationAngle(_fragmentationAngle);
    cv::Mat fragmentationPoints = finder.execute();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result = projectionObj.fragmentPointCloud(_cloud, fragmentationPoints);

    return result;
}

} // railroad