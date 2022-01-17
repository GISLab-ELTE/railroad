/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <opencv2/core.hpp>

#include "CutFilter.h"
#include "../helpers/LogHelper.h"
#include "../dim2/Projection.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr CutFilter::process()
{
    Projection projectionObj(_cloud, 0.25);
    cv::Mat projection = projectionObj.getImage();

    ImportantPartFinderProcessor finder(mode);
    finder.setInputImage(projection);
    cv::Mat areaToKeep = finder.execute();

    pcl::PointCloud<pcl::PointXYZ>::Ptr result = projectionObj.grepPointCloud(_cloud, areaToKeep);

    return result;
}

} // railroad
