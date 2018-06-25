/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/filters/radius_outlier_removal.h>

#include "OutlierFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr OutlierFilter::process()
{
    LOG(debug) << "Radius: " << radius << ", Neighbors: " << minNeighbors;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlierRemover;
    outlierRemover.setInputCloud(_cloud);
    outlierRemover.setRadiusSearch(radius);
    outlierRemover.setMinNeighborsInRadius(minNeighbors);
    outlierRemover.filter(*filtered);

    return filtered;
}

} // railroad
