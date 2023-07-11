/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include "CorrigateCentroidsFilter.h"
#include "../helpers/LogHelper.h"
#include "../helpers/PCLHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr CorrigateCentroidsFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    startTimeMeasure();
    uint centroidCount = _cloud->points.size();
    
    for(uint centroidIndex = 0; centroidIndex < centroidCount; ++centroidIndex)  {
        pcl::PointXYZ centroidPoint = _cloud->points.at(centroidIndex);

        pcl::PointCloud<pcl::PointXYZ>::Ptr sample = cutVerticalCylinder(centroidPoint, _baseCloud,
         sampleRadius, centroidPoint.z - 0.1, centroidPoint.z + 0.1);  

        pcl::PointXYZ corrigatedCentroid = getCentroid(sample);

        outputCloud->points.push_back(corrigatedCentroid);
    }

    //stopTimeMeasure();

    return outputCloud;
}

}  // railroad

