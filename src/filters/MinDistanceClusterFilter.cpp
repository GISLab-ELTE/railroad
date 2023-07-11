/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include "MinDistanceClusterFilter.h"
#include "../helpers/LogHelper.h"
#include "../helpers/PCLHelper.h"

using namespace std;

namespace railroad
{

inline bool arePointsFarApart(const pcl::PointXYZ& point_a, const pcl::PointXYZ& point_b, float dist)
{
    // TODO: make this parameterizable
    if(dist > 5.0)
        return false;
    else
        return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MinDistanceClusterFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud;

    if(_runOnSeed != SeedHelper::SeedType::NONE) {
        inputCloud = seedCloud(_runOnSeed);
        LOG(debug) << "Using seed cloud in filter";
    } else {
        inputCloud = _cloud;
        LOG(debug) << "Using input cloud in filter";
    }

    pcl::ConditionalEuclideanClustering<pcl::PointXYZ> cec(false);
    cec.setInputCloud (inputCloud);

    startTimeMeasure();

    cec.setConditionFunction(&arePointsFarApart);
    cec.setClusterTolerance(500.0);
    cec.setMinClusterSize(30);
    cec.setMaxClusterSize(5000);
    cec.segment(*clusters);

    uint clusterCount = clusters->size();//1;

    LOG(debug) << "Number of clusters: " << clusterCount;

    for(uint clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++) {
        uint pointCount = (*clusters)[clusterIndex].indices.size();
        LOG(debug) << "Number of points in cluster " << clusterIndex+1 << ": " << pointCount;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for(uint idx = 0; idx < pointCount; ++idx) {
            pcl::PointXYZ point = (*inputCloud)[(*clusters)[clusterIndex].indices[idx]];
            cluster->push_back(point);
        }

        pcl::PointXYZ centroidPoint = getCentroid(cluster);

        cloud->push_back(centroidPoint);
    }

    //stopTimeMeasure();

    return cloud;
}

}  // railroad

