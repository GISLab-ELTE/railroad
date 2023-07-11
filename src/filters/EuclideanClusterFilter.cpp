/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "EuclideanClusterFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr EuclideanClusterFilter::process()
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (_cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(_cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    LOG(debug) << "Cluster tolerance: " << clusterTolerance << ", Min cluster size: " 
        << minClusterSize << ", Max cluster size: " << maxClusterSize;

    for (const auto& cluster : cluster_indices) {
        int idx = cluster.indices[0];
        cloud->push_back((*_cloud)[idx]);
    }

    return cloud;
}
}  // railroad

