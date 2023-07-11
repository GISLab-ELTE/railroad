/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <cmath>
#include <set>

#include <pcl/common/common.h>
#include <pcl/search/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "CantileverFilter.h"
#include "../helpers/LogHelper.h"
#include "../helpers/PCLHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr CantileverFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*_cloud, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cableSeed = seedCloud(SeedHelper::SeedType::CABLE);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr poleSeed = seedCloud(SeedHelper::SeedType::POLE);

    startTimeMeasure();

    LOG(debug) << "Removing nearby points of poles";
    cloud = removeNearbyPoints(poleSeed, cloud, 0.08);

    LOG(info) << "Done removing pole points";

    LOG(debug) << "Removing nearby points of catenary cables";
    cloud = removeNearbyPoints(cableSeed, cloud, 0.05);

    LOG(info) << "Done removing cable points";

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.35, 0.35, 0.35);
    voxel_grid.filter(*voxel);

    LOG(info) << "Done constructing voxel grid";

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(voxel);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    for (const pcl::PointIndices& indices : cluster_indices) {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for(const auto& idx : indices.indices) {
            cluster->push_back((*cloud)[idx]);
        }

        if(cluster->points.size() > 2000) {

            Eigen::Vector3f vector = getFirstEigenVector(cluster);

            LOG(info) << "Vector: " << vector[0] << " " << vector[1] << " " << vector[2] << " cluster size: " << cluster->points.size();

            if(std::abs(vector[0]) > 0.8)
                *output += *cluster;
        }
    }

    return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CantileverFilter::removeNearbyPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr seed, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr returnCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::set<uint> indexesToRemove = std::set<uint>();

    float resolution = 128.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    for (auto seedPoint : seed->points)
    {
        pointIdxRadiusSearch.empty();
        pointRadiusSquaredDistance.empty();
        octree.radiusSearch(seedPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        for(auto idx : pointIdxRadiusSearch)
        {
            indexesToRemove.insert(idx);
        }
    }        

    pcl::PointIndices::Ptr remover(new pcl::PointIndices());
    remover->indices = std::vector<int>(indexesToRemove.begin(), indexesToRemove.end());

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(remover);
    extract.setNegative(true);
    extract.filter(*returnCloud);

    return returnCloud;
}

}  // railroad

