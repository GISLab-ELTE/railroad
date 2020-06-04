/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "CylinderFilter.h"
#include "../helpers/LogHelper.h"
#include "../helpers/PCLHelper.h"
#include "../dim2/Projection.h"

namespace railroad
{

typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr CylinderFilter::findCylinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    //http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(
        new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    LOG(debug) << "Plane coefficients: " << *coefficients_plane;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(normalDistanceWeight);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setRadiusLimits(0, maximumRadius);
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    LOG(debug) << "Cylinder coefficients: " << *coefficients_cylinder;

    // Write the cylinder inliers to disk
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);

    return cloud_cylinder;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CylinderFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*_cloud, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (int a = 0; a < 2; a++) {
        pcl::PointCloud<PointT>::Ptr cloud_cylinder = findCylinder(cloud);
        Projection projectionObj(cloud_cylinder, 0.25);
        projectionObj.saveToFile("cylinder" + std::to_string(a) + ".png");

        *result += *cloud_cylinder;

        cloud = diffPointClouds(cloud, cloud_cylinder);
        Projection projectionObj2(cloud, 0.25);
        projectionObj.saveToFile("afterCylinder" + std::to_string(a) + ".png");
    }
    return result;
}

} // railroad
