/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <cmath>

#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/angles.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/core.hpp>

#include "RansacCylinderFilter.h"
#include "../helpers/LogHelper.h"
#include "../helpers/PCLHelper.h"

using namespace std;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr RansacCylinderFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    uint poleCount = _cloud->points.size();
    uint detectedPoleCount = poleCount;

    startTimeMeasure();
    
    for(uint poleIndex = 0; poleIndex < poleCount; ++poleIndex)  {
        pcl::PointXYZ seedPoint = _cloud->points[poleIndex];

        LOG(info) << "Finding mast points with centre: " << seedPoint;

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = cutVerticalCylinder(seedPoint, _baseCloud,
         POLE_CENTRE_SEARCH_RAD, seedPoint.z - 5, seedPoint.z + 8);  

        LOG(debug) << "Candidate points for mast " << poleIndex+1 << " before RANSAC: " << input_cloud->points.size();         

        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*input_cloud, minPoint, maxPoint);

        float height = maxPoint.z - minPoint.z;        

        if(height < 8) {
            LOG(info) << "Mast candidate not tall enough, skipping.";
            LOG(info) << "------------------------------------------------------------------";
            --detectedPoleCount;
            continue;
        }       


        pcl::PointCloud<pcl::PointXYZ>::Ptr mastPoints(new pcl::PointCloud<pcl::PointXYZ>);      

        pcl::PointCloud<pcl::PointXYZ>::Ptr ransacCylinder(new pcl::PointCloud<pcl::PointXYZ>);

        LOG(debug) << "Estimating normals";
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(input_cloud);
        ne.setKSearch(6);
        ne.compute(*normals);

        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(_normalDistanceWeight);
        seg.setMaxIterations(20000);
        seg.setDistanceThreshold(0.09);
        seg.setRadiusLimits(0.1, 0.17);            
        seg.setAxis(Eigen::Vector3f::UnitZ());        
        seg.setEpsAngle(pcl::deg2rad(5.f));
        seg.setInputCloud(input_cloud);
        seg.setInputNormals(normals); 

        seg.segment(*inliers_cylinder, *coefficients_cylinder);        

        // Fall back to non-cylindrical mast point segmentation if cylinder RANSAC doesn't give enough inliers
        if(inliers_cylinder->indices.size() < RansacCylinderFilter::MIN_POLE_POINTCOUNT) {
            LOG(info) << "Not enough points fit RANSAC model for pole " << poleIndex+1 <<".("<< inliers_cylinder->indices.size() <<")";
            LOG(info) << "Trying to segment non-cylindrical mast points.";
            mastPoints = input_cloud;
            appendPoints(output_cloud, mastPoints);
            LOG(info) << "Points for mast " << poleIndex+1 << ". (non-cylindrical): " << mastPoints->points.size();
            LOG(info) << "------------------------------------------------------------------";
            continue;
        }

        LOG(info) << "Points for mast " << poleIndex+1 << ". (cylindrical) after RANSAC cylinder fitting: " << inliers_cylinder->indices.size();
        LOG(debug) << "Cylinder RANSAC coefficients: " << *coefficients_cylinder;

        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers_cylinder);
        extract.filter(*ransacCylinder);

        Eigen::Vector3f cylinderAxis;
        cylinderAxis[0] = roundToTwoDecimals(coefficients_cylinder->values[3]);
        cylinderAxis[1] = roundToTwoDecimals(coefficients_cylinder->values[4]);
        cylinderAxis[2] = roundToTwoDecimals(std::abs(coefficients_cylinder->values[5]));
        cylinderAxis.normalize();

        double radius = coefficients_cylinder->values[6] + 0.0976;

        LOG(info) << "Orientation of fitted cylinder: " << pcl::PointXYZ(cylinderAxis.x(), cylinderAxis.y(), cylinderAxis.z());

        pcl::PointXYZ mastCentreRansac;
        mastCentreRansac.x = coefficients_cylinder->values[0];
        mastCentreRansac.y = coefficients_cylinder->values[1];
        mastCentreRansac.z = 0;

        LOG(info) << "Centre of fitted cylinder (X, Y): " << mastCentreRansac.x << ", " << mastCentreRansac.y;
        LOG(info) << "Using radius of " << radius;

        for(uint i = 0; i < input_cloud->points.size(); ++i) {
            pcl::PointXYZ point = input_cloud->points.at(i);

            if(point.z < maxPoint.z && point.z > minPoint.z) {
                pcl::PointXYZ circleCenter = calcCircleCentreFromModel(mastCentreRansac, point.z, cylinderAxis);

                double dist = pcl::euclideanDistance(circleCenter, point);

                if(dist < radius) {
                    mastPoints->points.push_back(point);                       
                    output_cloud->points.push_back(point);
                }
            }
        }

        LOG(info) << "Points for mast " << poleIndex+1 << ". (cylindrical) after enriching RANSAC cylinder: " << mastPoints->points.size();
        LOG(info) << "------------------------------------------------------------------";
    }

    LOG(info) << "Detected " << detectedPoleCount << " masts in total out of " << poleCount << " candidates";    
    
    return output_cloud;
}

pcl::PointXYZ RansacCylinderFilter::calcCircleCentreFromModel(pcl::PointXYZ cylinderCentre, float pointZ, Eigen::Vector3f cylinderAxis) {
    float heightDifference = pointZ - cylinderCentre.z;
    Eigen::Vector3f transformed = cylinderCentre.getVector3fMap() + (cylinderAxis * heightDifference);
    pcl::PointXYZ centre;    
    centre.x = transformed[0];
    centre.y = transformed[1];
    centre.z = transformed[2];
    return centre;
}

float RansacCylinderFilter::roundToTwoDecimals(float num) {
    return round(num * 100.0) / 100.0;
}

void RansacCylinderFilter::appendPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr original, pcl::PointCloud<pcl::PointXYZ>::Ptr append)
{
    for(uint i = 0; i < append->points.size(); ++i) {
        pcl::PointXYZ point = append->points.at(i);
        original->points.push_back(point);
    }
}

} // railroad

