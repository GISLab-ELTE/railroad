/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <algorithm>
#include <cmath>

#include <pcl/common/io.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include "PCLHelper.h"

using namespace boost::accumulators;

std::ostream &operator<<(std::ostream &out, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (auto &point : cloud->points) {
        out << std::fixed
            << point.x << " "
            << point.y << " "
            << point.z << std::endl;
    }
    return out;
}

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr diffPointClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr original,
    pcl::PointCloud<pcl::PointXYZ>::Ptr remover)
{

    // Sort point cloud by X, Y, Z,.
    auto comp = [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
        if (a.x == b.x)
            if (a.y == b.y)
                return a.z < b.z;
            else return a.y < b.y;
        else return a.x < b.x;
    };
    std::sort(original->begin(), original->end(), comp);
    std::sort(remover->begin(), remover->end(), comp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    std::set_difference(
        original->begin(), original->end(),
        remover->begin(), remover->end(),
        std::inserter(result->points, result->points.begin()),
        comp);
    result->width = static_cast<uint32_t>(result->points.size());

    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr mergePointClouds(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr result)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*result, *target);

    // Concatenate input cloud
    *target += *input;

    // Sort point cloud by X, Y, Z
    std::sort(target->begin(), target->end(),
              [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
                  if (a.x == b.x)
                      if (a.y == b.y)
                          return a.z < b.z;
                      else return a.y < b.y;
                  else return a.x < b.x;
              });

    // Remove duplicated points (first occurrence will remain)
    target->erase(
        std::unique(target->begin(), target->end(),
                    [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
                        return a.x == b.x && a.y == b.y && a.z == b.z;
                    }),
        target->end());

    return target;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr mergePointClouds(
    pcl::PointCloud<pcl::PointXYZL>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZL>::ConstPtr result)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr target(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud(*result, *target);

    // Concatenate input cloud
    *target += *input;

    // Sort point cloud by X, Y, Z, then I (higher label first)
    std::sort(target->begin(), target->end(),
              [](const pcl::PointXYZL &a, const pcl::PointXYZL &b) {
                  if (a.x == b.x)
                      if (a.y == b.y)
                          if (a.z == b.z) return a.label > b.label;
                          else return a.z < b.z;
                      else return a.y < b.y;
                  else return a.x < b.x;
              });

    // Remove duplicated points (first occurrences with higher label will remain)
    target->erase(
        std::unique(target->begin(), target->end(),
                    [](const pcl::PointXYZL &a, const pcl::PointXYZL &b) {
                        return a.x == b.x && a.y == b.y && a.z == b.z;
                    }),
        target->end());

    return target;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr mergePointCloudsVisual(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr result,
    LASClass classification)
{
    // Add label to input point cloud
    pcl::PointCloud<pcl::PointXYZL>::Ptr source(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud(*input, *source);

    return mergePointCloudsVisual(source, result, classification);
}

pcl::PointCloud<pcl::PointXYZL>::Ptr mergePointCloudsVisual(
    pcl::PointCloud<pcl::PointXYZL>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr result,
    LASClass classification)
{
    // Add label to result point cloud
    pcl::PointCloud<pcl::PointXYZL>::Ptr target(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud(*result, *target);

    // Set label on result cloud
    for (auto it = target->begin(); it != target->end(); ++it) {
        it->label = static_cast<uint32_t>(classification);
    }

    // Concatenate input cloud
    *target += *input;

    // Sort point cloud by X, Y, Z, then I (higher label first)
    std::sort(target->begin(), target->end(),
              [](const pcl::PointXYZL &a, const pcl::PointXYZL &b) {
                  if (a.x == b.x)
                      if (a.y == b.y)
                          if (a.z == b.z) return a.label > b.label;
                          else return a.z < b.z;
                      else return a.y < b.y;
                  else return a.x < b.x;
              });

    // Remove duplicated points (first occurrences with higher label will remain)
    target->erase(
        std::unique(target->begin(), target->end(),
                    [](const pcl::PointXYZL &a, const pcl::PointXYZL &b) {
                        return a.x == b.x && a.y == b.y && a.z == b.z;
                    }),
        target->end());

    return target;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr diffPointClouds(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr original,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr remover,
    int precision)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<pcl::PointXYZ> originalPoints(original->begin(), original->end());
    std::vector<pcl::PointXYZ> removerPoints(remover->begin(), remover->end());

    float multiplier = std::pow(10, precision);

    for (auto &p : originalPoints) {
        p.x = std::round(p.x * multiplier) / multiplier ;
        p.y = std::round(p.y * multiplier) / multiplier;
        p.z = std::round(p.z * multiplier) / multiplier;
    }

    for (auto &p : removerPoints) {
        p.x = std::round(p.x * multiplier) / multiplier;
        p.y = std::round(p.y * multiplier) / multiplier;
        p.z = std::round(p.z * multiplier) / multiplier;
    }

    // Sort point clouds by X, Y, Z
    auto comp = [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
        if (a.x == b.x)
            if (a.y == b.y)
                return a.z < b.z;
            else return a.y < b.y;
        else return a.x < b.x;
    };
    std::sort(originalPoints.begin(), originalPoints.end(), comp);
    std::sort(removerPoints.begin(), removerPoints.end(), comp);

    // Diff point clouds
    std::set_difference(
        originalPoints.begin(), originalPoints.end(),
        removerPoints.begin(), removerPoints.end(),
        std::inserter(diffCloud->points, diffCloud->points.begin()),
        comp);


    diffCloud->width = static_cast<uint32_t>(diffCloud->points.size());
    return diffCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cutVerticalCylinder(
    pcl::PointXYZ centre,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
    double radius,
    float minZ,
    float maxZ)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr withinCylinderPoints(new pcl::PointCloud<pcl::PointXYZ>);    

    for(uint i = 0; i < input->points.size(); ++i) {
        pcl::PointXYZ point = input->points.at(i);

        double dist = pcl::euclideanDistance(centre, pcl::PointXYZ(point.x, point.y, centre.z));

        if(dist < radius && point.z < maxZ && point.z > minZ) {
            withinCylinderPoints->points.push_back(point);
        }
    }

    return withinCylinderPoints;    
}

pcl::PointXYZ getCentroid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input)
{
    pcl::PointXYZ minPt, maxPt;

    pcl::getMinMax3D(*input, minPt, maxPt);

    accumulator_set<double, features<tag::mean>> xmean;
    xmean(minPt.x);
    xmean(maxPt.x);
    accumulator_set<double, features<tag::mean>> ymean;
    ymean(minPt.y);
    ymean(maxPt.y);
    accumulator_set<double, features<tag::mean>> zmean;
    zmean(minPt.z);
    zmean(maxPt.z);

    return pcl::PointXYZ(mean(xmean), mean(ymean), mean(zmean));
}

Eigen::Vector3f getFirstEigenVector(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input)
{
    pcl::PCA<pcl::PointXYZ> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(input);

    Eigen::Vector3f vector = pca.getEigenVectors().col(0);

    return vector;
}

} // railroad
