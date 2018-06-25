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
#include <pcl/io/pcd_io.h>

#include "PCLHelper.h"

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

    // Sort point cloud by X, Y, Z, then I (higher intensity first)
    std::sort(target->begin(), target->end(),
              [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
                  if (a.x == b.x)
                      if (a.y == b.y)
                          return a.z < b.z;
                      else return a.y < b.y;
                  else return a.x < b.x;
              });

    // Remove duplicated points (first occurence with intensity will remain)
    target->erase(
        std::unique(target->begin(), target->end(),
                    [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
                        return a.x == b.x && a.y == b.y && a.z == b.z;
                    }),
        target->end());

    return target;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr mergePointCloudsVisual(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr result,
    float resultIntensity)
{
    // Add intensity to point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*input, *source);

    pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*result, *target);

    // Set intensity on result cloud
    for (auto it = target->begin(); it != target->end(); ++it) {
        it->intensity = resultIntensity;
    }

    // Concatenate inout cloud
    *target += *source;

    // Sort point cloud by X, Y, Z, then I (higher intensity first)
    std::sort(target->begin(), target->end(),
              [](const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
                  if (a.x == b.x)
                      if (a.y == b.y)
                          if (a.z == b.z) return a.intensity > b.intensity;
                          else return a.z < b.z;
                      else return a.y < b.y;
                  else return a.x < b.x;
              });

    // Remove duplicated points (first occurrences with intensity will remain)
    target->erase(
        std::unique(target->begin(), target->end(),
                    [](const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
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
    float divisor = std::pow(10, -precision);

    for (auto &p : originalPoints) {
        p.x = std::round(p.x * multiplier) / divisor;
        p.y = std::round(p.y * multiplier) / divisor;
        p.z = std::round(p.z * multiplier) / divisor;
    }

    for (auto &p : removerPoints) {
        p.x = std::round(p.x * multiplier) / divisor;
        p.y = std::round(p.y * multiplier) / divisor;
        p.z = std::round(p.z * multiplier) / divisor;
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

} // railroad
