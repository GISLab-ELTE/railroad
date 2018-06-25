/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_PCLHELPER_H
#define RAILROAD_PCLHELPER_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

std::ostream &operator<<(std::ostream &out, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr diffPointClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr original,
    pcl::PointCloud<pcl::PointXYZ>::Ptr remover);

pcl::PointCloud<pcl::PointXYZ>::Ptr diffPointClouds(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr original,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr remover,
    int precision);

pcl::PointCloud<pcl::PointXYZ>::Ptr mergePointClouds(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr result);

pcl::PointCloud<pcl::PointXYZI>::Ptr mergePointCloudsVisual(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr result,
    float resultIntensity = 1);

} // railroad

#endif //RAILROAD_PCLHELPER
