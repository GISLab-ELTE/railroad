/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_LASHELPER_H
#define RAILROAD_LASHELPER_H

#include <string>
#include <climits>

#include <lasreader.hpp>
#include <laswriter.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace railroad
{
extern bool LASAutoShift;

LASheader readLASHeader(const std::string &filename);

pcl::PointCloud<pcl::PointXYZ>::Ptr readLAS(
    const std::string &filename,
    LASheader &header,
    unsigned long maxSize = ULONG_MAX);

inline pcl::PointCloud<pcl::PointXYZ>::Ptr readLAS(
    const std::string &filename,
    unsigned long maxSize = ULONG_MAX)
{
    LASheader header;
    return readLAS(filename, header, maxSize);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr readLAS(
    const std::string &filename,
    LASheader &header,
    long minX, long maxX, long minY, long maxY,
    unsigned long maxSize = ULONG_MAX);

void writeLAS(
    const std::string &filename,
    const LASheader &header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void writeLAS(
    const std::string &filename,
    const LASheader &header,
    const pcl::PointCloud<pcl::PointXYZL>::Ptr cloud);

} // railroad

#endif //RAILROAD_LASHELPER_H
