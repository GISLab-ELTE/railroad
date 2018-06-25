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

/**
 * Literal for unsigned char.
 *
 * Many LAS fields are stored as unsigned char (1 byte in C++).
 * We define a custom literal ('_uc') so we can handle numbers easier in the code,
 * as C++ has no native literal forn unsigned char.
 */
inline constexpr unsigned char operator "" _uc(unsigned long long int value) noexcept
{
    return static_cast<unsigned char>(value);
}

namespace railroad
{

enum class LASClass : unsigned char
{
    CREATED = 0_uc,
    UNCLASSIFIED = 1_uc,
    GROUND = 2_uc,
    LOW_VEGETATION = 3_uc,
    MEDIUM_VEGETATION = 4_uc,
    HIGH_VEGETATION = 5_uc,
    BUILDING = 6_uc,
    LOW_POINT = 7_uc,
    MODEL_KEY_POINT = 8_uc,
    WATER = 9_uc,
    // 10-12 is ASPRS definition reserved
    CABLE = 13_uc
};

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
        const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

} // railroad

#endif //RAILROAD_LASHELPER_H
