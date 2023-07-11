/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_SEEDHELPER_H
#define RAILROAD_SEEDHELPER_H

#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace railroad
{
class SeedHelper
{
public:
    typedef typename pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;

    enum SeedType
    {
        NONE = -1,
        ARGUMENT = 1,
        CABLE = 10,
        POLE = 11,
        RAIL = 21,
    };

    SeedHelper(){};
    SeedHelper(const std::vector<std::string> seedPaths, const std::vector<std::string> seedTypes);
    SeedType getSeedTypeFromString(std::string seedTypeString);
    bool seedsAreValid();
    void loadSeedFiles();
    int getSeedFileCount();
    PointCloudConstPtr getSeedCloud(SeedType seedType);
    void addArgumentSeed(PointCloudConstPtr seed);
    void removeArgumentSeed();
private:
    std::map<std::string, SeedType> _seedMap{{"rail", RAIL}, {"pole", POLE}, {"cable", CABLE}};
    std::vector<std::string> _seedPaths;
    std::vector<SeedType> _seedTypes;
    std::map<SeedType, PointCloudConstPtr> _seedClouds;
}; // railroad
}
#endif //RAILROAD_SEEDHELPER_H
