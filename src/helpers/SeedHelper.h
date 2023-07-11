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

typedef typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;

namespace railroad
{
class SeedHelper
{

public:
    enum SeedType
    {
        NONE = -1,
        RAIL = 0,
        POLE = 1,
        CABLE = 2,
        TIES = 3
    };
    SeedHelper(){};
    SeedHelper(const std::vector<std::string> seedPaths, const std::vector<std::string> seedTypes);
    SeedType getSeedTypeFromString(std::string seedTypeString);
    bool seedsAreValid();
    void loadSeedFiles();
    void setTempSeedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &tempSeedCloud);
    int getSeedFileCount();
    PointCloudConstPtr getSeedCloud(SeedType seedType);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTempSeedCloud();
private:
    std::map<std::string, SeedType> _seedMap{{"rail", RAIL}, {"pole", POLE}, {"cable", CABLE}, {"ties", TIES}};
    std::vector<std::string> _seedPaths;
    std::vector<SeedType> _seedTypes;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _seedClouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _tempSeedCloud;
}; // railroad
}
#endif //RAILROAD_SEEDHELPER_H
