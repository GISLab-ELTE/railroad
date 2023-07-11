/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */


#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "SeedHelper.h"
#include "LASHelper.h"
#include "LogHelper.h"

namespace fs = boost::filesystem;

namespace railroad
{

SeedHelper::SeedHelper(const std::vector<std::string> seedPaths, const std::vector<std::string> seedTypes) {
    _seedPaths = seedPaths;

    for(std::string seedType : seedTypes) {
        _seedTypes.push_back(getSeedTypeFromString(seedType));
    }
}

int SeedHelper::getSeedFileCount() {
    return _seedPaths.size();
}

SeedHelper::SeedType SeedHelper::getSeedTypeFromString(std::string seedTypeString) {
    boost::algorithm::to_lower(seedTypeString);

    const auto type = _seedMap.find(seedTypeString);

    if(type != _seedMap.end()) {
        return type->second;
    } else {
        LOG(error) << "Invalid seedType provided as argument: " << seedTypeString;
        throw std::invalid_argument("Invalid seedType provided as argument");
    }
}

bool SeedHelper::seedsAreValid() {
    if(_seedPaths.size() != _seedTypes.size()) {
        LOG(error) << "The number of supplied seed paths (" << _seedPaths.size() << 
        ") and seed types (" << _seedTypes.size() << ") do not match!";

        return false;
    }

    for(std::string seedPath : _seedPaths) {
        if(seedPath.length() > 0 && !fs::exists(seedPath)) {
            LOG(error) << "The seed file does not exist on the following path: " << seedPath;
            return false;
        }
    }

    return true;
}

void SeedHelper::loadSeedFiles() {
    for(std::string seedPath : _seedPaths) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr seed = readLAS(seedPath);
        _seedClouds.push_back(seed);
        LOG(info) << "Seed file " << seedPath << " loaded, size: " << seed->size();
    }
}

void SeedHelper::setTempSeedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &tempSeedCloud) {
    _tempSeedCloud = tempSeedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SeedHelper::getTempSeedCloud() {
    return _tempSeedCloud;
}

PointCloudConstPtr SeedHelper::getSeedCloud(SeedType seedType) {
    for (uint i = 0; i < _seedClouds.size(); i++)
    {
        if(_seedTypes[i] == seedType) {
            return _seedClouds[i];
        }
    }    

    std::string keys[_seedMap.size()];
    for (auto it = _seedMap.begin(); it != _seedMap.end(); it++) {
        keys[it->second] =it->first;
    }
    
    LOG(error) << "No seed cloud provided for type " << keys[seedType];
    throw std::invalid_argument("Missing seed cloud!");
}

} // railroad
