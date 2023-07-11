/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */


#include <string>
#include <vector>
#include <set>
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

    for (const std::string& seedType : seedTypes) {
        _seedTypes.push_back(getSeedTypeFromString(seedType));
    }
}

int SeedHelper::getSeedFileCount() {
    return _seedPaths.size();
}

SeedHelper::SeedType SeedHelper::getSeedTypeFromString(std::string seedTypeString) {
    boost::algorithm::to_lower(seedTypeString);

    const auto type = _seedMap.find(seedTypeString);

    if (type != _seedMap.end()) {
        return type->second;
    } else {
        LOG(error) << "Invalid seedType provided as argument: " << seedTypeString;
        throw std::invalid_argument("Invalid seedType provided as argument");
    }
}

bool SeedHelper::seedsAreValid() {
    if (_seedPaths.size() != _seedTypes.size()) {
        LOG(error) << "The number of supplied seed paths (" << _seedPaths.size() <<
        ") and seed types (" << _seedTypes.size() << ") do not match!";

        return false;
    }

    for (const std::string& seedPath : _seedPaths) {
        if (seedPath.length() > 0 && !fs::exists(seedPath)) {
            LOG(error) << "The seed file does not exist on the following path: " << seedPath;
            return false;
        }
    }

    return true;
}

void SeedHelper::loadSeedFiles() {
    std::size_t index = 0;
    for (const std::string& seedPath : _seedPaths) {
        PointCloudPtr seed = readLAS(seedPath);
        SeedType seedType = _seedTypes[index++];
        _seedClouds[seedType] = seed;
        LOG(info) << "Seed file " << seedPath << " loaded, size: " << seed->size();
    }
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr SeedHelper::getSeedCloud(SeedType seedType) {
    if (_seedClouds.count(SeedType::ARGUMENT)) {
      LOG(debug) << "Returning argument seed instead of " << seedType;
      return _seedClouds[SeedType::ARGUMENT];
    }

    if (_seedClouds.count(seedType))
      return _seedClouds[seedType];

    std::vector<std::string> keys(_seedMap.size());
    for (auto it = _seedMap.begin(); it != _seedMap.end(); it++) {
        keys[it->second] =it->first;
    }

    LOG(error) << "No seed cloud provided for type " << keys[seedType];
    throw std::invalid_argument("Missing seed cloud!");
}


void SeedHelper::addArgumentSeed(PointCloudConstPtr seed)
{
    _seedClouds[SeedType::ARGUMENT] = seed;
    LOG(info) << "Argument seed added, size: " << seed->size();
}

void SeedHelper::removeArgumentSeed()
{
  _seedClouds.erase(SeedType::ARGUMENT);
  LOG(info) << "Argument seed removed";
}

} // railroad
