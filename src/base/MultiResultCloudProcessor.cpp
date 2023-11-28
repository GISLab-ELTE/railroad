/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/io.h>

#include "MultiResultCloudProcessor.h"
#include "../helpers/LogHelper.h"

namespace railroad
{
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> MultiResultCloudProcessor::execute()
{
    LOG(debug) << "[" << _name << "] Started filter";

    startTimeMeasure();
    std::vector<PointCloudPtr> result = process();
    stopTimeMeasure();

    int resultPointsSize = 0;

    for (std::size_t i = 0; i < result.size(); i++) {
        resultPointsSize += result[i]->size();
    }

    _numberOfFilteredPoints = _cloud->points.size() - resultPointsSize;

    LOG(info) << "[" << _name << "] "
              << "Filtered points: " << (_cloud->size() - resultPointsSize)
              << " (" << _cloud->size() << " -> " << resultPointsSize << ")";
    return result;
}
} // railroad