/*
 * BSD 3-Clause License
 * Copyright (c) 2018-2023, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/io.h>

#include "SingleResultCloudProcessor.h"
#include "../helpers/LogHelper.h"

namespace railroad
{
pcl::PointCloud<pcl::PointXYZ>::Ptr SingleResultCloudProcessor::execute()
{
    LOG(debug) << "[" << _name << "] Started filter";

    startTimeMeasure();
    PointCloudPtr result = process();
    stopTimeMeasure();
    _numberOfFilteredPoints = _cloud->points.size() - result->points.size();

    LOG(info) << "[" << _name << "] "
              << "Filtered points: " << (_cloud->size() - result->size())
              << " (" << _cloud->size() << " -> " << result->size() << ")";
    return result;
}
} // railroad