/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_BENCHMARKHELPER_H
#define RAILROAD_BENCHMARKHELPER_H

namespace railroad
{

class BenchmarkHelper
{
public:
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> noiseFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointXYZ> errorPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void writeCSV(std::string fileName, std::vector<pcl::PointXYZ> errors);
};
}
#endif //RAILROAD_BENCHMARKHELPER_H