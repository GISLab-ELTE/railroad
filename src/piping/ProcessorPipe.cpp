/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/io.h>

#include "ProcessorPipe.h"
#include "../dim2/Projection.h"
#include "../dim2/Dim2Helper.h"

namespace railroad
{

ProcessorPipe *ProcessorPipe::add(CloudProcessor *step)
{
    steps.push_back(step);
    return this;
}


ProcessorPipe::PointCloudPtr ProcessorPipe::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr workCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*_cloud, *workCloud);
    std::vector<cv::Mat> toShowObjects;
    int counter = 0;
    for (auto step : steps) {
        step->setInputCloud(workCloud);
        workCloud = step->execute();

        Projection projectionObj(workCloud, 0.25);
        projectionObj.saveToFile("step" + std::to_string(counter) + ".png");
        toShowObjects.push_back(projectionObj.getImage());
        counter++;
    }
    cv::Mat toShowObjectsResult = createGridFromMats(toShowObjects);
    imwriteWithInverse("steps.png", toShowObjectsResult);
    return workCloud;
}

ProcessorPipe::~ProcessorPipe()
{
    for (auto step : steps) {
        if (step != nullptr) {
            delete step;
        }
    }
}

ProcessorPipe::ProcessorPipe() : CloudProcessor("Pipe")
{

}

std::map<std::string, double> ProcessorPipe::getTimeResults() const
{
    std::map<std::string, double> ret;
    double sum = 0.0f;
    for (auto step : steps) {
        double stepTime = step->getMeasuredTime();
        ret[step->name()] = stepTime;
        sum += stepTime;
    }
    ret["SUM"] = sum;
    return ret;
}

std::map<std::string, int> ProcessorPipe::getFilteredPointsResults() const
{
    std::map<std::string, int> ret;
    int sum = 0;
    for (auto step : steps) {
        int numberOfFilteredPoints = step->getNumberOfFilteredPoints();
        ret[step->name()] = numberOfFilteredPoints;
        sum += numberOfFilteredPoints;
    }
    ret["SUM"] = sum;
    return ret;
}

} // railroad
