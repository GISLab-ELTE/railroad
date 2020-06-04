/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <iostream>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>

#include "RansacFilter.h"
#include "../helpers/LogHelper.h"

using namespace cv;

namespace railroad
{
pcl::PointCloud<pcl::PointXYZ>::Ptr RansacFilter::process()
{
    stopTimeMeasure();

    Eigen::VectorXf coeff;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*_cloud, *cloud);
    startTimeMeasure();

    int size;

    do {
        compute(cloud, temp, Eigen::Vector3f::UnitY(), .9, coeff);
        size = temp->size();
        LOG(trace) << "Size: " << temp->size();
        result->insert(result->end(), temp->begin(), temp->end());
    } while (size > _ransacMinSize);

    return result;
}

void RansacFilter::compute(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &temp,
                           const Eigen::Vector3f &axis, double threshold, Eigen::VectorXf &coeff)
{
    const double eps = 1.0;

    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_p(
        new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
    model_p->setAxis(axis);
    model_p->setEpsAngle(eps);
    std::vector<int> inliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(threshold);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coeff);

    pcl::copyPointCloud(*cloud, inliers, *temp);

    pcl::PointIndices::Ptr inlierss(new pcl::PointIndices());

    inlierss->indices = inliers;

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inlierss);
    extract.setNegative(true);
    extract.filter(*cloud);
}
}  // railroad
