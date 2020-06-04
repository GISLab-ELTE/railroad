/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>

#include "WidthFilter.h"

namespace railroad
{
pcl::PointCloud<pcl::PointXYZ>::Ptr WidthFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::VectorXf coeff;

    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_p(
        new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(_seedCloud));
    model_p->setAxis(Eigen::Vector3f::UnitY());
    model_p->setEpsAngle(1.0);
    std::vector<int> inliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(.9);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coeff);

    Eigen::Vector3f c(coeff[3], coeff[4], coeff[5]);

    float angle = asin(abs(c[0]) / c.norm());

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f(0.0, 0.0, 1.0)));
    transform.translation() << Eigen::Vector3f(0.0, 0.0, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedSeed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*_cloud, *rotatedCloud, transform);
    pcl::transformPointCloud(*_seedCloud, *rotatedSeed, transform);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*rotatedSeed, minPt, maxPt);

    double min = minPt.x - _maxDistance;
    double max = maxPt.x + _maxDistance;

    for (unsigned int i = 0; i < rotatedCloud->size(); ++i) {
        pcl::PointXYZ point = rotatedCloud->at(i);
        if (point.x <= max && point.x >= min)
            cloud->push_back(_cloud->at(i));
    }

    return cloud;
}
} // railroad