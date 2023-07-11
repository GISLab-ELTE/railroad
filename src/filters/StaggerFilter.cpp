/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "StaggerFilter.h"
#include "../helpers/LogHelper.h"
#include "../helpers/PCLHelper.h"

using namespace std;

namespace railroad
{

inline std::vector<float> getValuesOnAxis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, uint axis, double centre)
{
    std::vector<float> values = std::vector<float>(input->points.size());
    for(uint i = 0; i < input->points.size(); ++i)
    {
        double offsetFromCentre = input->points.at(i).getArray3fMap()[axis] - centre;
        values.push_back(offsetFromCentre);     
    }
    return values;
}

inline Eigen::Affine3f getTransformMatrix(Eigen::Vector3f rotationVector)
{
    float zRot = asin(abs(rotationVector[0]) / rotationVector.norm());

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-zRot, Eigen::Vector3f(0.0, 0.0, 1.0)));
    transform.translation() << Eigen::Vector3f(0.0, 0.0, 0.0);

    return transform;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StaggerFilter::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centreCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr railSeed = seedCloud(SeedHelper::SeedType::RAIL);

    startTimeMeasure();

    Eigen::Vector3f vector = getFirstEigenVector(railSeed);

    LOG(debug) << "First eigenvector: " << vector;

    Eigen::Affine3f transform = getTransformMatrix(vector);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*_cloud, *rotatedCloud, transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedRailSeed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*railSeed, *rotatedRailSeed, transform);

    Eigen::Vector3f rotatedVector = getFirstEigenVector(rotatedRailSeed);
    pcl::PointXYZ gaugeCentroid = getCentroid(rotatedRailSeed);
 
    uint axis;

    // Deciding if we need to check stagger on the X or Y axis
    if(std::abs(rotatedVector.x()) > std::abs(rotatedVector.y()))
        axis = 1;
    else
        axis = 0;

    double gaugeCentre = gaugeCentroid.getArray3fMap()[axis];

    LOG(debug) << "Track gauge centre: " << gaugeCentre;
    
    double maxPosStaggerFound = 0.0;
    double maxNegStaggerFound = 0.0;

    LOG(info) << "Working with stagger limit of +/-" << maxStaggerLimit << " meters (+/-" << staggerThreshold << " meters threshold)";

    for(uint i = 0; i < rotatedCloud->points.size(); ++i) {
        pcl::PointXYZ point_aligned = rotatedCloud->points.at(i);
        pcl::PointXYZ point = _cloud->points.at(i);

        double hValue = point_aligned.getArray3fMap()[axis];
        double relativeStagger = std::abs(hValue - gaugeCentre);

        if(hValue > gaugeCentre && relativeStagger > maxPosStaggerFound)
            maxPosStaggerFound = relativeStagger;
        else if(hValue < gaugeCentre && relativeStagger > maxNegStaggerFound)
            maxNegStaggerFound = relativeStagger;

        if(relativeStagger > (maxStaggerLimit + staggerThreshold))
            outputCloud->points.push_back(point);

        if(relativeStagger < minStaggerLimit)
            centreCloud->points.push_back(point);
    }

    LOG(info) << "Maximum positive stagger from track centre in meters: " << maxPosStaggerFound;
    LOG(info) << "Maximum negative stagger from track centre in meters: " << maxNegStaggerFound;

    double centrePercentage = (1.0*centreCloud->points.size() / _cloud->points.size())*100;

    LOG(info) << centrePercentage << "% of points fall close to track centre";

    if(centrePercentage > 45.0)
        LOG(warning) << "Possible lack of stagger: more than 45% of points fall within " << minStaggerLimit << " distance of the track centre!";

    return outputCloud;
}
}  // railroad

