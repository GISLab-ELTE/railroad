/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ErrorProcessor.h"
#include "../helpers/LogHelper.h"

namespace railroad
{
    double ErrorProcessor::getAngle(pcl::PointXYZ vector1, pcl::PointXYZ vector2)
    {
        
        double div = vector1.x * vector2.x + vector1.y * vector2.y / 
            ( 
                sqrt(vector1.x * vector1.x + vector1.y * vector1.y) *
                sqrt(vector2.x * vector2.x + vector2.y * vector2.y)
            );
        double result = acos(
            div > 1.0f ? div = 1.0f : (div < -1.0f ? div = -1.0f : div = div )
        );
        return std::min(result, abs((double) CV_PI - result));
    }

    double ErrorProcessor::getRansac()
    {
        Eigen::VectorXf coeff;

        pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_p(
                new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(separateLines()));


        model_p->setAxis(Eigen::Vector3f::UnitY());
        model_p->setEpsAngle(1.0);
        std::vector<int> inliers;

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();
        ransac.getInliers(inliers);
        ransac.getModelCoefficients(coeff);

        Eigen::Vector3f c(coeff[3], coeff[4], coeff[5]);
        return asin(abs(c[0]) / c.norm());
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr ErrorProcessor::separateLines()
    {        
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D( *_seedCloud, minPoint, maxPoint);
        double angle = getAngle(
            pcl::PointXYZ ( 
                maxPoint.x - minPoint.x, 
                maxPoint.y - minPoint.y,
                maxPoint.z - minPoint.z
            ),
            pcl::PointXYZ ( 0, 1, 0)
        );
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr leftLineCloud(new pcl::PointCloud<pcl::PointXYZ>);

        double middle = getMiddlePoint(getRotatedSeedCloud(angle)).x;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedSeedCloud = getRotatedSeedCloud(angle);

        for (unsigned int i = 0; i < rotatedSeedCloud->size(); ++i) {
            pcl::PointXYZ point = rotatedSeedCloud->at(i);
            if ( (point.x >= middle) )
                leftLineCloud->push_back(_seedCloud->at(i));
        }

        return  leftLineCloud;
        
    }

    

    pcl::PointXYZ ErrorProcessor::getMiddlePoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
    {
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloud, minPoint, maxPoint);
        return pcl::PointXYZ(
            minPoint.x + (maxPoint.x - minPoint.x) / 2, 
            minPoint.y + (maxPoint.y - minPoint.y) / 2,  
            minPoint.z + (maxPoint.z - minPoint.z) / 2
            );
    } 

    pcl::PointCloud<pcl::PointXYZ>::Ptr ErrorProcessor::getRotatedSeedCloud(double angle) 
    {
        Eigen::Affine3f rotation = Eigen::Affine3f::Identity();
        rotation.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f(0.0, 0.0, 1.0)));
        rotation.translation() << Eigen::Vector3f(0.0, 0.0, 0.0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedSeedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*_seedCloud, *rotatedSeedCloud, rotation);
        return rotatedSeedCloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ErrorProcessor::getRotatedCloud(double angle) 
    {        
        Eigen::Affine3f rotation = Eigen::Affine3f::Identity();
        rotation.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f(0.0, 0.0, 1.0)));
        rotation.translation() << Eigen::Vector3f(0.0, 0.0, 0.0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*_cloud, *rotatedCloud, rotation);
        return rotatedCloud;
    }
    
} // railroad