/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */



#include "CableDistanceFilter.h"
namespace railroad
{

    

   
    pcl::PointCloud<pcl::PointXYZ>::Ptr CableDistanceFilter::getCableCloud(pcl::PointXYZ middle, pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud)  
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int i = 0; i < rotatedCloud->size(); i++) {
            if (((rotatedCloud->at(i).x >= middle.x + _maxCableWidth
            || (rotatedCloud->at(i).x <= middle.x - _maxCableWidth ))
             && rotatedCloud->at(i).z <= middle.z + _maxDistance+0.4 )
            || (rotatedCloud->at(i).z <= middle.z + _maxDistance ))
            {
                if(((rotatedCloud->at(i).x >= middle.x - 1.435/2) 
                    && (rotatedCloud->at(i).x <= middle.x + 1.435/2 ) )){
                    cloud->push_back(_cloud->at(i));
                }
            }
            
        }
        return cloud;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr CableDistanceFilter::process()
    { 
        double angle = getRansac();
        pcl::PointXYZ middle = getMiddlePoint(getRotatedSeedCloud(angle));
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud = getRotatedCloud(angle);
        return getCableCloud(middle,rotatedCloud); 
    }
}