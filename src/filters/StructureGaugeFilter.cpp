/*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include "StructureGaugeFilter.h"

namespace railroad
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr StructureGaugeFilter::getStructureCloud(pcl::PointXYZ middle, pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud)  
    {
        std::vector<std::tuple<pcl::PointXYZ,bool>> polyLine;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if(_regionNational){
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-1.575,0,middle.z+ 0.430),false});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-1.575,0,middle.z+ 3.500),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-1.395,0,middle.z+ 3.805),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-0.690,0,middle.z+ 4.650),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+0.690,0,middle.z+ 4.650),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+1.395,0,middle.z+ 3.805),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+1.575,0,middle.z+ 3.500),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+1.575,0,middle.z+ 0.430),false});
        }
        else{
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-1.575,0,middle.z+ 0.430),false});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-1.575,0,middle.z+ 3.175),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-1.000,0,middle.z+ 4.000),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x-0.400,0,middle.z+ 4.280),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+0.400,0,middle.z+ 4.280),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+1.000,0,middle.z+ 3.000),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+1.575,0,middle.z+ 3.175),true});
            polyLine.push_back(std::tuple<pcl::PointXYZ,bool>{pcl::PointXYZ(middle.x+1.575,0,middle.z+ 0.430),false});
        }

        for (unsigned int i = 0; i < rotatedCloud->size(); i++) {
            pcl::PointXYZ point = rotatedCloud->at(i);
            bool is=true;
            for(int j = 0 ; j< polyLine.size();j++){
                 pcl::PointXYZ pointfst,pointsnd;
                if(j< polyLine.size()-1) {
                    pointfst=std::get<0>(polyLine.at(j));
                    pointsnd=std::get<0>(polyLine.at(j+1));
                }
                else
                {
                    pointfst=std::get<0>(polyLine.at(j));
                    pointsnd=std::get<0>(polyLine.at(0));
                }
                    if(pointfst.x==pointsnd.x){
                        if(!((point.x<pointfst.x && std::get<1>(polyLine.at(j)))||
                        (point.x>pointfst.x && !std::get<1>(polyLine.at(j)))))
                            is=false;
                    }
                    else {
                        if (pointfst.z==pointsnd.z){
                            if(!((point.z<pointfst.z && std::get<1>(polyLine.at(j)))||
                            (point.z>pointfst.z && !std::get<1>(polyLine.at(j)))))
                                is=false;
                        } else  {
                            if(!(((point.z< pointfst.z+(point.x-pointfst.x)*((pointfst.z-pointsnd.z)/(pointfst.x-pointsnd.x))&&std::get<1>(polyLine.at(j))))
                            ||((point.z> pointfst.z+(point.x-pointfst.x)*((pointfst.z-pointsnd.z)/(pointfst.x-pointsnd.x))&&!std::get<1>(polyLine.at(j))))))
                                is=false;
                        }

                    }
            }
            if (is)
                {   
                    cloud->push_back(_cloud->at(i));
                }
        }
        return cloud;
    }
   pcl::PointCloud<pcl::PointXYZ>::Ptr StructureGaugeFilter::process()
    { 
        double angle = getRansac();
        pcl::PointXYZ middle = getMiddlePoint(getRotatedSeedCloud(angle));
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud = getRotatedCloud(angle);
        return  getStructureCloud(middle,rotatedCloud);
    }
}
