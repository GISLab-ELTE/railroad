    /*
 * BSD 3-Clause License
 * Copyright (c) 2022, Milán Horváth
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */


#include <iostream>
#include <string>
#include <vector>

#include <pcl/common/distances.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "LogHelper.h"

#include "BenchmarkHelper.h"
namespace railroad
{
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> BenchmarkHelper::noiseFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr noiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr errorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int i = 0; i < cloud->size(); i++) {
            int c=0;
            for(unsigned int j=0; j < cloud->size(); j++){
                if(pcl::euclideanDistance (cloud->at(i),cloud->at(j))<0.8)
                    c++;
            }
            if(c<40)
                noiseCloud->push_back(cloud->at(i));
            else
            {
                errorCloud->push_back(cloud->at(i));
            }
        }
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec;
        vec.push_back(errorCloud);
        vec.push_back(noiseCloud);

        return vec;
    }
    std::vector<pcl::PointXYZ> BenchmarkHelper::errorPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        std::vector<pcl::PointXYZ> errors={cloud->at(0)};

        for(unsigned int i=1; i < cloud->size(); i++){
            bool b=false;
            for(auto e: errors){
                if(pcl::euclideanDistance (e,cloud->at(i))<5)
                    b=true;
            }    
            if(!b)
                errors.push_back(cloud->at(i));
        }
        return errors;
    }
    void BenchmarkHelper::writeCSV(std::string fileName, std::vector<pcl::PointXYZ> errors){
        std::ofstream write;
        write.open(fileName);
        LOG(debug) << "Found error no.: " << errors.size();
        write << "Found " << errors.size() << " error(s) at following coordinate(s)\n";
        write << "x,y\n";
        for(auto e : errors){
            write << e.x<< ","<< e.y<<std::endl;
        }
        write.close();
    }

}