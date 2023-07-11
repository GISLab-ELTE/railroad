/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <iostream>

#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "GrowthFilter.h"
#include "../helpers/LogHelper.h"

namespace railroad
{
pcl::PointCloud<pcl::PointXYZ>::Ptr GrowthFilter::process()
{
    stopTimeMeasure();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr seedCloudToUse;

    if(_runOnSeed != SeedHelper::SeedType::NONE) {
        seedCloudToUse = seedCloud(_runOnSeed);
        LOG(debug) << "Using seed cloud specified in Pipes.h in filter";
    } else {
        LOG(error) << "Seed type not specified in Pipes.h for HeightFilter";
    }

    pcl::copyPointCloud(*_cloud, *cloud);
    startTimeMeasure();

    const double eps = 1.0;
    Eigen::VectorXf coeff;

    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_p(
        new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(seedCloudToUse));
    model_p->setAxis(Eigen::Vector3f::UnitY());
    model_p->setEpsAngle(eps);
    std::vector<int> inliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(.9);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coeff);

    Eigen::Vector3f c(coeff[3], coeff[4], coeff[5]);
    LOG(debug) << "Direction: " << c;
    float angle = asin(abs(c[0]) / c.norm());
    LOG(debug) << "Angle: " << angle;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f(0.0, 0.0, 1.0)));
    transform.translation() << Eigen::Vector3f(0.0, 0.0, 0.0);

    pcl::transformPointCloud(*seedCloudToUse, *rotatedCloud, transform);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0.0;
    coefficients->values[1] = 1.0;
    coefficients->values[2] = 0.0;
    coefficients->values[3] = 0.0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(rotatedCloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projectedCloud);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*projectedCloud, minPt, maxPt);

    int gridSize = _seedGridCount;
    std::vector<std::vector<std::vector<int>>> grids(
        gridSize, std::vector<std::vector<int>>(gridSize, std::vector<int>(0)));
    float x = (float) gridSize / (maxPt.x - minPt.x);
    float z = (float) gridSize / (maxPt.z - minPt.z);

    for (size_t i = 0; i < projectedCloud->size(); ++i) {
        int xGrid = floor((projectedCloud->at(i).x - minPt.x) * x);
        int zGrid = floor((projectedCloud->at(i).z - minPt.z) * z);

        if (xGrid == gridSize)
            xGrid = gridSize - 1;

        if (zGrid == gridSize)
            zGrid = gridSize - 1;

        grids[xGrid][zGrid].push_back(i);
    }

    std::vector<int> indices;
    std::vector<pcl::PointXYZ> seeds;
    std::vector<Eigen::Vector3f> directions;

    for (int i = 0; i < gridSize; ++i) {
        for (int j = 0; j < gridSize; ++j) {
            if (!grids[i][j].empty()) {
                indices.insert(indices.end(), grids[i][j].begin(), grids[i][j].end());

                pcl::copyPointCloud(*seedCloudToUse, indices, *tempCloud);
                std::sort(tempCloud->points.begin(), tempCloud->points.end(),
                          [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) { return p1.y < p2.y; });
                seeds.push_back(calcSeed(tempCloud, tempCloud->size()*0.05));
                indices.clear();
            }
        }
    }

    float boxLength = _seedBoxLength;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    std::vector<int> insideBox;
    std::vector<int> cable;

        for(int i = 0; i < 3; ++i)
        {
            BoundingBox b(seeds[i], boxLength);
            LOG(trace) << "Seed point:" << seeds[i] << std::endl;

            for (size_t j = 0; j < cloud->size(); ++j) {
                if (b.containsPoint(cloud->at(j))) {
                    insideBox.push_back(j);
                    cable.push_back(j);
                }
            }

            std::sort(insideBox.begin(), insideBox.end(),
                      [cloud](const int &i, const int &j) { return cloud->at(i).y < cloud->at(j).y; });

            pcl::PointXYZ s = seeds[i];
            while (cloud->at(insideBox[insideBox.size() - 1]).y + 0.1 < maxPt.y) {
                int quarterSize = insideBox.size() * 0.75;
                insideBox.erase(insideBox.begin(), insideBox.begin() + quarterSize);

                pcl::copyPointCloud(*cloud, insideBox, *tempCloud);
                pcl::PointXYZ tempSeed = s;
                s = calcSeed(tempCloud, tempCloud->size());

                if (tempSeed.y >= s.y)
                    s.y += _seedBoxLength;

                findCable(s, boxLength, insideBox, cloud, cable, -_seedBoxLength);
            }

            insideBox.clear();

            //std::cout << "cloud " << cloud->size() << std::endl;
            //std::cout << "box " << insideBox.size() << std::endl;
        }


        for(size_t i = 0; i < 3; ++i)
        {
            BoundingBox b(seeds[i], boxLength);
            LOG(trace) << "Seed point:" << seeds[i] << std::endl;

            for(size_t j = 0; j < cloud->size(); ++j)
            {
                //std::cout << "point " << rotatedCloud->at(j) << std::endl;
                if(b.containsPoint(cloud->at(j)))
                {
                    insideBox.push_back(j);
                    cable.push_back(j);
                }
            }

            //std::cout << "first box size " << insideBox.size() << std::endl;
            std::sort(insideBox.begin(), insideBox.end(), [cloud](const int &i, const int &j) { return cloud->at(i).y < cloud->at(j).y; });

            pcl::PointXYZ s = seeds[i];

            while(cloud->at(insideBox[0]).y - 0.1 > minPt.y)
            {
                //std::cout << " seedpoint in " << s << std::endl;
                int quarterSize = insideBox.size() * 0.75;
                insideBox.erase(insideBox.end() - quarterSize, insideBox.end());

                pcl::copyPointCloud(*cloud, insideBox,*tempCloud);
                pcl::PointXYZ tempSeed = s;
                s = calcSeed(tempCloud, tempCloud->size());

                if(tempSeed.y <= s.y)
                    s.y -= _seedBoxLength;

                findCable(s, boxLength, insideBox, cloud, cable, _seedBoxLength);
            }

            insideBox.clear();

            //std::cout << "cloud " << cloud->size() << std::endl;
            //std::cout << "box " << insideBox.size() << std::endl;
    }

    stopTimeMeasure();

    std::sort(cable.begin(), cable.end());
    auto last = std::unique(cable.begin(), cable.end());
    cable.erase(last, cable.end());
    pcl::copyPointCloud(*cloud, cable, *tempCloud);

    return tempCloud;
}

    pcl::PointXYZ GrowthFilter::calcSeed(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, size_t length)
    {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    for (size_t i = 0; i < length; ++i) {
        x += cloud->at(i).x;
        y += cloud->at(i).y;
        z += cloud->at(i).z;
    }
        return {x / length, y / length, z / length};
    }

    void GrowthFilter::findCable(pcl::PointXYZ s, float boxLength, std::vector<int>& insideBox, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& cable, float skip)
    {
        BoundingBox bb(s, boxLength);
        insideBox.clear();

        for(size_t j = 0; j < cloud->size(); ++j)
        {
            //std::cout << "point " << cloud->at(j) << std::endl;
            if(bb.containsPoint(cloud->at(j)))
            {
                insideBox.push_back(j);
            }
        }

        if(insideBox.empty())
        {
            s.y += skip;

            bb = BoundingBox(s, boxLength);

            for(size_t j = 0; j < cloud->size(); ++j)
            {
                if(bb.containsPoint(cloud->at(j)))
                {
                    insideBox.push_back(j);
                }
            }

            float xAvg = 0.0f;
            for(size_t box = cable.size() - 100; box < cable.size(); ++box)
            {
                xAvg += (float) cloud->at(cable[box]).x;
            }

            xAvg /= 100.0f;

            insideBox.erase(std::remove_if(insideBox.begin(), insideBox.end(), [&cloud, &xAvg](const int &i) { return (xAvg + 0.5 < cloud->at(i).x || xAvg - 0.5 > cloud->at(i).x); }), insideBox.end());
        }

        if(insideBox.size() > _seedMaxNumberOfPoints) {
            BoundingBox quarterBox = BoundingBox(s, (boxLength * 0.25f));
            std::vector<int> newBox;

            for (size_t j = 0; j < cloud->size(); ++j) {
                if (quarterBox.containsPoint(cloud->at(j))) {
                    newBox.push_back(j);
                }
            }

            if(newBox.size() > 2)
            {
                insideBox.clear();
                insideBox.insert(insideBox.end(), newBox.begin(), newBox.end());
            }
            else {
                std::sort(insideBox.begin(), insideBox.end(), [cloud](const int &i, const int &j) { return cloud->at(i).x < cloud->at(j).x; });
                float size = insideBox.size() * 0.1;
                insideBox.erase(insideBox.begin() + size, insideBox.end());
            }
        }

        std::sort(insideBox.begin(), insideBox.end(), [cloud](const int &i, const int &j) { return cloud->at(i).y < cloud->at(j).y; });
        //std::cout << "box size " << insideBox.size() << std::endl;

        cable.insert(cable.end(), insideBox.begin(), insideBox.end());
    }
} // railroad
