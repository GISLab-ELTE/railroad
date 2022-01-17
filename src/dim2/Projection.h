//
// Created by hudi on 2018.06.13..
//

#ifndef RAILROAD_PROJECTION_H
#define RAILROAD_PROJECTION_H

#include <string>

/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>

namespace railroad
{

class Projection
{
public:
    Projection(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, float gridCellSize);

    cv::Mat getImage();

    pcl::PointXYZ getMinPoint() const;
    pcl::PointXYZ getSpaceSize() const;

    cv::Vec2i projectPoint(pcl::PointXYZ xyz);
    void saveToFile(std::string filename) const;
    void saveToTextFile(std::string filename) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    grepPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, cv::Mat areaToKeep,
                   bool extraCondition(const pcl::PointXYZ &minPt, const pcl::PointXYZ &maxPt,
                                       const pcl::PointXYZ &spaceSize,
                                       uchar actCell, const pcl::PointXYZ &point) = [](
                       const pcl::PointXYZ &minPt,
                       const pcl::PointXYZ &maxPt,
                       const pcl::PointXYZ &spaceSize,
                       uchar actCellValue,
                       const pcl::PointXYZ &point) -> bool {
                       return true;
                   });
protected:
    pcl::PointXYZ minPt, maxPt;
    pcl::PointXYZ spaceSize;
    cv::Mat projection;
    float gridCellSize;
};

} // railroad

#endif //RAILROAD_PROJECTION_H
