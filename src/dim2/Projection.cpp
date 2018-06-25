/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Projection.h"
#include "Dim2Helper.h"
#include "../helpers/LogHelper.h"

using namespace cv;

namespace railroad
{

Projection::Projection(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, float gridCellSize)
{
    this->gridCellSize = gridCellSize;
    if (cloud->empty()) {
        projection = Mat();
        minPt = pcl::PointXYZ(0, 0, 0);
        maxPt = pcl::PointXYZ(0, 0, 0);
        spaceSize = pcl::PointXYZ(0, 0, 0);
    } else {
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        spaceSize = pcl::PointXYZ(maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z);
        LOG(debug) << "Min:" << minPt << " Max:" << maxPt << " Size:" << spaceSize;
        Size resolution = Size((int) ceil(spaceSize.x / gridCellSize) + 1, (int) ceil(spaceSize.y / gridCellSize) + 1);
        LOG(debug) << "resolution:" << resolution;

        projection = Mat(resolution, CV_8UC1, Scalar(0));
        for (auto point : cloud->points) {
            Vec2i posInMatrix((int) floor((point.x - minPt.x) / gridCellSize),
                              (int) floor((point.y - minPt.y) / gridCellSize));
            uchar actCellVal = projection.at<uchar>(posInMatrix[1], posInMatrix[0]);
            auto actPointVal = (uchar) ((point.z - minPt.z) * 255 / spaceSize.z);
            if (actCellVal < actPointVal) {
                projection.at<uchar>(posInMatrix[1], posInMatrix[0]) = actPointVal;
            }
        }
        LOG(debug) << "projection finished";
    }
}

cv::Mat Projection::getImage()
{
    return projection;
}

pcl::PointXYZ Projection::getMinPoint() const
{
    return minPt;
}

pcl::PointXYZ Projection::getSpaceSize() const
{
    return spaceSize;
}

Vec2i Projection::projectPoint(pcl::PointXYZ point)
{
    return Vec2i((int) floor((point.x - minPt.x) / gridCellSize), (int) floor((point.y - minPt.y) / gridCellSize));;
}

void Projection::saveToFile(std::string filename) const
{
    LOG(trace) << filename;
    imwriteWithInverse(filename, projection);
}

void Projection::saveToTextFile(std::string filename) const
{
    std::ofstream f(filename.c_str());
    int channels = projection.channels();
    int nRows = projection.rows;
    int nCols = projection.cols * channels;

    int i, j;
    const uchar *p;
    for (i = 0; i < nRows; ++i) {
        p = projection.ptr<uchar>(i);
        for (j = 0; j < nCols; ++j) {
            f << (int) p[j] << " ";
        }
        f << std::endl;
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
Projection::grepPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, cv::Mat areaToKeep,
                           bool extraCondition(const pcl::PointXYZ &minPt, const pcl::PointXYZ &maxPt,
                                               const pcl::PointXYZ &spaceSize,
                                               uchar actCell, const pcl::PointXYZ &point)
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    size_t taggedPointCounter = 0;
    result->resize(cloud->size());
    for (unsigned int pointIt = 0; pointIt < cloud->size(); pointIt++) {
        auto point = cloud->points[pointIt];
        cv::Vec2i posInMatrix = projectPoint(point);

        uchar toKeep = areaToKeep.at<uchar>(posInMatrix[1], posInMatrix[0]);
        uchar projectionCellValue = projection.at<uchar>(posInMatrix[1], posInMatrix[0]);
        if (toKeep > 0 && extraCondition(minPt, maxPt, spaceSize, projectionCellValue, point)) {
            result->points[taggedPointCounter] = pcl::PointXYZ(point.x, point.y, point.z);
            taggedPointCounter++;
        }
    }
    result->resize(taggedPointCounter);
    return result;
}

} // railroad
