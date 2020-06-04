/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>

#include <opencv2/core/core.hpp>

#include "AboveFilter.h"
#include "../dim2/Dim2Helper.h"
#include "../dim2/ImportantPartFinderProcessor.h"
#include "../dim2/TrackFinderProcessor.h"
#include "../dim2/Projection.h"
#include "../helpers/PCLHelper.h"

using namespace std;
using namespace cv;

namespace railroad
{

double isOutlierComparedToTheSurroundingPoints(Mat mat, int centerI, int centerJ, int padSize, int maxDistance);

pcl::PointCloud<pcl::PointXYZ>::Ptr AboveFilter::processInner(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    Projection projectionObj(cloud, gridCellSize);
    Mat projection = projectionObj.getImage();

    stopTimeMeasure();
    imwriteWithInverse("projection.png", projection);
    startTimeMeasure();

    TrackFinderProcessor trackFinderProcessor;
    trackFinderProcessor.setInputImage(projection.clone());
    trackFinderProcessor.setGridSize(gridCellSize);

    Mat importantParts = trackFinderProcessor.execute();

    pcl::PointCloud<pcl::PointXYZ>::Ptr result =
        projectionObj.
            grepPointCloud(cloud, importantParts,
                           [](
                               const pcl::PointXYZ &minPt,
                               const pcl::PointXYZ &maxPt,
                               const pcl::PointXYZ &spaceSize,
                               uchar actCellValue,
                               const pcl::PointXYZ &point) -> bool {
                               uchar actPointVal = (uchar) ((point.z - minPt.z) * 255 / spaceSize.z);
                               return 110 < actPointVal && actCellValue - actPointVal < 3;
                           });

    return result;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr AboveFilter::process()
{
    stopTimeMeasure();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*_cloud, *cloud);
    startTimeMeasure();

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (int a = 0; a < 2; a++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder = processInner(cloud);

        stopTimeMeasure();
        Projection projectionObj(cloud_cylinder, 0.25);
        projectionObj.saveToFile("cylinder" + std::to_string(a) + ".png");
        startTimeMeasure();

        *result += *cloud_cylinder;

        cloud = diffPointClouds(cloud, cloud_cylinder);

        stopTimeMeasure();
        Projection projectionObj2(cloud, 0.25);
        projectionObj2.saveToFile("afterCylinder" + std::to_string(a) + ".png");
        startTimeMeasure();
    }
    return result;
}

} // railroad
