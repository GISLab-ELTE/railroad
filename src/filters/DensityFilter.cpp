/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <cmath>
#include <climits>
#include <fstream>

#include <pcl/common/common.h>

#include "DensityFilter.h"
#include "../helpers/LogHelper.h"

using namespace std;
using namespace cv;

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr DensityFilter::process()
{
    LOG(debug) << "Resolution: " << resolution << ", Threshold: " << threshold;

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*_cloud, min, max);

    int sizeX = (int) ceil((max.x - min.x) / resolution);
    int sizeY = (int) ceil((max.y - min.y) / resolution);
    int sizeZ = (int) ceil((max.z - min.z) / resolution);
    int sizes[] = {sizeX, sizeY, sizeZ};

    Mat density(3, sizes, CV_32SC1, Scalar(0));
    for (auto point : _cloud->points) {
        int posX = (int) floor((point.x - min.x) / resolution);
        int posY = (int) floor((point.y - min.y) / resolution);
        int posZ = (int) floor((point.z - min.z) / resolution);
        density.at<int>(posX, posY, posZ) += 1;
    }
    LOG(debug) << "Min: [" << min.x << ", " << min.y << ", " << min.z << "]";
    LOG(debug) << "Max: [" << max.x << ", " << max.y << ", " << max.z << "]";
    LOG(debug) << "Size: [" << sizeX << ", " << sizeY << ", " << sizeZ << "]";

    // Dump density map
    stopTimeMeasure();
    dump2DMap(density);
    startTimeMeasure();

    // Dense Pass
    unsigned int passCounter = 0;
    unsigned int changeCounter;
    unsigned int changeLimit;
    unsigned int aboveThreshold;
    unsigned short denseCounter;
    do {
        changeCounter = 0;
        ++passCounter;
        LOG(trace) << "Dense Pass #" << passCounter;

        aboveThreshold = 0;
        for (int x = 1; x < sizeX - 1; ++x)
            for (int y = 1; y < sizeY - 1; ++y)
                for (int z = 1; z < sizeZ - 1; ++z)
                    if (density.at<int>(x, y, z) >= threshold) {
                        ++aboveThreshold;
                        Range ranges[] = {
                            Range(x - 1, x + 2),
                            Range(y - 1, y + 2),
                            Range(z, z + 1)
                        };
                        Mat neighbourhood(density, ranges);

                        denseCounter = 0;
                        for (unsigned int i = 0; i < 3; ++i)
                            for (unsigned int j = 0; j < 3; ++j)
                                if (neighbourhood.at<int>(i, j, 0) >= threshold)
                                    ++denseCounter;

                        if (denseCounter < 3) {
                            density.at<int>(x, y, z) = -1;
                            ++changeCounter;
                        } else if (denseCounter > 6) {
                            density.at<int>(x, y, z) = -2;
                            ++changeCounter;
                        }
                    }

        changeLimit = aboveThreshold / 5;
        LOG(trace) << "Above threshold: " << aboveThreshold;
        LOG(trace) << "Changes: " << changeCounter << " (limit: " << changeLimit << ")";
    } while (changeCounter > changeLimit && passCounter < USHRT_MAX);

    // Small Bend Pass: 3x3x3
    passCounter = 0;
    unsigned short backDense, frontDense, leftDense, rightDense;
    do {
        changeCounter = 0;
        ++passCounter;
        LOG(trace) << "Small Bend Pass #" << passCounter;

        aboveThreshold = 0;
        for (int x = 1; x < sizeX - 1; ++x)
            for (int y = 1; y < sizeY - 1; ++y)
                for (int z = 1; z < sizeZ - 1; ++z)
                    if (density.at<int>(x, y, z) >= threshold) {
                        ++aboveThreshold;

                        backDense = 0;
                        for (int i = -1; i <= 1; ++i)
                            for (int k = -1; k <= 1; ++k) {
                                if (density.at<int>(x + i, y - 1, z + k) >= threshold)
                                    ++backDense;
                            }

                        frontDense = 0;
                        for (int i = -1; i <= 1; ++i)
                            for (int k = -1; k <= 1; ++k) {
                                if (density.at<int>(x + i, y + 1, z + k) >= threshold)
                                    ++frontDense;
                            }

                        leftDense = 0;
                        for (int j = -1; j <= 1; ++j)
                            for (int k = -1; k <= 1; ++k) {
                                if (density.at<int>(x - 1, y + j, z + k) >= threshold)
                                    ++leftDense;
                            }

                        rightDense = 0;
                        for (int j = -1; j <= 1; ++j)
                            for (int k = -1; k <= 1; ++k) {
                                if (density.at<int>(x + 1, y + j, z + k) >= threshold)
                                    ++rightDense;
                            }

                        if (!
                            ((leftDense >= 0 && rightDense >= 0 && leftDense < 3 && rightDense < 3) &&
                             (frontDense >= 0 && backDense >= 0 && frontDense < 3 && backDense < 3))) {
                            density.at<int>(x, y, z) = -3;
                            ++changeCounter;
                        }
                    }

        changeLimit = aboveThreshold / 10;
        LOG(trace) << "Above threshold: " << aboveThreshold;
        LOG(trace) << "Changes: " << changeCounter << " (limit: " << changeLimit << ")";
    } while (changeCounter > changeLimit && passCounter < USHRT_MAX);

    // Large Bend Pass: 5x3x3
    passCounter = 0;
    do {
        changeCounter = 0;
        ++passCounter;
        LOG(trace) << "Large Bend Pass #" << passCounter;

        aboveThreshold = 0;
        for (int x = 2; x < sizeX - 2; ++x)
            for (int y = 2; y < sizeY - 2; ++y)
                for (int z = 1; z < sizeZ - 1; ++z)
                    if (density.at<int>(x, y, z) >= threshold) {
                        ++aboveThreshold;

                        backDense = 0;
                        for (int i = -2; i <= 2; ++i)
                            for (int j = 1; j <= 2; ++j)
                                for (int k = -1; k <= 1; ++k) {
                                    if (density.at<int>(x + i, y - j, z + k) >= threshold)
                                        ++backDense;
                                }

                        frontDense = 0;
                        for (int i = -1; i <= 1; ++i)
                            for (int j = 1; j <= 2; ++j)
                                for (int k = -1; k <= 1; ++k) {
                                    if (density.at<int>(x + i, y + j, z + k) >= threshold)
                                        ++frontDense;
                                }

                        leftDense = 0;
                        for (int j = -1; j <= 1; ++j)
                            for (int i = 1; i <= 2; ++i)
                                for (int k = -1; k <= 1; ++k) {
                                    if (density.at<int>(x - i, y + j, z + k) >= threshold)
                                        ++leftDense;
                                }

                        rightDense = 0;
                        for (int j = -1; j <= 1; ++j)
                            for (int i = 1; i <= 2; ++i)
                                for (int k = -1; k <= 1; ++k) {
                                    if (density.at<int>(x + i, y + j, z + k) >= threshold)
                                        ++rightDense;
                                }

                        if (!
                            ((leftDense >= 0 && rightDense >= 0 && leftDense < 4 && rightDense < 4) &&
                             (frontDense >= 0 && backDense >= 0 && frontDense < 4 && backDense < 4))) {
                            density.at<int>(x, y, z) = -3;
                            ++changeCounter;
                        }
                    }

        changeLimit = aboveThreshold / 10;
        LOG(trace) << "Above threshold: " << aboveThreshold;
        LOG(trace) << "Changes: " << changeCounter << " (limit: " << changeLimit << ")";
    } while (changeCounter > changeLimit && passCounter < USHRT_MAX);

    // Around Bend Pass: 3x3x3
    /*passCounter = 0;
    unsigned short leftBottomDense, leftMiddleDense, leftTopDense,
      centerBottomDense, centerTopDense,
      rightBottomDense, rightMiddleDense, rightTopDense;
    do
    {
      changeCounter = 0;
      ++passCounter;
      LOG(trace) << "Around Bend Pass #" << passCounter;

      aboveThreshold = 0;
      for(int x = 1; x < sizeX - 1; ++x)
        for(int y = 1; y < sizeY - 1; ++y)
          for(int z = 1; z < sizeZ - 1; ++z)
            if(density.at<int>(x, y, z) >= threshold)
            {
              ++aboveThreshold;

              leftBottomDense = leftMiddleDense = leftTopDense =
                centerBottomDense = centerTopDense =
                rightBottomDense = rightMiddleDense = rightTopDense = 0;
              for(int k = -1; k <= 1; ++k)
              {
                if(density.at<int>(x - 1, y - 1, z + k) >= threshold)
                  ++leftBottomDense;

                if(density.at<int>(x - 1, y, z + k) >= threshold)
                  ++leftMiddleDense;

                if(density.at<int>(x - 1, y + 1, z + k) >= threshold)
                  ++leftTopDense;

                if(density.at<int>(x, y - 1, z + k) >= threshold)
                  ++centerBottomDense;

                if(density.at<int>(x, y + 1, z + k) >= threshold)
                  ++centerTopDense;

                if(density.at<int>(x + 1, y - 1, z + k) >= threshold)
                  ++rightBottomDense;

                if(density.at<int>(x + 1, y, z + k) >= threshold)
                  ++rightMiddleDense;

                if(density.at<int>(x + 1, y + 1, z + k) >= threshold)
                  ++rightTopDense;
              }

              if(leftBottomDense && !rightTopDense ||
                leftMiddleDense && !rightMiddleDense ||
                leftTopDense && !rightBottomDense ||
                centerBottomDense && !centerTopDense ||
                !leftBottomDense && rightTopDense ||
                !leftMiddleDense && rightMiddleDense ||
                !leftTopDense && rightBottomDense ||
                !centerBottomDense && centerTopDense)
              {
                density.at<int>(x, y, z) = -3;
                ++changeCounter;
              }
            }

      changeLimit = aboveThreshold / 5;
      LOG(trace) << "Above threshold: " << aboveThreshold;
      LOG(trace) << "Changes: " << changeCounter << " (limit: " << changeLimit << ")";
    }
    while(changeCounter > changeLimit && passCounter < USHRT_MAX);*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    int taggedPointCounter = 0;

    result->resize(_cloud->size());
    for (auto it = _cloud->begin(); it != _cloud->end(); ++it) {
        int posX = (int) floor((it->x - min.x) / resolution);
        int posY = (int) floor((it->y - min.y) / resolution);
        int posZ = (int) floor((it->z - min.z) / resolution);

        if (density.at<int>(posX, posY, posZ) >= threshold) {
            result->points[taggedPointCounter] = *it;
            ++taggedPointCounter;
        }
    }
    result->resize(taggedPointCounter);

    return result;
}

void DensityFilter::dump2DMap(Mat density)
{
    LOG(trace) << "Dumping density map";
    ofstream map("densitymap.txt");

    unsigned int sizeX = density.size[0];
    unsigned int sizeY = density.size[1];
    unsigned int sizeZ = density.size[2];

    for (unsigned int x = 0; x < sizeX; ++x) {
        for (unsigned int y = 0; y < sizeY; ++y) {
            unsigned int totalDensity = 0;
            for (unsigned int z = 0; z < sizeZ; ++z) {
                totalDensity += density.at<int>(x, y, z);
            }

            map << totalDensity << " ";
        }
        map << endl;
    }

    map.close();
    LOG(trace) << "Dumped density map";
}

} // railroad
