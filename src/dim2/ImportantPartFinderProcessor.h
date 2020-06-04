/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_IMPORTANTPARTFINDERPORCESSOR_H
#define RAILROAD_IMPORTANTPARTFINDERPORCESSOR_H

#include <vector>
#include <string>

#include "ImageProcessor.h"

namespace railroad
{
class ImportantPartFinderProcessor : public ImageProcessor
{
public:
    enum Mode
    {
        VORONOI,
        SKELETON,
        ANGLE
    };

private:

    std::vector<cv::Mat> toShowObjects;

    cv::Mat calculateDirectionWithLeveling(int levelNum);
    cv::Mat grepImageByCenterlines(cv::Mat centerlinesImage, int distance);

    double calculateWeightedMeanAngle(const std::vector<cv::Vec2d> &levelAnglesWithWeight) const;
    double calculateLinesAnglesMean(const std::vector<cv::Vec4i> &lines);
    cv::Mat intervalThreshold(const cv::Mat &image, int loverThreshold, int upperThreshold) const;

    std::vector<cv::Point> calculateConvexHull(cv::Mat image);

    cv::Mat calculateSkeletonInner(cv::Mat img);
    cv::Mat calculateSkeleton(cv::Mat image);

    cv::Mat calculateVoronoi(cv::Mat image);
    Mode mode;
public:
    //Available modes: skeleton, voronoi, angle
    explicit ImportantPartFinderProcessor(Mode mode);
    cv::Mat execute() override;


};

} // railroad

#endif //RAILROAD_IMPORTANTPARTFINDERPORCESSOR_H
