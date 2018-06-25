/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_TRACKFINDERPROCESSOR_H
#define RAILROAD_TRACKFINDERPROCESSOR_H

#include <vector>

#include "ImageProcessor.h"

namespace railroad
{

class TrackFinderProcessor : public ImageProcessor
{
    std::vector<cv::Mat> toShowObjects;
    float gridCellSize;

public:
    cv::Mat execute();
    void setGridSize(float size);

private:
    double isOutlierComparedToTheSurroundingPoints(cv::Mat mat, int centerI, int centerJ, int padSize, int maxDistance);
    cv::Mat houghP(cv::Mat input, int threshold, double minLineLength, double maxLineGap);
};

} // railroad

#endif //RAILROAD_TRACKFINDERPROCESSOR_H
