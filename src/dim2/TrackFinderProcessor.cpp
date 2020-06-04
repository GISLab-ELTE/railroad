/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TrackFinderProcessor.h"
#include "Dim2Helper.h"
#include "../helpers/LogHelper.h"

using namespace std;
using namespace cv;

namespace railroad
{

double TrackFinderProcessor::isOutlierComparedToTheSurroundingPoints(Mat mat, int centerI, int centerJ, int padSize,
                                                                     int maxDistance)
{
    int nRows = mat.rows;
    int nCols = mat.cols;
    uchar *p;
    int i, j;
    double sum = 0;
    int counter = 0;
    int similarPointCounter = 0;
    auto centerValue = mat.at<uchar>(centerI, centerJ);
    for (i = max(centerI - padSize, 0); i < min(centerI + padSize, nRows); ++i) {
        p = mat.ptr<uchar>(i);
        for (j = max(centerJ - padSize, 0); j < min(centerJ + padSize, nCols); ++j) {
            if (p[j] > 0 && (centerI != i || centerJ != j)) {
                sum += p[j];
                counter++;

                if (abs((int) p[j] - (int) centerValue) <= maxDistance) {
                    similarPointCounter++;
                }
            }
        }
    }
    return similarPointCounter < 6;
}

Mat TrackFinderProcessor::houghP(Mat input, int threshold, double minLineLength, double maxLineGap)
{
    vector<Vec4i> lines2;
    HoughLinesP(input, lines2, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);

    Mat moreInterestingPartsMask = Mat::zeros(input.size(), CV_8UC1);
    for (const auto &vec : lines2) {
        line(moreInterestingPartsMask, Point(vec[0], vec[1]), Point(vec[2], vec[3]), 255, 1, CV_AA);
    }
    return moreInterestingPartsMask;
}


cv::Mat TrackFinderProcessor::execute()
{
    vector<Mat> toShowObjects;
    toShowObjects.push_back(_image);

    Mat workImage = _image;
    threshold(workImage, workImage, 110, 255, THRESH_TOZERO);
    toShowObjects.push_back(workImage.clone());

    dilate(workImage, workImage, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    //workImage = skeleton(workImage);
    //toShowObjects.push_back(workImage);

    Mat houghPLinesMat = houghP(workImage, 50, 100, 50);
    Mat imageWithHoughPLines;
    bitwise_or(workImage, houghPLinesMat, imageWithHoughPLines);
    toShowObjects.push_back(imageWithHoughPLines);

    toShowObjects.push_back(houghPLinesMat);

    Mat moreInterestingPartsMask2 = houghP(workImage, 50, 300, 50);
    morphologyEx(moreInterestingPartsMask2, moreInterestingPartsMask2, MORPH_CLOSE,
                 getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    toShowObjects.push_back(moreInterestingPartsMask2.clone());

    int nRows = moreInterestingPartsMask2.rows;
    int nCols = moreInterestingPartsMask2.cols;
    int nonZero1 = countNonZero(moreInterestingPartsMask2);
    int i, j;
    uchar *p;
    for (i = 0; i < nRows; ++i) {
        p = moreInterestingPartsMask2.ptr<uchar>(i);
        for (j = 0; j < nCols; ++j) {
            if (p[j] > 0) {
                if (isOutlierComparedToTheSurroundingPoints(workImage, i, j, 5, 4)) {
                    p[j] = 0;
                }
            }
        }
    }
    LOG(debug) << "From " << nonZero1 << " to " << countNonZero(moreInterestingPartsMask2);
    toShowObjects.push_back(moreInterestingPartsMask2);

    saveImagesAsGridAndAloneAlso("interestingPartProgress.png", toShowObjects);
    return moreInterestingPartsMask2;
}

void TrackFinderProcessor::setGridSize(float size)
{
    gridCellSize = size;
}

} // railroad
