/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Balázs Tábori
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_STRAIGHTTRACKPARTFINDERPROCESSOR_H
#define RAILROAD_STRAIGHTTRACKPARTFINDERPROCESSOR_H

#include <vector>
#include <map>
#include <string>
#include <set>

#include "ImageProcessor.h"

namespace railroad
{

const std::string THRESHOLD_CONTOUR = "threshold_contour";
const std::string CANNY_HOUGH = "canny_hough";
const std::string GENERALIZED_HOUGH = "generalized_hough";

class StraightTrackPartFinderProcessor : public ImageProcessor
{
    std::vector<cv::Mat> toShowObjects;
    std::string mode;

public:
    StraightTrackPartFinderProcessor(const std::string &mode)
        : mode(mode) {}
    cv::Mat execute() override;

    void setFragmentationAngle(int fragmentationAngle)
    {
        _fragmentationAngle = fragmentationAngle;
    }

private:
    int _fragmentationAngle;

    cv::Mat cutImageByCurvature(std::vector<std::pair<int, double> > curve);
    std::vector<std::pair<int, double>> findTrackContour();
    std::vector<std::pair<int, double>> findTrackHough();
    std::vector<std::pair<int, double>> findTrackGenHoughBallard();
    cv::Mat filterVegetation(const cv::Mat& image, std::map<int, std::set<int> >* orientationAngles = nullptr);

    double minLineSegmentsDistance(cv::Point2d a, cv::Point2d b, cv::Point2d c, cv::Point2d d);
    double maxLineSegmentsDistance(cv::Point2d a, cv::Point2d b, cv::Point2d c, cv::Point2d d);
    bool lineSegmentsIntersect(cv::Point2d a, cv::Point2d b, cv::Point2d c, cv::Point2d d);
    double pointLineSegmentDistance(cv::Point2d p, cv::Point2d a, cv::Point2d b);
    std::vector<cv::Point> getNeighbours(unsigned int start, const std::vector<std::pair<cv::Point, cv::Point> > &lines, std::vector<int> &visitedLines, int maxDist);
    bool isEnoughLines(int lineLength, int linesCount);
    void calculateLineOrientation(int angle, const std::pair<cv::Point, cv::Point>& line1, const std::pair<cv::Point, cv::Point>& line2, std::map<int, std::set<int> >* orientationAngles);
    std::vector<std::pair<int, double> > simplifyCurvatureData(std::vector<std::vector<std::pair<int, double> > > curvatureData);

    bool cutAlongYAxis() { return _image.size().width <= _image.size().height; }
};

} // railroad

#endif //RAILROAD_STRAIGHTTRACKPARTFINDERPROCESSOR_H
