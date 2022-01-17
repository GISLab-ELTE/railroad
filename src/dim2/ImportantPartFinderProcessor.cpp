/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <vector>
#include <numeric>

#include <opencv2/imgproc.hpp>

#include "ImportantPartFinderProcessor.h"
#include "Dim2Helper.h"
#include "../helpers/LogHelper.h"

using namespace cv;
using namespace std;

namespace railroad
{

Mat ImportantPartFinderProcessor::calculateDirectionWithLeveling(int levelNum)
{
    vector<Vec2d> levelAnglesWithWeight;

    for (int a = 0; a < levelNum; a++) {
        int loverThreshold = 255 / (levelNum + 1) * a;
        int upperThreshold = 255 / (levelNum + 1) * (a + 2);
        LOG(trace) << loverThreshold << " " << upperThreshold;
        Mat levelImage = intervalThreshold(_image, loverThreshold, upperThreshold);
        toShowObjects.push_back(levelImage);
        vector<Vec4i> lines;
        HoughLinesP(levelImage, lines, 1, CV_PI / 180, 50, 200, 50);

        if (!lines.empty()) {
            double mean = calculateLinesAnglesMean(lines);
            LOG(trace) << "Mean:" << mean * 180 / CV_PI << " with " << 1.0f / lines.size() << " (" << lines.size()
                       << ") weight ";
            levelAnglesWithWeight.push_back(Vec2d(mean, 1.0f / lines.size()));
        }

    }
    double fullMean = calculateWeightedMeanAngle(levelAnglesWithWeight);

    LOG(trace) << "Global mean:" << fullMean * 180 / CV_PI;

    Mat mainDirection = Mat::zeros(_image.size(), CV_8UC1);
    Point center(_image.cols / 2, _image.rows / 2);
    Point dir((int) (cos(fullMean) * 2000), (int) (sin(fullMean) * 2000));
    line(mainDirection, center - dir, center + dir, Scalar(255), 5);

    Mat imageWithMainDirection = _image.clone();
    bitwise_or(imageWithMainDirection, mainDirection, imageWithMainDirection);
    toShowObjects.push_back(imageWithMainDirection);

    return mainDirection;
}

Mat ImportantPartFinderProcessor::intervalThreshold(const cv::Mat &image, int loverThreshold, int upperThreshold) const
{
    Mat levelImage;
    threshold(image, levelImage, loverThreshold, 255, THRESH_TOZERO);
    threshold(levelImage, levelImage, upperThreshold, 255, THRESH_TOZERO_INV);
    return levelImage;
}

double ImportantPartFinderProcessor::calculateLinesAnglesMean(const vector<Vec4i> &lines)
{
    double mean;
    Mat levelImageWithLines = Mat::zeros(_image.size(), CV_8UC1);
    vector<double> angles;
    angles.resize(lines.size());
    for (unsigned int b = 0; b < lines.size(); b++) {
        Point p1(lines[b][0], lines[b][1]);
        Point p2(lines[b][2], lines[b][3]);
        angles[b] = calcDirection(p1, p2);
        line(levelImageWithLines, p1, p2, Scalar(255), 1);
    }
    toShowObjects.push_back(levelImageWithLines);
    mean = std::accumulate(angles.begin(), angles.end(), 0.0) / angles.size();
    return mean;
}

double ImportantPartFinderProcessor::calculateWeightedMeanAngle(const vector<Vec2d> &levelAnglesWithWeight) const
{
    double fullMean = 0;
    double mainDivisor = 0;
    for (auto obj : levelAnglesWithWeight) {
        fullMean += obj[0] * obj[1];
        mainDivisor += obj[1];
    }
    fullMean /= mainDivisor;
    return fullMean;
}


cv::Mat ImportantPartFinderProcessor::grepImageByCenterlines(cv::Mat centerlinesImage, int distance)
{
    cv::Mat areaToSeeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(distance, distance));
    cv::Mat areaToSee;
    dilate(centerlinesImage, areaToSee, areaToSeeElement);
    toShowObjects.push_back(areaToSee);
    Mat interestingArea;
    bitwise_and(_image, areaToSee, interestingArea);
    toShowObjects.push_back(interestingArea);

    return interestingArea;
}


vector<Point> ImportantPartFinderProcessor::calculateConvexHull(Mat image)
{
    Mat workingImage;
    morphologyEx(image, workingImage, MORPH_CLOSE, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(50, 50)));
    threshold(image, workingImage, 1, 255, THRESH_BINARY);
    toShowObjects.push_back(workingImage.clone());

    //CONTOUER SEARCH
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(workingImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

    int biggestContour = -1;
    for (unsigned int i = 0; i < contours.size(); i++) {
        if (biggestContour == -1 || contours[biggestContour].size() < contours[i].size()) {
            biggestContour = i;
        }
    }

    vector<vector<Point> > hull(1);
    //CONVEX HULL
    convexHull(Mat(contours[biggestContour]), hull[0], false);
    Mat convexHull = Mat::zeros(workingImage.size(), CV_8UC1);
    drawContours(convexHull, hull, 0, 255, 1, 1, vector<Vec4i>(), 0, Point());
    toShowObjects.push_back(convexHull);
    LOG(debug) << "Hull size:" << hull[0].size();

    vector<Point> points = hull[0];
    bool isRunning;
    do {
        //LOG(trace) << points;
        unsigned int a;
        isRunning = false;
        for (a = 0; a < points.size(); a++) {
            double deg = calcAngle(
                points[(a) % points.size()],
                points[(a + 1) % points.size()],
                points[(a + 2) % points.size()]
            );
            /*LOG(trace) << points[(a) % points.size()] << " "
                       << points[(a+1) % points.size()] << " "
                       << points[(a+2) % points.size()] << " "
                       << deg << " " << (deg < 0.05?"DELETE":"");
            */if (deg < 0.05) {
                isRunning = true;
                break;
            }
        }
        if (isRunning) {
            points.erase(points.begin() + (a + 1) % points.size());
        }
    } while (isRunning && points.size() > 4);

    LOG(debug) << "Hull size after reduction:" << points.size();

    Mat biggestContourImage = image.clone();
    polylines(biggestContourImage, points, true, Scalar(255), 1, cv::LINE_AA, 0);
    int counter = 0;
    for (const auto &p : hull[0]) {
        circle(biggestContourImage, p, 10, Scalar(50 + counter * 25), cv::FILLED);
        counter++;
    }
    toShowObjects.push_back(biggestContourImage);
    return points;
}


Mat ImportantPartFinderProcessor::calculateSkeleton(Mat image)
{
    vector<Point> points = calculateConvexHull(image);

    Mat filedConvexHull = Mat::zeros(image.size(), CV_8UC1);
    fillConvexPoly(filedConvexHull, points.data(), points.size(), Scalar(255));
    toShowObjects.push_back(filedConvexHull);

    Mat skeleton = calculateSkeletonInner(filedConvexHull.clone());

    toShowObjects.push_back(skeleton);
    // SKELETON CALCULATION
    return skeleton;
}

Mat ImportantPartFinderProcessor::calculateSkeletonInner(Mat img)
{
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do {
        cv::erode(img, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(img, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(img);

        done = (cv::countNonZero(img) == 0);
    } while (!done);
    return skel;
}


Mat ImportantPartFinderProcessor::calculateVoronoi(Mat image)
{
    vector<Point> points = calculateConvexHull(image);
    //Source: https://www.learnopencv.com/delaunay-triangulation-and-voronoi-diagram-using-opencv-c-python/

    Size size = image.size();
    Rect rect(0, 0, size.width, size.height);
    Subdiv2D subdiv(rect);
    for (auto it = points.begin(); it != points.end(); it++) {
        subdiv.insert(*it);
    }

    vector<vector<Point2f> > facets;
    vector<Point2f> centers;
    subdiv.getVoronoiFacetList(vector<int>(), facets, centers);

    vector<Point> ifacet;
    vector<vector<Point> > ifacets(1);

    Mat workingImage = image.clone();

    Mat imagePlustCenterline = Mat::zeros(image.size(), CV_8UC1);;
    Mat centerlineImage = Mat::zeros(image.size(), CV_8UC1);
    for (size_t i = 0; i < facets.size(); i++) {
        ifacet.resize(facets[i].size());
        for (size_t j = 0; j < facets[i].size(); j++)
            ifacet[j] = facets[i][j];

        ifacets[0] = ifacet;
        polylines(centerlineImage, ifacets, true, Scalar(255), 1, cv::LINE_AA, 0);
        circle(workingImage, points[i], 10, Scalar(255), cv::FILLED);
    }
    bitwise_or(workingImage, centerlineImage, workingImage);
    toShowObjects.push_back(workingImage.clone());
    return centerlineImage;
}

Mat ImportantPartFinderProcessor::execute()
{
    toShowObjects.push_back(_image);
    Mat mainDirection;
    std::string filename;
    if (mode == ANGLE) {
        mainDirection = calculateDirectionWithLeveling(4);
        filename = "angle";
    } else if (mode == SKELETON) {
        mainDirection = calculateSkeleton(_image);
        filename = "skeleton";
    } else if (mode == VORONOI) {
        mainDirection = calculateVoronoi(_image);
        filename = "voronoi";
    } else {
        LOG(error) << "Unknown mode in ImportantPartFinderProcessor execute (" << mode << ") ";
        mainDirection = Mat(_image.size(), CV_8UC1, 1);
        filename = "unknown";
    }

    Mat interestingArea = grepImageByCenterlines(mainDirection, (int) (50 / 0.25));
    saveImagesAsGridAndAloneAlso(filename + ".png", toShowObjects);

    return interestingArea;
}

ImportantPartFinderProcessor::ImportantPartFinderProcessor(Mode mode) : mode(mode)
{

}

} // railroad