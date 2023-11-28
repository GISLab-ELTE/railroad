/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Balázs Tábori
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <vector>
#include <map>
#include <utility>
#include <set>

#include <opencv2/imgproc/imgproc.hpp>

#include "StraightTrackPartFinderProcessor.h"
#include "Dim2Helper.h"
#include "../helpers/LogHelper.h"

using namespace cv;
using namespace std;

namespace railroad
{

Mat StraightTrackPartFinderProcessor::execute()
{
    toShowObjects.push_back(_image);

    Mat interestingArea;

    if (mode == THRESHOLD_CONTOUR) {
        vector<pair<int, double> > curve = findTrackContour();
        interestingArea = cutImageByCurvature(curve);
    } else if (mode == CANNY_HOUGH) {
        vector<pair<int, double> > curve = findTrackHough();
        interestingArea = cutImageByCurvature(curve);
    } else if (mode == GENERALIZED_HOUGH) {
        vector<pair<int, double> > curve = findTrackGenHoughBallard();
        interestingArea = cutImageByCurvature(curve);
    } else {
        LOG(error) << "Unknown mode in StraightTrackPartFinderProcessor execute (" << mode << ") ";
        interestingArea = Mat(_image.size(), CV_8UC1, 1);
    }

    saveImagesAsGridAndAloneAlso(mode + ".png", toShowObjects);

    return interestingArea;
}

Mat StraightTrackPartFinderProcessor::cutImageByCurvature(vector<pair<int, double> > curve)
{
    vector<int> cuttingPoints;

    for (unsigned int i = 0; i + 1 < curve.size(); i++) {
        double angle1= curve[i].second;

        bool nearlyStraight = true;

        i++;

        while (nearlyStraight && i < curve.size()) {
            double angle2 = curve[i].second;

            nearlyStraight = abs(angle1 - angle2) < _fragmentationAngle;

            if (!nearlyStraight) {
                cuttingPoints.push_back(curve[i-1].first);
                i--;
                LOG(info) << "Cutting at: " << curve[i-1].first;
            } else {
                i++;
            }
        }
    }

    Mat returnImage;
    Mat logImage = _image.clone();

    if (cutAlongYAxis()) {
        returnImage = Mat::zeros(_image.rows, 1, CV_8UC1);

        for (unsigned int i = 0; i < cuttingPoints.size(); i++) {
            returnImage.at<uchar>(cuttingPoints[i], 0) = 255;
            line(logImage, Point(0, cuttingPoints[i]), Point(_image.size().width, cuttingPoints[i]), Scalar(255), 2, LINE_AA);
        }
    } else {
        returnImage = Mat::zeros(1, _image.cols, CV_8UC1);

        for (unsigned int i = 0; i < cuttingPoints.size(); i++) {
            returnImage.at<uchar>(0, cuttingPoints[i]) = 255;
            line(logImage, Point(cuttingPoints[i], 0), Point(cuttingPoints[i], _image.size().height), Scalar(255), 2, LINE_AA);
        }
    }

    toShowObjects.push_back(logImage);

    return returnImage;
}

vector<pair<int, double> > StraightTrackPartFinderProcessor::findTrackContour()
{
    Mat thresholdImage;

    double thresholdValue = threshold(_image, thresholdImage, 0, 255, THRESH_BINARY + THRESH_OTSU);
    int stepCount = 0;
    int maxStep = 15;
    bool enoughLines = false;

    vector<pair<int, pair<Point, Point> > > maxLenContourLines;

    while (!enoughLines && stepCount <= maxStep) {
        if (stepCount != 0) {
            threshold(_image, thresholdImage, thresholdValue, 255, THRESH_BINARY);
        }

        toShowObjects.push_back(thresholdImage.clone());

        vector<vector<Point> > contours;
        findContours(thresholdImage, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double maxLinesLen = 0;
        int maxId = -1;
        vector<vector<pair<int, pair<Point, Point> > > > contourLines(contours.size());

        for (unsigned int i = 0; i < contours.size(); i++) {
            double arcLen = arcLength(contours[i], true);
            int contourLen = contours[i].size();

            if (arcLen != 0) {
                Mat tempImage = Mat::zeros(_image.size(), CV_8UC1);
                drawContours(tempImage, contours, i, Scalar(255));

                double linesLen = 0;

                vector<Vec4i> houghLines;
                HoughLinesP(tempImage, houghLines, 1, CV_PI / 180, ((contourLen / arcLen) * 50), 10, 10);

                if (!houghLines.empty()) {
                    map<int, vector<pair<Point, Point> > > linesByAngle;

                    for (unsigned int j = 0; j < houghLines.size(); j++) {
                        Vec4i l = houghLines[j];

                        double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;

                        Point p1(l[0], l[1]);
                        Point p2(l[2], l[3]);

                        linesByAngle[round(angle)].push_back(make_pair(p1, p2));
                    }

                    for (auto const& item : linesByAngle) {
                        int const& angle = item.first;
                        auto const& lines = item.second;

                        if (lines.size() < 2) {
                            continue;
                        }

                        vector<int> visitedLines;

                        for (unsigned int j = 0; j < lines.size() - 1; j++) {
                            if(find(visitedLines.begin(), visitedLines.end(), j) != visitedLines.end()) {
                                continue;
                            }

                            vector<Point> neighbouringPoints = getNeighbours(j, lines, visitedLines, 10);
                            visitedLines.push_back(j);

                            if (neighbouringPoints.size() > 0) {
                                neighbouringPoints.push_back(lines[j].first);
                                neighbouringPoints.push_back(lines[j].second);

                                auto compareLambda = cutAlongYAxis() ? [](Point lhs, Point rhs) {return lhs.y < rhs.y;} : [](Point lhs, Point rhs) {return lhs.x < rhs.x;};

                                auto minMax = minmax_element(begin(neighbouringPoints), end(neighbouringPoints), compareLambda);

                                Point min = *minMax.first;
                                Point max = *minMax.second;

                                contourLines[i].push_back(make_pair(angle, make_pair(min, max)));

                                linesLen += hypot(max.x - min.x, max.y - min.y);
                            } else {
                                contourLines[i].push_back(make_pair(angle, lines[j]));
                                linesLen += hypot(lines[j].second.x - lines[j].first.x, lines[j].second.y - lines[j].first.y);
                            }
                        }
                    }
                }

                if (linesLen > maxLinesLen) {
                    maxLinesLen = linesLen;
                    maxId = i;
                    maxLenContourLines = contourLines[maxId];
                }
            }
        }

        Mat maxContourImage = Mat::zeros(_image.size(), CV_8UC1);
        drawContours(maxContourImage, contours, maxId, Scalar(255));
        toShowObjects.push_back(maxContourImage);

        thresholdValue += 10;
        stepCount++;

        enoughLines = isEnoughLines(maxLinesLen, contourLines[maxId].size());
    }

    Mat maxLinesImage = Mat::zeros(_image.size(), CV_8UC1);
    for (unsigned int i = 0; i < maxLenContourLines.size(); i++) {
        line(maxLinesImage, maxLenContourLines[i].second.first, maxLenContourLines[i].second.second, Scalar(255), 1, LINE_AA);
    }

    toShowObjects.push_back(maxLinesImage);

    vector<vector<pair<int, double> > > curveHelper(cutAlongYAxis() ? _image.size().height : _image.size().width);

    for (unsigned int i = 0; i < maxLenContourLines.size(); i++) {
        int const& angle = maxLenContourLines[i].first;
        auto const& line = maxLenContourLines[i].second;

        int min, max;
        double lineLen = hypot(line.first.x - line.second.x, line.first.y - line.second.y);

        if (cutAlongYAxis()) {
            if (line.first.y < line.second.y) {
                min = line.first.y;
                max = line.second.y;
            } else {
                min = line.second.y;
                max = line.first.y;
            }
        } else {
            if (line.first.x < line.second.x) {
                min = line.first.x;
                max = line.second.x;
            } else {
                min = line.second.x;
                max = line.first.x;
            }
        }

        for (int j = min; j <= max; j++) {
            curveHelper[j].push_back(make_pair(angle, lineLen));
        }
    }

    vector<pair<int, double> > curve = simplifyCurvatureData(curveHelper);

    return curve;
}

vector<pair<int, double> > StraightTrackPartFinderProcessor::findTrackHough()
{
    vector<vector<pair<int, double> > > curveHelper(cutAlongYAxis() ? _image.size().height : _image.size().width);
    Mat filteredImage = filterVegetation(_image);

    Mat cannyImage;
    Canny(filteredImage, cannyImage, 50, 200);
    toShowObjects.push_back(cannyImage);

    vector<Vec4i> houghLines;
    HoughLinesP(cannyImage, houghLines, 1, CV_PI / 180, 50, 200, 50);

    if (!houghLines.empty()) {
        Mat houghImage = Mat::zeros(_image.size(), CV_8UC1);
        map<int, vector<pair<Point, Point> > > linesByAngle;

        for (unsigned int j = 0; j < houghLines.size(); j++) {
            Vec4i l = houghLines[j];

            double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;

            Point p1(l[0], l[1]);
            Point p2(l[2], l[3]);

            line(houghImage, p1, p2, Scalar(255), 1, LINE_AA);

            linesByAngle[round(angle)].push_back(make_pair(p1, p2));
        }

        toShowObjects.push_back(houghImage);

        Mat rectangleImage = Mat::zeros(_image.size(), CV_8UC1);

        for (auto const& item : linesByAngle) {
            int const& angle = item.first;
            auto const& lines = item.second;

            vector<int> visitedLines;

            for (unsigned int j = 0; j < lines.size(); j++) {
                if(find(visitedLines.begin(), visitedLines.end(), j) != visitedLines.end()) {
                    continue;
                }

                vector<Point> neighbouringPoints = getNeighbours(j, lines, visitedLines, 20);
                visitedLines.push_back(j);

                if (neighbouringPoints.size() > 0) {
                    neighbouringPoints.push_back(lines[j].first);
                    neighbouringPoints.push_back(lines[j].second);

                    RotatedRect rRect = minAreaRect(neighbouringPoints);

                    Point2f vertices[4];
                    rRect.points(vertices);
                    for (int i = 0; i < 4; i++) {
                        line(rectangleImage, vertices[i], vertices[(i+1)%4], Scalar(255), 1, LINE_AA);
                    }

                    int min;
                    int max;

                    if (cutAlongYAxis()) {
                        auto minMax = minmax_element(begin(vertices), end(vertices), [](Point2f lhs, Point2f rhs) {return lhs.y < rhs.y;});

                        min = round(minMax.first->y);
                        max = round(minMax.second->y);
                    } else {
                        auto minMax = minmax_element(begin(vertices), end(vertices), [](Point2f lhs, Point2f rhs) {return lhs.x < rhs.x;});

                        min = round(minMax.first->x);
                        max = round(minMax.second->x);
                    }


                    if (min < 0) {
                        min = 0;
                    }
                    if (max >= _image.size().height) {
                        max = _image.size().height - 1;
                    }

                    for (int i = min; i <= max; i++) {
                        curveHelper[i].push_back(make_pair(angle, rRect.size.area()));
                    }

                } else {
                    line(rectangleImage, lines[j].first, lines[j].second, Scalar(255), 1, LINE_AA);

                    int min, max;
                    double lineLen = hypot(lines[j].first.x - lines[j].second.x, lines[j].first.y - lines[j].second.y);

                    if (cutAlongYAxis()) {
                        if (lines[j].first.y < lines[j].second.y) {
                            min = lines[j].first.y;
                            max = lines[j].second.y;
                        } else {
                            min = lines[j].second.y;
                            max = lines[j].first.y;
                        }
                    } else {
                        if (lines[j].first.x < lines[j].second.x) {
                            min = lines[j].first.x;
                            max = lines[j].second.x;
                        } else {
                            min = lines[j].second.x;
                            max = lines[j].first.x;
                        }
                    }

                    for (int i = min; i <= max; i++) {
                        curveHelper[i].push_back(make_pair(angle, lineLen));
                    }
                }
            }

        }

        toShowObjects.push_back(rectangleImage);
    }

    vector<pair<int, double> > curve = simplifyCurvatureData(curveHelper);

    return curve;
}

vector<pair<int, double> > StraightTrackPartFinderProcessor::findTrackGenHoughBallard()
{
	map<int, set<int> > angleInfos;
    Mat filteredImage = filterVegetation(_image, &angleInfos);

    Ptr<GeneralizedHoughBallard> genHoughBallard = createGeneralizedHoughBallard();
    genHoughBallard->setMinDist(1000);
    genHoughBallard->setLevels(360);
    genHoughBallard->setDp(2);
    genHoughBallard->setMaxBufferSize(1000);
    genHoughBallard->setCannyLowThresh(50);
    genHoughBallard->setCannyHighThresh(200);

    Mat genHoughImage = _image.clone();
    Mat returnImage = cutAlongYAxis() ? Mat(_image.rows, 1, CV_8UC1) : Mat(1, _image.cols, CV_8UC1);
    vector<vector<pair<int, double> > > curveHelper(cutAlongYAxis() ? _image.size().height : _image.size().width);

    for (auto const& angleInfo : angleInfos) {
        int angle = angleInfo.first;

        for (int distance : angleInfo.second) {
            int radius = (distance / 2) / sin((_fragmentationAngle / 2) * (CV_PI / 180));
            vector<Vec4f> position;

            Mat templateImage = Mat::zeros(Size(3 * radius, 3 * radius), CV_8UC1);
            circle(templateImage, Point(1.5 * radius, 1.5 * radius), radius, Scalar(255));

            int startX = 1.5 * radius + radius * cos(angle * CV_PI / 180);
            int startY = 1.5 * radius + radius * sin(angle * CV_PI / 180);

            int endX = 1.5 * radius + radius * cos((angle + _fragmentationAngle) * CV_PI / 180);
            int endY = 1.5 * radius + radius * sin((angle + _fragmentationAngle) * CV_PI / 180);

            int minX = startX < endX ? startX : endX;
            int maxX = startX < endX ? endX : startX;

            int minY = startY < endY ? startY : endY;
            int maxY = startY < endY ? endY : startY;

            Rect myROI(minX, minY, maxX - minX + 1, maxY - minY + 1);
			Mat croppedImage = templateImage(myROI);

            vector<Point> arcPoints;
            findNonZero(croppedImage, arcPoints);

            genHoughBallard->setVotesThreshold(3.0 * arcPoints.size());
            genHoughBallard->setTemplate(croppedImage);
            genHoughBallard->detect(filteredImage, position);

            for (size_t i = 0; i < position.size(); ++i) {
                Point2f pos(position[i][0], position[i][1]);

                RotatedRect rect;
                rect.center = pos;
                rect.size = Size2f(croppedImage.cols, croppedImage.rows);

                Point2f pts[4];
                rect.points(pts);

                if (pts[0].x < 0 || pts[0].y < 0 || pts[0].x >= _image.size().width || pts[0].y >= _image.size().height ||
                    pts[1].x < 0 || pts[1].y < 0 || pts[1].x >= _image.size().width || pts[1].y >= _image.size().height ||
                    pts[2].x < 0 || pts[2].y < 0 || pts[2].x >= _image.size().width || pts[2].y >= _image.size().height ||
                    pts[3].x < 0 || pts[3].y < 0 || pts[3].x >= _image.size().width || pts[3].y >= _image.size().height) {
                    break;
                }

                line(genHoughImage, pts[0], pts[1], Scalar(255), 3);
                line(genHoughImage, pts[1], pts[2], Scalar(255), 3);
                line(genHoughImage, pts[2], pts[3], Scalar(255), 3);
                line(genHoughImage, pts[3], pts[0], Scalar(255), 3);

                for (unsigned int j = 0; j < arcPoints.size(); j++) {
                    int y = pts[1].y + arcPoints[j].y;
                    int x = pts[1].x + arcPoints[j].x;

                    if (y > 0 && x > 0 && y < _image.size().height && x < _image.size().width) {
                        genHoughImage.at<uchar>(y, x) = 255;
                    }
                }

                int min, max;

                if (cutAlongYAxis()) {
                    min = pts[1].y < pts[2].y ? pts[1].y : pts[2].y;
                    max = pts[0].y < pts[3].y ? pts[3].y : pts[0].y;
                } else {
                    min = pts[1].x < pts[2].x ? pts[1].x : pts[2].x;
                    max = pts[0].x < pts[3].x ? pts[3].x : pts[0].x;
                }

                double angleStep = (double)_fragmentationAngle / (max - min);

                for (int j = min; j <= max; j++) {
                    curveHelper[j].push_back(make_pair(round((angle % 180) + ((j - min) * angleStep)), 1));
                }
            }
        }
    }

    toShowObjects.push_back(genHoughImage);

    vector<pair<int, double> > curve = simplifyCurvatureData(curveHelper);

    return curve;
}

Mat StraightTrackPartFinderProcessor::filterVegetation(const Mat& image, map<int, set<int> >* orientationAngles)
{
    LOG(info) << "Starting vegatation filtering";

    Mat thresholdImage;

    double thresholdValue = threshold(image, thresholdImage, 0, 255, THRESH_BINARY + THRESH_OTSU);
    int stepCount = 0;
    int maxStep = 15;
    bool enoughLines = false;

    map<int, vector<pair<Point, Point> > > maxLenContourLinesByAngle;
    vector<vector<Point> > maxLenContours;
    int maxLenContoursId = -1;

    while (!enoughLines && stepCount <= maxStep) {
        if (stepCount != 0) {
            threshold(image, thresholdImage, thresholdValue, 255, THRESH_BINARY);
        }

        vector<vector<Point> > contours;
        findContours(thresholdImage, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double maxLinesLen = 0;
        int maxId = -1;
        vector<vector<pair<int, pair<Point, Point> > > > contourLines(contours.size());

        for (unsigned int i = 0; i < contours.size(); i++) {
            double arcLen = arcLength(contours[i], true);
            int contourLen = contours[i].size();

            if (arcLen != 0) {
                Mat tempImage = Mat::zeros(image.size(), CV_8UC1);
                drawContours(tempImage, contours, i, Scalar(255));

                double linesLen = 0;

                vector<Vec4i> houghLines;
                HoughLinesP(tempImage, houghLines, 1, CV_PI / 180, ((contourLen / arcLen) * 50), 10, 10);

                map<int, vector<pair<Point, Point> > > linesByAngle;

                if (!houghLines.empty()) {
                    for (unsigned int j = 0; j < houghLines.size(); j++) {
                        Vec4i l = houghLines[j];

                        double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;

                        Point p1(l[0], l[1]);
                        Point p2(l[2], l[3]);

                        linesByAngle[round(angle)].push_back(make_pair(p1, p2));
                    }

                    for (auto const& item : linesByAngle) {
                        int const& angle = item.first;
                        auto const& lines = item.second;

                        if (lines.size() < 2) {
                            continue;
                        }

                        vector<int> visitedLines;

                        for (unsigned int j = 0; j < lines.size() - 1; j++) {
                            if(find(visitedLines.begin(), visitedLines.end(), j) != visitedLines.end()) {
                                continue;
                            }

                            vector<Point> neighbouringPoints = getNeighbours(j, lines, visitedLines, 10);
                            visitedLines.push_back(j);

                            if (neighbouringPoints.size() > 0) {
                                neighbouringPoints.push_back(lines[j].first);
                                neighbouringPoints.push_back(lines[j].second);

                                auto compareLambda = cutAlongYAxis() ? [](Point lhs, Point rhs) {return lhs.y < rhs.y;} : [](Point lhs, Point rhs) {return lhs.x < rhs.x;};

                                auto minMax = minmax_element(begin(neighbouringPoints), end(neighbouringPoints), compareLambda);

                                Point min = *minMax.first;
                                Point max = *minMax.second;

                                contourLines[i].push_back(make_pair(angle, make_pair(min, max)));

                                linesLen += hypot(max.x - min.x, max.y - min.y);
                            } else {
                                contourLines[i].push_back(make_pair(angle, lines[j]));
                                linesLen += hypot(lines[j].second.x - lines[j].first.x, lines[j].second.y - lines[j].first.y);
                            }
                        }
                    }
                }

                if (linesLen > maxLinesLen) {
                    maxLinesLen = linesLen;
                    maxId = i;
                    maxLenContourLinesByAngle = linesByAngle;
                    maxLenContours = contours;
                    maxLenContoursId = i;
                }
            }
        }

        thresholdValue += 10;
        stepCount++;

        enoughLines = isEnoughLines(maxLinesLen, contourLines[maxId].size());
    }

    Mat filteredImage = Mat::zeros(image.size(), CV_8UC1);
    if (maxLenContoursId >= 0)
        drawContours(filteredImage, maxLenContours, maxLenContoursId, Scalar(255), 1);
    else
        LOG(error) << "maxLenContoursId was not set";

    toShowObjects.push_back(filteredImage);

    if (orientationAngles != nullptr) {
        for (auto const& x : maxLenContourLinesByAngle) {
            if (maxLenContourLinesByAngle.find(x.first + _fragmentationAngle) != maxLenContourLinesByAngle.end()) {
                for (auto const& line1 : maxLenContourLinesByAngle[x.first]) {
                    for (auto const& line2 : maxLenContourLinesByAngle[x.first + _fragmentationAngle]) {
                        calculateLineOrientation(x.first, line1, line2, orientationAngles);
                    }
                }
            } else if (maxLenContourLinesByAngle.find(x.first + _fragmentationAngle - 1) != maxLenContourLinesByAngle.end()) {
                for (auto const& line1 : maxLenContourLinesByAngle[x.first]) {
                    for (auto const &line2 : maxLenContourLinesByAngle[x.first + _fragmentationAngle - 1]) {
                        calculateLineOrientation(x.first, line1, line2, orientationAngles);
                    }
                }
            } else if (maxLenContourLinesByAngle.find(x.first + _fragmentationAngle + 1) != maxLenContourLinesByAngle.end()) {
                for (auto const& line1 : maxLenContourLinesByAngle[x.first]) {
                    for (auto const &line2 : maxLenContourLinesByAngle[x.first + _fragmentationAngle + 1]) {
                        calculateLineOrientation(x.first, line1, line2, orientationAngles);
                    }
                }
            }
        }
    }

    LOG(info) << "Finished vegatation filtering";

    return filteredImage;
}

vector<Point> StraightTrackPartFinderProcessor::getNeighbours(unsigned int start, const vector<pair<Point, Point> > &lines, vector<int> &visitedLines, int maxDist)
{
    vector<Point> neighbouringPoints;

    for (unsigned int i = start + 1; i < lines.size(); i++) {
        if(find(visitedLines.begin(), visitedLines.end(), i) != visitedLines.end()) {
            continue;
        }

        if (minLineSegmentsDistance(lines[start].first, lines[start].second, lines[i].first, lines[i].second) < maxDist) {
            visitedLines.push_back(i);

            neighbouringPoints.push_back(lines[i].first);
            neighbouringPoints.push_back(lines[i].second);

            vector<Point> neighboursOfNeighbour = getNeighbours(i, lines, visitedLines, maxDist);
            neighbouringPoints.insert(neighbouringPoints.end(), begin(neighboursOfNeighbour), end(neighboursOfNeighbour));
        }

    }

    return neighbouringPoints;
}


double StraightTrackPartFinderProcessor::minLineSegmentsDistance(Point2d a, Point2d b, Point2d c, Point2d d)
{
    if (lineSegmentsIntersect(a, b, c, d)) {
        return 0;
    }

    vector<double> distances;

    distances.push_back(pointLineSegmentDistance(a, c, d));
    distances.push_back(pointLineSegmentDistance(b, c, d));
    distances.push_back(pointLineSegmentDistance(c, a, b));
    distances.push_back(pointLineSegmentDistance(d, a, b));

    return *min_element(distances.begin(), distances.end());
}


double StraightTrackPartFinderProcessor::maxLineSegmentsDistance(Point2d a, Point2d b, Point2d c, Point2d d)
{
    if (lineSegmentsIntersect(a, b, c, d)) {
        return 0;
    }

    vector<double> distances;

    distances.push_back(pointLineSegmentDistance(a, c, d));
    distances.push_back(pointLineSegmentDistance(b, c, d));
    distances.push_back(pointLineSegmentDistance(c, a, b));
    distances.push_back(pointLineSegmentDistance(d, a, b));

    return *max_element(distances.begin(), distances.end());
}

bool StraightTrackPartFinderProcessor::lineSegmentsIntersect(Point2d a, Point2d b, Point2d c, Point2d d)
{
    double dx1 = b.x - a.x;
    double dy1 = b.y - a.y;
    double dx2 = d.x - c.x;
    double dy2 = d.y - c.y;
    double delta = dx2 * dy1 - dy2 * dx1;

    if (delta == 0 ) {
        return false;
    }

    double s = (dx1 * (c.y - a.y) + dy1 * (a.x - c.x)) / delta;
    double t = (dx2 * (a.y - c.y) + dy2 * (c.x - a.x)) / (-delta);

    return (0 <= s) && (s <= 1) && (0 <= t) && (t <= 1);
}

double StraightTrackPartFinderProcessor::pointLineSegmentDistance(Point2d p, Point2d a, Point2d b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    if (dx == dy && dx == 0) {
        return hypot(p.x - a.x, p.y - a.y);
    }

    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);

    if (t < 0) {
        dx = p.x - a.x;
        dy = p.y - a.y;
    } else if (t > 1) {
        dx = p.x - b.x;
        dy = p.y - b.y;
    } else {
        double near_x = a.x + t * dx;
        double near_y = a.y + t * dy;
        dx = p.x - near_x;
        dy = p.y - near_y;
    }

    return hypot(dx, dy);
}

bool StraightTrackPartFinderProcessor::isEnoughLines(int linesLength, int linesCount)
{
    int maxLinesCount = (cutAlongYAxis() ? _image.size().height : _image.size().width) / 60;
    double minLinesLength = (cutAlongYAxis() ? _image.size().height : _image.size().width) * 2 * 0.75;

    return linesLength >= minLinesLength && linesCount <= maxLinesCount;
}

void StraightTrackPartFinderProcessor::calculateLineOrientation(int angle, const pair<Point, Point>& line1, const pair<Point, Point>& line2, map<int, set<int>> *orientationAngles) {
    int distanceThreshold = 100;

    int minDistance = round(minLineSegmentsDistance(line1.first, line1.second, line2.first, line2.second) / 100) * 100;
    int maxDistance = round(maxLineSegmentsDistance(line1.first, line1.second, line2.first, line2.second) / 100) * 100;

    if (minDistance > distanceThreshold) {
        (*orientationAngles)[(angle + 90) % 360].insert(minDistance);
        (*orientationAngles)[(angle + 90 + 180) % 360].insert(minDistance);
    }
    if (maxDistance > distanceThreshold) {
        (*orientationAngles)[(angle + 90) % 360].insert(maxDistance);
        (*orientationAngles)[(angle + 90 + 180) % 360].insert(maxDistance);
    }
}

vector<pair<int, double> > StraightTrackPartFinderProcessor::simplifyCurvatureData(vector<vector<pair<int, double> > > curvatureData) {
    vector<pair<int, double> > simplifiedCurvature;

    for (unsigned int i = 0; i < curvatureData.size(); i++) {
        double sum = 0;
        double n = 0;

        for (unsigned j = 0; j < curvatureData[i].size(); j++) {
            sum += curvatureData[i][j].first * curvatureData[i][j].second;
            n += curvatureData[i][j].second;
        }

        if (n != 0) {
            double avgAngle = sum / n;
            simplifiedCurvature.push_back(make_pair(i, avgAngle));
        }
    }

    return simplifiedCurvature;
}

} // railroad
