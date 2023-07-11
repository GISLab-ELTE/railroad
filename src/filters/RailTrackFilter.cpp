/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Adalbert Demján
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <cmath>
#include <fstream>
#include <thread>
#include <future>
#include <algorithm>
#include <set>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "RailTrackFilter.h"
#include "AboveFilter.h"
#include "../dim2/Dim2Helper.h"
#include "../dim2/Projection.h"
#include "../helpers/LogHelper.h"

using namespace std;
using namespace cv;

namespace railroad
{

//Mathematical functions
bool RailTrackFilter::isTwoPointsEqual(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
{
    return (point1.x == point2.x && point1.y == point2.y && point1.z == point2.z);
}

float RailTrackFilter::calculate3DAngleBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2)
{
    float dotproduct = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    float magnitude1 = sqrt(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
    float magnitude2 = sqrt(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);
    float divresult = dotproduct / (magnitude1 * magnitude2);
    if (divresult > 1.0f)
        divresult = 1.0f;
    else if (divresult < -1.0f)
        divresult = -1.0f;
    float result = acos(divresult);
    return min(result, abs((float) CV_PI - result));
}

float RailTrackFilter::calculate2DAngleBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2)
{
    float dotproduct = vec1.x * vec2.x + vec1.y * vec2.y;
    float magnitude1 = sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
    float magnitude2 = sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
    float divresult = dotproduct / (magnitude1 * magnitude2);
    if (divresult > 1.0f)
        divresult = 1.0f;
    else if (divresult < -1.0f)
        divresult = -1.0f;
    float result = acos(divresult);
    return min(result, abs((float) CV_PI - result));
}

float RailTrackFilter::calculate2DDistanceBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2)
{
    float xdist = vec1.x - vec2.x;
    float ydist = vec1.y - vec2.y;
    return sqrt(xdist * xdist + ydist * ydist);
}

float RailTrackFilter::calculate2DDistanceBetweenLineAndPoint(const pcl::PointXYZ &line1point1,
                                                              const pcl::PointXYZ &line1point2,
                                                              const pcl::PointXYZ &line2point)
{
    float numerator = (line1point2.y - line1point1.y) * line2point.x;
    numerator -= (line1point2.x - line1point1.x) * line2point.y;
    numerator += line1point2.x * line1point1.y;
    numerator -= line1point2.y * line1point1.x;
    numerator = abs(numerator);
    return numerator / calculate2DDistanceBetweenTwoVectors(line1point1, line1point2);
}

float RailTrackFilter::calculate3DDistanceBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2)
{
    float xdist = vec1.x - vec2.x;
    float ydist = vec1.y - vec2.y;
    float zdist = vec1.z - vec1.z;
    return sqrt(xdist * xdist + ydist * ydist + zdist * zdist);
}

float RailTrackFilter::GetDirectionCoordinateValue(const pcl::PointXYZ &point)
{
    if (isDirectionX == true)
        return point.x;
    else
        return point.y;
}

//RailSegmentLine
RailSegmentLine::RailSegmentLine()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    linePoints = cloud;
}

RailSegmentLine::RailSegmentLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) : linePoints(
    new pcl::PointCloud<pcl::PointXYZ>)
{
    if (cloud->size() == 0) {
        linePoints = cloud;
        return;
    }

    double centroidX = 0.0f;
    double centroidY = 0.0f;
    double centroidZ = 0.0f;

    for (size_t i = 0; i < cloud->size(); ++i) {
        centroidX += cloud->points[i].x / cloud->size();
        centroidY += cloud->points[i].y / cloud->size();
        centroidZ += cloud->points[i].z / cloud->size();
    }
    averageHeight = centroidZ;

    centroid.x = centroidX;
    centroid.y = centroidY;
    centroid.z = centroidZ;

    deviationX = 0;
    deviationY = 0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        deviationX += pow(cloud->points[i].x - centroid.x, 2) / (cloud->size() - 1);
        deviationY += pow(cloud->points[i].y - centroid.y, 2) / (cloud->size() - 1);
    }

    deviationX = sqrt(deviationX);
    deviationY = sqrt(deviationY);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (abs(cloud->points[i].x - centroid.x) <= deviationX * 2
            || abs(cloud->points[i].y - centroid.y) <= deviationY * 2)
            linePoints->push_back(cloud->points[i]);
    }

    if (linePoints->size() == 0)
        return;

    pcl::PointXYZ maxXPoint = linePoints->points[0];
    pcl::PointXYZ maxYPoint = linePoints->points[0];
    pcl::PointXYZ minXPoint = linePoints->points[0];
    pcl::PointXYZ minYPoint = linePoints->points[0];

    for (size_t i = 0; i < linePoints->size(); ++i) {
        if (linePoints->points[i].x > maxXPoint.x) {
            maxXPoint = linePoints->points[i];
        } else if (linePoints->points[i].x < minXPoint.x) {
            minXPoint = linePoints->points[i];
        }

        if (linePoints->points[i].y > maxYPoint.y) {
            maxYPoint = linePoints->points[i];
        } else if (linePoints->points[i].y < minYPoint.y) {
            minYPoint = linePoints->points[i];
        }
    }

    if (RailTrackFilter::calculate2DDistanceBetweenTwoVectors(maxXPoint, maxYPoint) >
        RailTrackFilter::calculate2DDistanceBetweenTwoVectors(maxXPoint, minYPoint)) {
        startPoint = maxXPoint;
        endPoint = maxYPoint;
    } else {
        startPoint = maxXPoint;
        endPoint = minYPoint;
    }
    startPoint.z = averageHeight;
    endPoint.z = averageHeight;

    directionVec.x = endPoint.x - startPoint.x;
    directionVec.y = endPoint.y - startPoint.y;
    directionVec.z = averageHeight;

    LOG(trace) << "LINE: ";
    LOG(trace) << "\togsize: " << cloud->size();
    LOG(trace) << "\tsize: " << linePoints->size();
    LOG(trace) << "\tcenter: " << centroid.x << " " << centroid.y << " " << centroid.z;
    LOG(trace) << "\tdirvec: " << directionVec;
    LOG(trace) << "\tstartPoint :" << startPoint;
    LOG(trace) << "\tendPoint :" << endPoint;
    LOG(trace) << "\tdevX: " << deviationX;
    LOG(trace) << "\tdevY: " << deviationY;
}

pcl::PointXYZ RailSegmentLine::GetDirectionVec()
{
    return directionVec;
}

pcl::PointXYZ RailSegmentLine::GetCentroid()
{
    return centroid;
}

float RailSegmentLine::GetAverageHeight()
{
    return averageHeight;
}

pcl::PointXYZ RailSegmentLine::GetStartPoint()
{
    return startPoint;
}

pcl::PointXYZ RailSegmentLine::GetEndPoint()
{
    return endPoint;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RailSegmentLine::GetCloudPoints()
{
    return linePoints;
}

//RailSeedLine
RailSeedLine::RailSeedLine(float startX, float startY, float endX, float endY,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    linePoints = cloud;

    startPoint2D.x = startX;
    startPoint2D.y = startY;
    startPoint2D.z = 0;
    endPoint2D.x = endX;
    endPoint2D.y = endY;
    endPoint2D.z = 0;
    centroid2D.x = (startX + endX) / 2;
    centroid2D.y = (startY + endY) / 2;
    centroid2D.z = 0;
    directionVec2D.x = endX - startX;
    directionVec2D.y = endY - startY;
    directionVec2D.z = 0;

    averageHeight = 0;
    for (size_t i = 0; i < linePoints->size(); ++i) {
        averageHeight += linePoints->points[i].z / linePoints->size();
    }

    LOG(trace) << "LINE: ";
    LOG(trace) << "\tsize: " << linePoints->size();
    LOG(trace) << "\tcenter: " << centroid2D.x << " " << centroid2D.y;
    LOG(trace) << "\tdirvec: " << directionVec2D;
    LOG(trace) << "\tstartPoint :" << startPoint2D;
    LOG(trace) << "\tendPoint :" << endPoint2D;
    LOG(trace) << "\taverageHeight: " << averageHeight;
}

pcl::PointXYZ RailSeedLine::GetStartPoint2D()
{
    return startPoint2D;
}

pcl::PointXYZ RailSeedLine::GetEndPoint2D()
{
    return endPoint2D;
}

pcl::PointXYZ RailSeedLine::GetDirectionVec2D()
{
    return directionVec2D;
}

pcl::PointXYZ RailSeedLine::GetCentroid2D()
{
    return centroid2D;
}

float RailSeedLine::GetAverageHeight()
{
    return averageHeight;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RailSeedLine::GetCloudPoints()
{
    return linePoints;
}

//Railtrack detection
void RailTrackFilter::calculateStartAndDirection()
{
    pcl::PointXYZ maxX = originalCloud->points[0];
    pcl::PointXYZ maxY = originalCloud->points[0];
    pcl::PointXYZ minX = originalCloud->points[0];
    pcl::PointXYZ minY = originalCloud->points[0];
    for (auto it = originalCloud->begin(); it != originalCloud->end(); it++) {
        if (it->x > maxX.x)
            maxX = *it;
        else if (it->x < minX.x)
            minX = *it;

        if (it->y > maxY.y)
            maxY = *it;
        else if (it->x < minY.y)
            minY = *it;
    }

    if (calculate2DDistanceBetweenTwoVectors(maxX, minX) > calculate2DDistanceBetweenTwoVectors(maxY, minY)) {
        isDirectionX = true;
        start = floor(minX.x);
    } else {
        isDirectionX = false;
        start = floor(minY.y);
    }
    LOG(trace) << "isDirectionX: " << isDirectionX;
    LOG(trace) << "start: " << start;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RailTrackFilter::courseClassification()
{
    float maxheightvalue = 0;
    int mostcommonheight = 0;
    for (auto it = originalCloud->begin(); it != originalCloud->end(); it++) {
        float directionCoordValue = GetDirectionCoordinateValue(*it);
        if (directionCoordValue <= start + classificationCut) {
            mapHeightsFrequency[it->z]++;
            if (mapHeightsFrequency[it->z] > maxheightvalue) {
                maxheightvalue = mapHeightsFrequency[it->z];
                mostcommonheight = it->z;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr trackbed(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto it = originalCloud->begin(); it != originalCloud->end(); it++) {
        if (abs(mostcommonheight - it->z) <= 0.75f) {
            float directionCoordValue = GetDirectionCoordinateValue(*it);
            if (directionCoordValue <= start + classificationCut) {
                trackbed->push_back(*it);
            }
        }
    }
    return trackbed;
}

cv::Mat RailTrackFilter::calculateCovMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> pointIdxVec)
{
    cv::Mat result(3, 3, CV_64FC1);
    float xmean = 0;
    float ymean = 0;
    float zmean = 0;
    for (size_t i = 0; i < pointIdxVec.size(); ++i) {
        xmean += cloud->points[pointIdxVec[i]].x / pointIdxVec.size();
        ymean += cloud->points[pointIdxVec[i]].y / pointIdxVec.size();
        zmean += cloud->points[pointIdxVec[i]].z / pointIdxVec.size();
    }

    float cov_xx = 0;
    float cov_xy = 0;
    float cov_xz = 0;
    float cov_yy = 0;
    float cov_yz = 0;
    float cov_zz = 0;
    for (size_t i = 0; i < pointIdxVec.size(); ++i) {
        float xval = cloud->points[pointIdxVec[i]].x - xmean;
        float yval = cloud->points[pointIdxVec[i]].y - ymean;
        float zval = cloud->points[pointIdxVec[i]].z - zmean;
        cov_xx += xval * xval / (pointIdxVec.size() - 1);
        cov_xy += xval * yval / (pointIdxVec.size() - 1);
        cov_xz += xval * zval / (pointIdxVec.size() - 1);
        cov_yy += yval * yval / (pointIdxVec.size() - 1);
        cov_yz += yval * zval / (pointIdxVec.size() - 1);
        cov_zz += zval * zval / (pointIdxVec.size() - 1);
    }

    result.at<float>(0, 0) = cov_xx;
    result.at<float>(0, 1) = cov_xy;
    result.at<float>(0, 2) = cov_xz;
    result.at<float>(1, 0) = cov_xy;
    result.at<float>(1, 1) = cov_yy;
    result.at<float>(1, 2) = cov_yz;
    result.at<float>(2, 0) = cov_xz;
    result.at<float>(2, 1) = cov_yz;
    result.at<float>(2, 2) = cov_zz;

    return result;
}

float RailTrackFilter::calculate95HeightPercentile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    vector<float> heights;
    for (size_t i = 0; i < cloud->size(); ++i) {
        heights.push_back(cloud->points[i].z);
    }

    sort(heights.begin(), heights.end());
    float percentileIndex = (heights.size() * 0.95) - 1;
    int pIndexRounded = (int) roundf(percentileIndex);
    if (roundf(percentileIndex) == percentileIndex) {
        return (heights.at(pIndexRounded) + heights.at(pIndexRounded + 1)) / 2;
    } else {
        return heights.at(pIndexRounded);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
RailTrackFilter::calculateSeedPointsForPoint(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr trackBed,
                                             const pcl::PointXYZ &point,
                                             float H95)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    vector<int> pointIdxVec;
    vector<float> pointRadiusSquaredDistance;
    octree.radiusSearch(point, 0.1, pointIdxVec, pointRadiusSquaredDistance);
    cv::Mat covMatrix = calculateCovMatrix(trackBed, pointIdxVec);
    cv::Mat evec = Mat::zeros(3, 3, CV_64FC1);
    cv::Mat eval = Mat::zeros(1, 3, CV_64FC1);
    cv::eigen(covMatrix, eval, evec);
    if (eval.at<float>(0, 2) >= 0.01) {
        for (size_t i = 0; i < pointIdxVec.size(); ++i) {
            if (trackBed->points[pointIdxVec[i]].z >= H95) {
                result->push_back(trackBed->points[pointIdxVec[i]]);
            }
        }
    }

    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RailTrackFilter::calculateRailH95(pcl::PointCloud<pcl::PointXYZ>::Ptr trackBed)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> trackBedOctree(0.75f);
    trackBedOctree.setInputCloud(trackBed);
    trackBedOctree.addPointsFromInputCloud();

    LOG(trace) << "trackBed size: " << trackBed->size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr railH95(new pcl::PointCloud<pcl::PointXYZ>);
    float H95 = calculate95HeightPercentile(trackBed);
    for (size_t i = 0; i < trackBed->size(); ++i) {
        *railH95 += *(calculateSeedPointsForPoint(trackBedOctree, trackBed, trackBed->points[i], H95));
    }
    return railH95;
}

void RailTrackFilter::calculatePotentialRailLinesWithHoughTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr railH95,
                                                                    vector<RailSeedLine> &railSeedLines)
{
    Projection railH95proj(railH95, gridCellSize);
    cv::Mat railH95mat = railH95proj.getImage();

    vector<Mat> toShowObjects;
    toShowObjects.push_back(railH95mat);

    vector<Vec4i> lines;
    int threshold = 50;
    while (lines.size() < 2 && threshold != 0) {
        cv::HoughLinesP(railH95mat, lines, 1, CV_PI / 180, threshold, 0, 10);
        threshold -= 5;
    }
    LOG(trace) << "threshold: " << threshold + 5;
    LOG(trace) << "cvHough size: " << lines.size();
    vector<cv::Mat> railMats;

    for (const auto &vec : lines) {
        cv::Mat lineMat = Mat::zeros(railH95mat.size(), CV_8UC1);
        cv::Mat startEndPointMat = Mat::zeros(railH95mat.size(), CV_8UC1);
        cv::line(lineMat, Point(vec[0], vec[1]), Point(vec[2], vec[3]), 255, 1, cv::LINE_AA);
        toShowObjects.push_back(lineMat);
        railMats.push_back(lineMat);
    }

    saveImagesAsGridAndAloneAlso("railtrackprogress.png", toShowObjects);

    LOG(trace) << "segment line start";
    for (size_t i = 0; i < railMats.size(); ++i) {
        RailSeedLine railLine(lines[i][0], lines[i][1], lines[i][2], lines[i][3],
                              railH95proj.grepPointCloud(railH95, railMats[i]));
        railSeedLines.push_back(railLine);
    }
    LOG(trace) << "segment line finishes";
}

void RailTrackFilter::findRailPairs(vector<RailSeedLine> &railSeedLines,
                                    vector<pair<RailSeedLine, RailSeedLine> > &railPairs)
{
    for (auto it = railSeedLines.begin(); it != railSeedLines.end() - 1 && it != railSeedLines.end(); it++) {
        for (auto it2 = it + 1; it2 != railSeedLines.end(); ++it2) {
            if (calculate2DAngleBetweenTwoVectors(it->GetDirectionVec2D(), it2->GetDirectionVec2D()) <=
                5 * CV_PI / 180 &&
                abs(calculate2DDistanceBetweenLineAndPoint(it->GetStartPoint2D(), it->GetEndPoint2D(),
                                                           it2->GetEndPoint2D()) * gridCellSize - 1.455) <= 0.1) {
                LOG(trace) << "found rail pair: " << it->GetDirectionVec2D() << " " << it2->GetDirectionVec2D();
                railPairs.push_back(pair<RailSeedLine, RailSeedLine>(*it, *it2));
                break;
            }
        }
    }

    LOG(trace) << "Pairs: " << railPairs.size();
    for (size_t i = 0; i < railPairs.size(); ++i) {
        LOG(trace) << "\tpair distance: "
                   << calculate2DDistanceBetweenLineAndPoint(
                       railPairs[i].first.GetStartPoint2D(),
                       railPairs[i].first.GetEndPoint2D(),
                       railPairs[i].second.GetEndPoint2D()) * gridCellSize << endl;
    }
}

RailSegmentLine RailTrackFilter::growRail(railroad::RailSegmentLine &rail, vector<RailSegmentLine> &previousIterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud = rail.GetCloudPoints();

    pcl::PointCloud<pcl::PointXYZ>::Ptr candidateRailSegment(new pcl::PointCloud<pcl::PointXYZ>);
    LOG(trace) << "segment size: " << segmentCloud->size();
    for (auto it = segmentCloud->begin(); it != segmentCloud->end(); ++it) {
        vector<int> pointIdxVec;
        vector<float> pointRadiusSquaredDistance;
        cloudOctree.radiusSearch(*it, growSize, pointIdxVec, pointRadiusSquaredDistance);

        for (size_t i = 0; i < pointIdxVec.size(); ++i) {
            pcl::PointXYZ directionVector;
            directionVector.x = originalCloud->points[pointIdxVec[i]].x - it->x;
            directionVector.y = originalCloud->points[pointIdxVec[i]].y - it->y;

            if (abs(rail.GetAverageHeight() - originalCloud->points[pointIdxVec[i]].z) < 0.05
                && calculate2DAngleBetweenTwoVectors(directionVector, rail.GetDirectionVec()) < 5 * CV_PI / 180) {
                if (isPointInCloud(originalCloud->points[pointIdxVec[i]], candidateRailSegment) == false
                    && isPointInCloud(originalCloud->points[pointIdxVec[i]], segmentCloud) == false) {
                    bool isInPrevIts = false;
                    for (size_t j = 0; j < previousIterations.size() && isInPrevIts != true; ++j) {
                        isInPrevIts |= isPointInCloud(originalCloud->points[pointIdxVec[i]],
                                                      previousIterations[j].GetCloudPoints());
                    }

                    if (isInPrevIts == false)
                        candidateRailSegment->push_back(originalCloud->points[pointIdxVec[i]]);
                }
            }
        }
    }

    return RailSegmentLine(candidateRailSegment);
}

bool RailTrackFilter::isPointInCloud(const pcl::PointXYZ &point, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (auto it = cloud->begin(); it != cloud->end(); ++it) {
        if (isTwoPointsEqual(point, *it) == true)
            return true;
    }

    return false;
}

RailSegmentLine RailTrackFilter::removeDuplicates(RailSeedLine &rail)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZ>);

    LOG(trace) << "rail size: " << rail.GetCloudPoints()->size();

    for (auto it = rail.GetCloudPoints()->begin(); it != rail.GetCloudPoints()->end(); ++it) {
        if (isPointInCloud(*it, segmentCloud) == false)
            segmentCloud->push_back(*it);
    }

    LOG(trace) << "segment size: " << segmentCloud->size();

    return RailSegmentLine(segmentCloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RailTrackFilter::growRailPair(pair<RailSeedLine, RailSeedLine> &railPair)
{
    bool noMoreFound = false;
    std::future<RailSegmentLine> filledLine1 = std::async( &railroad::RailTrackFilter::removeDuplicates, this, std::ref(railPair.first));
    std::future<RailSegmentLine> filledLine2 = std::async( &railroad::RailTrackFilter::removeDuplicates, this, std::ref(railPair.second));
    RailSegmentLine grow1 = filledLine1.get();
    RailSegmentLine grow2 = filledLine2.get();

    std::vector<RailSegmentLine> previousIterations1(prevItNum);
    std::vector<RailSegmentLine> previousIterations2(prevItNum);

    pcl::PointCloud<pcl::PointXYZ>::Ptr growingRail(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr growingRail1 = grow1.GetCloudPoints();
    pcl::PointCloud<pcl::PointXYZ>::Ptr growingRail2 = grow2.GetCloudPoints();
    RailSegmentLine railBefore;

    LOG(debug) << "fill";
    while (noMoreFound == false) {
        std::future<RailSegmentLine> candidateLine1 = std::async( &railroad::RailTrackFilter::growRail, this, std::ref(grow1),
                                                                 std::ref(previousIterations1));
        std::future<RailSegmentLine> candidateLine2 = std::async( &railroad::RailTrackFilter::growRail, this, std::ref(grow2),
                                                                 std::ref(previousIterations2));

        RailSegmentLine candidateRailSegmentLine1 = candidateLine1.get();
        RailSegmentLine candidateRailSegmentLine2 = candidateLine2.get();
        LOG(trace) << "grow both rails";

        if (candidateRailSegmentLine1.GetCloudPoints()->size() < 10 ||
            candidateRailSegmentLine2.GetCloudPoints()->size() < 10)
            break;

        if (calculate2DAngleBetweenTwoVectors(candidateRailSegmentLine1.GetDirectionVec(),
                                              candidateRailSegmentLine2.GetDirectionVec()) <= 10 * CV_PI / 180) {
            for (int i = prevItNum - 1; i > 0; --i) {
                previousIterations1[i] = previousIterations1[i - 1];
                previousIterations2[i] = previousIterations2[i - 1];
            }
            previousIterations1[0] = grow1;
            previousIterations2[0] = grow2;
            grow1 = candidateRailSegmentLine1;
            grow2 = candidateRailSegmentLine2;
            *growingRail1 += *grow1.GetCloudPoints();
            *growingRail2 += *grow2.GetCloudPoints();
            LOG(trace) << "rail grown";
        } else {
            noMoreFound = true;
        }
    }

    *growingRail = *growingRail1 + *growingRail2;

    return growingRail;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RailTrackFilter::process()
{
    stopTimeMeasure();

    if(_useTempSeed)
        originalCloud = _seedHelper.getTempSeedCloud();
    else
        pcl::copyPointCloud(*_cloud, *originalCloud);

    startTimeMeasure();

    if (originalCloud->size() == 0)
        return originalCloud;

    LOG(debug) << "start";
    calculateStartAndDirection();
    pcl::PointCloud<pcl::PointXYZ>::Ptr trackBed = courseClassification();
    LOG(debug) << "classification";

    if (trackBed->size() == 0)
        return trackBed;

    pcl::PointCloud<pcl::PointXYZ>::Ptr railH95 = calculateRailH95(trackBed);
    LOG(debug) << "Trackbed size: " << trackBed->size();
    LOG(debug) << "railH95 size: " << railH95->size();

    vector<RailSeedLine> railSeedLines;
    calculatePotentialRailLinesWithHoughTransform(railH95, railSeedLines);

    vector<pair<RailSeedLine, RailSeedLine> > railPairs;
    findRailPairs(railSeedLines, railPairs);

    cloudOctree.setInputCloud(originalCloud);
    cloudOctree.addPointsFromInputCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto itPairs = railPairs.begin(); itPairs != railPairs.end(); ++itPairs) {
        *result += *growRailPair(*itPairs);
    }

    return result;
}
}
