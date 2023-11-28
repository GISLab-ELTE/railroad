/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Adalbert Demján
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_RAILTRACKFILTER_H
#define RAILROAD_RAILTRACKFILTER_H

#include <mutex>

#include <pcl/octree/octree_search.h>

#include <opencv2/core.hpp>

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{
class RailSegmentLine
{
public:
    RailSegmentLine();
    RailSegmentLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PointXYZ GetDirectionVec();
    pcl::PointXYZ GetCentroid();
    float GetAverageHeight();
    pcl::PointXYZ GetStartPoint();
    pcl::PointXYZ GetEndPoint();

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudPoints();

private:
    pcl::PointXYZ directionVec;
    pcl::PointXYZ centroid;
    pcl::PointXYZ startPoint;
    pcl::PointXYZ endPoint;
    float deviationX;
    float deviationY;
    float averageHeight;

    pcl::PointCloud<pcl::PointXYZ>::Ptr linePoints;
};

class RailSeedLine
{
public:
    RailSeedLine(float startX, float startY, float endX, float endY, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PointXYZ GetStartPoint2D();
    pcl::PointXYZ GetEndPoint2D();
    pcl::PointXYZ GetDirectionVec2D();
    pcl::PointXYZ GetCentroid2D();
    float GetAverageHeight();

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudPoints();

private:
    pcl::PointXYZ startPoint2D;
    pcl::PointXYZ endPoint2D;
    pcl::PointXYZ directionVec2D;
    pcl::PointXYZ centroid2D;
    float averageHeight;

    pcl::PointCloud<pcl::PointXYZ>::Ptr linePoints;
};

class RailTrackFilter : public SingleResultCloudProcessor
{
public:
    RailTrackFilter(float gridCellSize = 0.25, const std::string &name = "RailTrackFilter")
        : SingleResultCloudProcessor(name), gridCellSize(gridCellSize), classificationCut(5), start(0), prevItNum(3),
          growSize(1), cloudOctree(0.25f), originalCloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
    }

    //Railtrack detection
    PointCloudPtr process();
    void calculateStartAndDirection();
    PointCloudPtr courseClassification();
    cv::Mat calculateCovMatrix(PointCloudPtr cloud, std::vector<int> pointIdxVec);
    float calculate95HeightPercentile(PointCloudPtr cloud);
    PointCloudPtr
    calculateSeedPointsForPoint(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, PointCloudPtr trackBed,
                                const pcl::PointXYZ &point, float H95);
    PointCloudPtr calculateRailH95(PointCloudPtr trackBed);
    void calculatePotentialRailLinesWithHoughTransform(PointCloudPtr railH95, std::vector<RailSeedLine> &railSeedLines);
    void findRailPairs(std::vector<RailSeedLine> &railSeedLines,
                       std::vector<std::pair<RailSeedLine, RailSeedLine> > &railPairs);

    RailSegmentLine growRail(RailSegmentLine &railToGrow, std::vector<RailSegmentLine> &previousIterations);
    RailSegmentLine removeDuplicates(RailSeedLine &rail);

    //Mathematical functions
    float GetDirectionCoordinateValue(const pcl::PointXYZ &point);
    PointCloudPtr growRailPair(std::pair<RailSeedLine, RailSeedLine> &railPair);

    static float calculate2DAngleBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2);
    static float calculate2DDistanceBetweenLineAndPoint(const pcl::PointXYZ &line1point1,
                                                        const pcl::PointXYZ &line1point2,
                                                        const pcl::PointXYZ &line2point);
    static float calculate3DAngleBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2);
    static float calculate2DDistanceBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2);
    static float calculate3DDistanceBetweenTwoVectors(const pcl::PointXYZ &vec1, const pcl::PointXYZ &vec2);
    static bool isTwoPointsEqual(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2);
    static bool isPointInCloud(const pcl::PointXYZ &point, PointCloudPtr cloud);


protected:
    std::map<float, int> mapHeightsFrequency;
    float gridCellSize;
    float classificationCut;
    bool isDirectionX;
    float start;
    int prevItNum;

    float growSize;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> cloudOctree;
    PointCloudPtr originalCloud;

};
} // railroad

#endif //RAILROAD_RAILTRACKFILTER_H
