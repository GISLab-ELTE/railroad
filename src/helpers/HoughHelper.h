/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */


#ifndef RAILROAD_HOUGHHELPER_H
#define RAILROAD_HOUGHHELPER_H

#include <vector>
#include <deque>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

namespace railroad
{
class HoughHelper
{
public:
    // parameter space discretization and allocation of voting space
    HoughHelper(const pcl::PointXYZ &minP, const pcl::PointXYZ &maxP, float var_dx,
                unsigned int sphereGranularity);
    // accumulator array A
    std::vector<unsigned int> VotingSpace;

    size_t num_b;
    // x' and y'
    float dx, max_x;
    size_t num_x;

    // returns the line with most votes (rc = number of votes)
    unsigned int getLine(Eigen::Vector3f *point, Eigen::Vector3f *direction);
    // add all points from point cloud to voting space
    void add(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);
    // subtract all points from point cloud to voting space
    void subtract(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);

    // direction vectors
    std::vector<pcl::PointXYZ> vertices;
    // surface triangles
    std::deque<unsigned int> triangles;
    // creates the directions by subdivisions of icosahedron
    void fromIcosahedron(unsigned int subDivisions = 4);

private:
    // add or subtract (add==false) one point from voting space
    void pointVote(const pcl::PointXYZ &point, bool add);

    // creates nodes and edges of icosahedron
    void getIcosahedron();
    // one subdivision step
    void subDivide();
    // make vectors nondirectional and unique
    void makeUnique();
};
} // railroad

#endif //RAILROAD_HOUGHHELPER_H
