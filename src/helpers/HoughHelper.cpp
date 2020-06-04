/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 *
 * https://www.ipol.im/pub/art/2017/208/
 */

#include "HoughHelper.h"

namespace railroad
{
static float roundToNearest(float num)
{
    return (num > 0.0f) ? floor(num + 0.5f) : ceil(num - 0.5f);
}

HoughHelper::HoughHelper(const pcl::PointXYZ &minP, const pcl::PointXYZ &maxP, float var_dx,
                         unsigned int sphereGranularity)
{
    this->fromIcosahedron(sphereGranularity);
    num_b = vertices.size();

    // compute x'y' discretization
    max_x = std::max(maxP.getVector3fMap().norm(), minP.getVector3fMap().norm());
    float range_x = 2.0f * max_x;
    dx = var_dx;
    if (dx == 0.0f) {
        dx = range_x / 64.0f;
    }
    num_x = 100;

    // allocate voting space
    VotingSpace.resize(num_x * num_x * num_b);
}

void HoughHelper::fromIcosahedron(unsigned int subDivisions)
{
    this->getIcosahedron();
    for (unsigned int i = 0; i < subDivisions; i++) {
        subDivide();
    }

    this->makeUnique();
}

void HoughHelper::getIcosahedron()
{
    vertices.clear();
    triangles.clear();
    float tau = 1.61803399f; // golden_ratio
    float norm = sqrt(1.0f + tau * tau);
    float v = 1.0f / norm;
    tau = tau / norm;

    pcl::PointXYZ vec;
    vec.x = -v;
    vec.y = tau;
    vec.z = 0.f;
    vertices.push_back(vec); // 1
    vec.x = v;
    vec.y = tau;
    vec.z = 0.f;
    vertices.push_back(vec); // 2
    vec.x = 0.f;
    vec.y = v;
    vec.z = -tau;
    vertices.push_back(vec); // 3
    vec.x = 0.f;
    vec.y = v;
    vec.z = tau;
    vertices.push_back(vec); // 4
    vec.x = -tau;
    vec.y = 0.f;
    vec.z = -v;
    vertices.push_back(vec); // 5
    vec.x = tau;
    vec.y = 0.f;
    vec.z = -v;
    vertices.push_back(vec); // 6
    vec.x = -tau;
    vec.y = 0.f;
    vec.z = v;
    vertices.push_back(vec); // 7
    vec.x = tau;
    vec.y = 0.f;
    vec.z = v;
    vertices.push_back(vec); // 8
    vec.x = 0.f;
    vec.y = -v;
    vec.z = -tau;
    vertices.push_back(vec); // 9
    vec.x = 0.f;
    vec.y = -v;
    vec.z = tau;
    vertices.push_back(vec); // 10
    vec.x = -v;
    vec.y = -tau;
    vec.z = 0.f;
    vertices.push_back(vec); // 11
    vec.x = v;
    vec.y = -tau;
    vec.z = 0.f;
    vertices.push_back(vec); // 12
    // add all edges of all triangles
    triangles.push_back(0);
    triangles.push_back(1);
    triangles.push_back(2); // 1
    triangles.push_back(0);
    triangles.push_back(1);
    triangles.push_back(3); // 2
    triangles.push_back(0);
    triangles.push_back(2);
    triangles.push_back(4); // 3
    triangles.push_back(0);
    triangles.push_back(4);
    triangles.push_back(6); // 4
    triangles.push_back(0);
    triangles.push_back(3);
    triangles.push_back(6); // 5
    triangles.push_back(1);
    triangles.push_back(2);
    triangles.push_back(5); // 6
    triangles.push_back(1);
    triangles.push_back(3);
    triangles.push_back(7); // 7
    triangles.push_back(1);
    triangles.push_back(5);
    triangles.push_back(7); // 8
    triangles.push_back(2);
    triangles.push_back(4);
    triangles.push_back(8); // 9
    triangles.push_back(2);
    triangles.push_back(5);
    triangles.push_back(8); // 10
    triangles.push_back(3);
    triangles.push_back(6);
    triangles.push_back(9); // 1
    triangles.push_back(3);
    triangles.push_back(7);
    triangles.push_back(9); // 12
    triangles.push_back(4);
    triangles.push_back(8);
    triangles.push_back(10); // 13
    triangles.push_back(8);
    triangles.push_back(10);
    triangles.push_back(11); // 14
    triangles.push_back(5);
    triangles.push_back(8);
    triangles.push_back(11); // 15
    triangles.push_back(5);
    triangles.push_back(7);
    triangles.push_back(11); // 16
    triangles.push_back(7);
    triangles.push_back(9);
    triangles.push_back(11); // 17
    triangles.push_back(9);
    triangles.push_back(10);
    triangles.push_back(11); // 18
    triangles.push_back(6);
    triangles.push_back(9);
    triangles.push_back(10); // 19
    triangles.push_back(4);
    triangles.push_back(6);
    triangles.push_back(10); // 20
}

void HoughHelper::subDivide()
{
    unsigned int vert_num = vertices.size();
    float norm;
    // subdivide each triangle
    unsigned int num = triangles.size() / 3;
    for (unsigned int i = 0; i < num; i++) {
        pcl::PointXYZ a, b, c, d, e, f;

        unsigned int ai, bi, ci, di, ei, fi;
        ai = triangles.front();
        triangles.pop_front();
        bi = triangles.front();
        triangles.pop_front();
        ci = triangles.front();
        triangles.pop_front();

        a = vertices[ai];
        b = vertices[bi];
        c = vertices[ci];

        //  d = a+b
        d.x = (a.x + b.x);
        d.y = (a.y + b.y);
        d.z = (a.z + b.z);
        norm = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        d.x /= norm;
        d.y /= norm;
        d.z /= norm;
        //  e = b+c
        e.x = (c.x + b.x);
        e.y = (c.y + b.y);
        e.z = (c.z + b.z);
        norm = sqrt(e.x * e.x + e.y * e.y + e.z * e.z);
        e.x /= norm;
        e.y /= norm;
        e.z /= norm;
        //  f = c+a
        f.x = (a.x + c.x);
        f.y = (a.y + c.y);
        f.z = (a.z + c.z);
        norm = sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
        f.x /= norm;
        f.y /= norm;
        f.z /= norm;

        // add all new edge indices of new triangles to the triangles deque
        bool d_found = false;
        bool e_found = false;
        bool f_found = false;
        for (unsigned int j = vert_num; j < vertices.size(); j++) {
            if (vertices[j].getVector3fMap() == d.getVector3fMap()) {
                d_found = true;
                di = j;
                continue;
            }
            if (vertices[j].getVector3fMap() == e.getVector3fMap()) {
                e_found = true;
                ei = j;
                continue;
            }
            if (vertices[j].getVector3fMap() == f.getVector3fMap()) {
                f_found = true;
                fi = j;
                continue;
            }

        }
        if (!d_found) {
            di = vertices.size();
            vertices.push_back(d);
        }
        if (!e_found) {
            ei = vertices.size();
            vertices.push_back(e);
        }
        if (!f_found) {
            fi = vertices.size();
            vertices.push_back(f);
        }

        triangles.push_back(ai);
        triangles.push_back(di);
        triangles.push_back(fi);

        triangles.push_back(di);
        triangles.push_back(bi);
        triangles.push_back(ei);

        triangles.push_back(fi);
        triangles.push_back(ei);
        triangles.push_back(ci);

        triangles.push_back(fi);
        triangles.push_back(di);
        triangles.push_back(ei);
    }
}

// make vectors nondirectional and unique
void HoughHelper::makeUnique()
{
    for (unsigned int i = 0; i < vertices.size(); i++) {
        if (vertices[i].z < 0) { // make hemisphere
            vertices.erase(vertices.begin() + i);

            // delete all triangles with vertex_i
            int t = 0;
            for (auto it = triangles.begin();
                 it != triangles.end();) {
                if (triangles[t] == i || triangles[t + 1] == i ||
                    triangles[t + 2] == i) {
                    it = triangles.erase(it);
                    it = triangles.erase(it);
                    it = triangles.erase(it);
                } else {
                    ++it;
                    ++it;
                    ++it;
                    t += 3;
                }
            }
            // update indices
            for (unsigned int &triangle : triangles) {
                if (triangle > i) {
                    triangle--;
                }
            }

            i--;
        } else if (vertices[i].z == 0) { // make equator vectors unique
            if (vertices[i].x < 0) {
                vertices.erase(vertices.begin() + i);
                // delete all triangles with vertex_i
                int t = 0;
                for (auto it = triangles.begin();
                     it != triangles.end();) {
                    if (triangles[t] == i || triangles[t + 1] == i ||
                        triangles[t + 2] == i) {
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                    } else {
                        ++it;
                        ++it;
                        ++it;
                        t += 3;
                    }
                }
                // update indices
                for (unsigned int &triangle : triangles) {
                    if (triangle > i) {
                        triangle--;
                    }
                }
                i--;
            } else if (vertices[i].x == 0 && vertices[i].y == -1) {
                vertices.erase(vertices.begin() + i);
                // delete all triangles with vertex_i
                int t = 0;
                for (auto it = triangles.begin();
                     it != triangles.end();) {
                    if (triangles[t] == i || triangles[t + 1] == i ||
                        triangles[t + 2] == i) {
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                    } else {
                        ++it;
                        ++it;
                        ++it;
                        t += 3;
                    }
                }
                // update indices
                for (unsigned int &triangle : triangles) {
                    if (triangle > i) {
                        triangle--;
                    }
                }
                i--;
            }
        }
    }
}

// add all points from point cloud to voting space
void HoughHelper::add(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{
    for (auto it : *pc) {
        pointVote(it, true);
    }
}

// subtract all points from point cloud to voting space
void HoughHelper::subtract(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{
    for (auto it : *pc) {
        pointVote(it, false);
    }
}

// add or subtract (add==false) one point from voting space
// (corresponds to inner loop of Algorithm 2 in IPOL paper)
void HoughHelper::pointVote(const pcl::PointXYZ &point, bool add)
{

    // loop over directions B
    for (size_t j = 0; j < vertices.size(); j++) {

        pcl::PointXYZ b = vertices[j];
        float beta = 1.0f / (1.0f + b.z);    // denominator in Eq. (2)

        // compute x' according to left hand side of Eq. (2)
        float x_new = ((1.0f - (beta * (b.x * b.x))) * point.x)
                       - ((beta * (b.x * b.y)) * point.y)
                       - (b.x * point.z);

        // compute y' according to right hand side Eq. (2)
        float y_new = ((-beta * (b.x * b.y)) * point.x)
                       + ((1.0f - (beta * (b.y * b.y))) * point.y)
                       - (b.y * point.z);

        size_t x_i = roundToNearest((x_new + max_x) / dx);
        size_t y_i = roundToNearest((y_new + max_x) / dx);

        // compute one-dimensional index from three indices
        // x_i * #planes * #direction_Vec + y_i * #direction_Vec + #loop
        size_t index = (x_i * num_x * num_b) + (y_i * num_b) + j;

        if (index < VotingSpace.size()) {
            if (add) {
                VotingSpace[index]++;
            } else {
                VotingSpace[index]--;
            }
        }
    }
}

// returns the line with most votes (rc = number of votes)
unsigned int HoughHelper::getLine(Eigen::Vector3f *a, Eigen::Vector3f *b)
{
    unsigned int votes = 0;
    unsigned int index = 0;

    for (unsigned int i = 0; i < VotingSpace.size(); i++) {
        if (VotingSpace[i] > votes) {
            votes = VotingSpace[i];
            index = i;
        }
    }

    // retrieve x' coordinate from VotingSpace[num_x * num_x * num_b]
    float x = (int) (index / (num_x * num_b));
    index -= (int) x * num_x * num_b;
    x = x * dx - max_x;

    // retrieve y' coordinate from VotingSpace[num_x * num_x * num_b]
    float y = (int) (index / num_b);
    index -= (int) y * num_b;
    y = y * dx - max_x;

    // retrieve directional vector
    *b = vertices[index].getVector3fMap();

    // compute anchor point according to Eq. (3)
    a->x() = x * (1.0f - ((b->x() * b->x()) / (1.0f + b->z())))
             - y * ((b->x() * b->y()) / (1.0f + b->z()));
    a->y() = x * (-((b->x() * b->y()) / (1.0f + b->z())))
             + y * (1 - ((b->y() * b->y()) / (1.0f + b->z())));
    a->z() = -x * b->x() - y * b->y();

    return votes;
}
} // railroad