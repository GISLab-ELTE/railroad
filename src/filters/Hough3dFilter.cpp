/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 *
 * The implementation of the 3D Hough filter was inspired and built on the following paper and reference code:
 * Christoph Dalitz et al.: Iterative Hough Transform for Line Detection in 3D Point Clouds, Image Processing On Line, 7 (2017), pp. 184–196.
 * https://www.ipol.im/pub/art/2017/208/
 */

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

#include "Hough3dFilter.h"
#include "../helpers/HoughHelper.h"
#include "../helpers/LogHelper.h"

namespace railroad
{

pcl::PointCloud<pcl::PointXYZ>::Ptr Hough3dFilter::process()
{
    stopTimeMeasure();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*_cloud, *cloud);
    startTimeMeasure();

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector3f shift = ((minPt.getVector3fMap() + maxPt.getVector3fMap()) / 2.0);
    transform.translation() << -1.0 * shift;

    pcl::transformPointCloud(*cloud, *transformed, transform);

    pcl::getMinMax3D(*transformed, minPt, maxPt);

    float opt_dx = 1.0f;

    auto *hough = new HoughHelper(minPt, maxPt, opt_dx, 4);

    hough->add(transformed);

    // iterative Hough transform
    // (Algorithm 1 in IPOL paper)
    pcl::PointCloud<pcl::PointXYZ>::Ptr Y(new pcl::PointCloud<pcl::PointXYZ>);    // points close to line
    std::vector<int> indices;
    float rc;
    unsigned int nvotes;
    int nlines = 0;
    int opt_nlines = _houghLineCount;
    int opt_minvotes = 2;
    do {
        Eigen::Vector3f a; // indices anchor point of line
        Eigen::Vector3f b; // direction of line

        hough->subtract(Y); // do it here to save one call

        nvotes = hough->getLine(&a, &b);
        {
            Eigen::Vector3f p = a + shift;
            LOG(trace) << "Highest number of Hough votes is " << nvotes << " for for the following line: "
                       << "a=(" << p.x() << "," << p.y() << "," << p.z() << "), b=(" << b.x() << "," << b.y() << "," << b.z() << ")";
        }

        pointsCloseToLine(a, b, opt_dx, Y, transformed, indices);

        rc = orthogonal_LSQ(Y, &a, &b);
        if (rc == 0.0f) break;

        pointsCloseToLine(a, b, opt_dx, Y, transformed, indices);

        nvotes = Y->size();
        pcl::copyPointCloud(*cloud, indices, *temp);
        result->insert(result->end(), temp->begin(), temp->end());

        if (nvotes < (unsigned int) opt_minvotes) break;

        rc = orthogonal_LSQ(Y, &a, &b);
        if (rc == 0.0f) break;

        a += shift;

        nlines++;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        inliers->indices = indices;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(transformed);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*transformed);

        extract.setInputCloud(cloud);
        extract.filter(*cloud);
    } while ((transformed->size() > 1) &&
             ((opt_nlines == 0) || (opt_nlines > nlines)));

    LOG(debug) << "Hough lines: " << nlines;

    delete hough;

    return result;
}

void Hough3dFilter::pointsCloseToLine(const Eigen::Vector3f &a, const Eigen::Vector3f &b, float dx,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Y,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &indices)
{
    Y->clear();
    indices.clear();
    for (size_t i = 0; i < cloud->size(); i++) {
        // distance computation after IPOL paper Eq. (7)
        float t = (b.dot(cloud->at(i).getVector3fMap() - a));
        Eigen::Vector3f d = (cloud->at(i).getVector3fMap() - (a + (t * b)));

        if (d.norm() <= dx) {
            indices.push_back(i);
            Y->push_back(cloud->at(i));
        }
    }
}

// orthogonal least squares fit with libeigen
// rc = largest eigenvalue
float Hough3dFilter::orthogonal_LSQ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, Eigen::Vector3f *a, Eigen::Vector3f *b)
{
    float rc = 0.0f;

    // anchor point is mean value
    *a = meanValue(pc);

    // copy points to libeigen matrix
    Eigen::MatrixXf points = Eigen::MatrixXf::Constant(pc->size(), 3, 0);

    for (int i = 0; i < points.rows(); i++) {
        points(i, 0) = pc->at(i).x;
        points(i, 1) = pc->at(i).y;
        points(i, 2) = pc->at(i).z;
    }

    // compute scatter matrix ...
    Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
    Eigen::MatrixXf scatter = (centered.adjoint() * centered);

    // ... and its eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(scatter);
    Eigen::MatrixXf eigvecs = eig.eigenvectors();

    // we need eigenvector to largest eigenvalue
    // libeigen yields it as LAST column
    b->x() = eigvecs(0, 2);
    b->y() = eigvecs(1, 2);
    b->z() = eigvecs(2, 2);
    rc = eig.eigenvalues()(2);

    return (rc);
}

Eigen::Vector3f Hough3dFilter::meanValue(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{
    Eigen::Vector3f ret(0, 0, 0);
    for (auto &i : *pc) {
        ret += i.getVector3fMap();
    }
    if (!pc->empty())
        return (ret / (float) pc->size());
    else
        return ret;
}
} // railroad
