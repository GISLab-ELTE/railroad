/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_DIM2HELPER_H
#define RAILROAD_DIM2HELPER_H

#include <opencv2/core/core.hpp>

namespace railroad
{

bool isPointInMat(cv::Mat &mat, cv::Point &pt);
double calcDirection(cv::Point p1, cv::Point p2);
double calcAngle(cv::Point p, cv::Point l1, cv::Point l2);
cv::Mat createGridFromMats(const std::vector<cv::Mat> &objects);
void imwriteWithInverse(const std::string &filename, cv::Mat image);
void saveImagesAsGridAndAloneAlso(const std::string &filename, const std::vector<cv::Mat> &objects);

} // railroad

#endif //RAILROAD_DIM2HELPER_H
