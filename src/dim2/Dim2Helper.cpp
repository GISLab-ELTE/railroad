/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <vector>

#include <opencv2/highgui.hpp>

#include "Dim2Helper.h"
#include "../helpers/LogHelper.h"

using namespace cv;
using namespace std;

namespace railroad
{

bool isPointInMat(Mat &mat, Point &pt)
{
    return 0 <= pt.x && 0 <= pt.y && pt.y <= mat.rows && pt.x <= mat.cols;
}

double calcDirection(Point p1, Point p2)
{
    return atan2(p1.y - p2.y, p1.x - p2.x);

}

double calcAngle(Point p, Point l1, Point l2)
{
    double deg1 = atan2(l1.y - p.y, l1.x - p.x);
    double deg2 = atan2(l2.y - p.y, l2.x - p.x);
    return max(deg1, deg2) - min(deg1, deg2);
}


Mat createGridFromMats(const vector<Mat> &objects)
{
    int cellWidth = 0;
    int rowHeight = 0;
    for (auto obj : objects) {
        if (obj.rows > rowHeight) {
            rowHeight = obj.rows;
        }
        if (obj.cols > cellWidth) {
            cellWidth = obj.cols;
        }
    }

    int rowNum = objects.size() > 5 ? 2 : 1;
    auto colNum = (int) ceil((double) objects.size() / (double) rowNum);

    Mat result = cv::Mat(rowNum * rowHeight, cellWidth * colNum, CV_8UC1, Scalar(0));
    int actualCell = 0;
    int actualRow = 0;
    for (auto obj : objects) {
        cv::Rect ROI(actualCell * cellWidth, actualRow * rowHeight, obj.cols, obj.rows);
        obj.copyTo(result(ROI));
        actualCell++;
        if (colNum <= actualCell) {
            actualCell = 0;
            actualRow++;
        }
    }
    return result;
}

void saveImagesAsGridAndAloneAlso(const std::string &filename, const std::vector<cv::Mat> &objects)
{
    Mat result = createGridFromMats(objects);
    imwriteWithInverse(filename, result);
    size_t lastindex = filename.find_last_of('.');
    string rawName = filename.substr(0, lastindex);

    for (unsigned int a = 0; a < objects.size(); a++) {
        imwriteWithInverse(rawName + "_part_" + std::to_string(a) + ".png", objects[a]);
    }
}

void imwriteWithInverse(const std::string &filename, cv::Mat image)
{
    if (image.total() == 0) {
        LOG(warning) << filename << " skipped, would be empty";
        return; // empty image
    }

    imwrite(filename, image);

    Mat toDraw;
    bitwise_not(image, toDraw);
    imwrite("inverse_" + filename, toDraw);
}

} // railroad