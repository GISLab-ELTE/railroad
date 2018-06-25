/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_MATPROCESSOR_H
#define RAILROAD_MATPROCESSOR_H

#include <opencv2/core/core.hpp>

namespace railroad
{

class ImageProcessor
{
public:
    cv::Mat getInputImage()
    {
        return _image;
    }

    void setInputImage(const cv::Mat &image)
    {
        _image = image;
    }

    virtual cv::Mat execute() = 0;

protected:
    cv::Mat _image;
};

} // railroad

#endif //RAILROAD_MATPROCESSOR_H
