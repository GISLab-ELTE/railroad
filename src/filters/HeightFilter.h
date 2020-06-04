/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Friderika Mayer
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_HEIGHTFILTER_H
#define RAILROAD_HEIGHTFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{
class HeightFilter : public CloudProcessor
{
public:
    HeightFilter(const std::string &name = "HeightFilter")
        : CloudProcessor(name, true) {}

protected:
    PointCloudPtr process() override;
};
} // railroad

#endif //RAILROAD_HEIGHTFILTER_H
