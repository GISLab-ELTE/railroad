/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Balázs Tábori
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_FRAGMENTATIONFILTER_H
#define RAILROAD_FRAGMENTATIONFILTER_H

#include <string>

#include "../base/MultiResultCloudProcessor.h"
#include "../dim2/StraightTrackPartFinderProcessor.h"

namespace railroad
{

class FragmentationFilter : public MultiResultCloudProcessor
{
public:
    FragmentationFilter(const std::string &mode, const std::string &name = "FragmentationFilter")
            : MultiResultCloudProcessor(name), mode(mode) {}

    void setFragmentationAngle(int fragmentationAngle)
    {
        _fragmentationAngle = fragmentationAngle;
    }

protected:
    std::string mode;
    int _fragmentationAngle;

    std::vector<PointCloudPtr> process();
};

} // railroad

#endif //RAILROAD_FRAGMENTATIONFILTER_H
