/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_CUTFILTER_H
#define RAILROAD_CUTFILTER_H

#include <string>

#include "../base/CloudProcessor.h"
#include "../dim2/ImportantPartFinderProcessor.h"

namespace railroad
{

class CutFilter : public CloudProcessor
{
public:
    CutFilter(ImportantPartFinderProcessor::Mode mode, const std::string &name = "CutFilter")
        : CloudProcessor(name), mode(mode) {}

protected:
    ImportantPartFinderProcessor::Mode mode;
    PointCloudPtr process();
};

} // railroad

#endif // RAILROAD_CUTFILTER_H
