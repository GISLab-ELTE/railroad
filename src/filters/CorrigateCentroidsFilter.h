/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Attila Láber
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_CORRIGATECENTROIDSFILTER_H
#define RAILROAD_CORRIGATECENTROIDSFILTER_H

#include "../base/CloudProcessor.h"

namespace railroad
{

class CorrigateCentroidsFilter : public CloudProcessor
{
public:
    CorrigateCentroidsFilter(double sampleRadius = 5.0, const std::string &name = "CorrigateCentroidsFilter")
        : CloudProcessor(name), sampleRadius(sampleRadius) {}

protected:
    double sampleRadius;
    PointCloudPtr process();       
};

} // railroad

#endif //RAILROAD_CORRIGATECENTROIDSFILTER_H
