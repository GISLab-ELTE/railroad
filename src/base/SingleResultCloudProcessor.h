/*
 * BSD 3-Clause License
 * Copyright (c) 2018-2023, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_SINGLERESULTCLOUDPROCESSOR_H
#define RAILROAD_SINGLERESULTCLOUDPROCESSOR_H

#include "CloudProcessor.h"

namespace railroad
{

class SingleResultCloudProcessor : public CloudProcessor
{
public:
    SingleResultCloudProcessor(const std::string &name) :
        CloudProcessor(name)
    {}

    PointCloudPtr execute();

protected:
    virtual PointCloudPtr process() = 0;
};

} // railroad

#endif //RAILROAD_SINGLERESULTCLOUDPROCESSOR_H
