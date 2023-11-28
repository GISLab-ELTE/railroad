/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_MULTIRESULTCLOUDPROCESSOR_H
#define RAILROAD_MULTIRESULTCLOUDPROCESSOR_H

#include <vector>

#include "CloudProcessor.h"

namespace railroad
{

class MultiResultCloudProcessor : public CloudProcessor
{
public:
    MultiResultCloudProcessor(const std::string &name) :
        CloudProcessor(name)
    {}

    std::vector<PointCloudPtr> execute();

protected:
    virtual std::vector<PointCloudPtr> process() = 0;
};

} // railroad

#endif //RAILROAD_MULTISULTCLOUDPROCESSOR_H
