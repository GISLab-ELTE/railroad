/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_PROCESSORPIPE_H
#define RAILROAD_PROCESSORPIPE_H

#include <vector>
#include <map>

#include "../base/SingleResultCloudProcessor.h"

namespace railroad
{

class ProcessorPipe : public SingleResultCloudProcessor
{
public:
    ProcessorPipe();

    ProcessorPipe *add(SingleResultCloudProcessor *step);

    PointCloudPtr process();

    virtual ~ProcessorPipe();

    std::map<std::string, double> getTimeResults() const;
    std::map<std::string, int> getFilteredPointsResults() const;
protected:
    std::vector<SingleResultCloudProcessor *> steps;
};

} // railroad

#endif //RAILROAD_PROCESSORPIPE_H
