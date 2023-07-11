/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <iostream>

#include "ProcessorPipeBunch.h"

namespace railroad
{

ProcessorPipeBunch *ProcessorPipeBunch::add(const std::string &name, LASClass classification, ProcessorPipe *pipe, const int cloudIndex)
{
    pipes.push_back(Element(name, classification, pipe, cloudIndex));
    return this;
}
ProcessorPipeBunch *ProcessorPipeBunch::add(const std::string &name, LASClass classification, std::vector<ProcessorPipe*> pipeVector, const int cloudIndex)
{
    pipes.push_back(Element(name, classification, pipeVector, cloudIndex));
    return this;
}
void ProcessorPipeBunch::addToRunOnlyList(const std::string &name)
{
    runOnlyNames.push_back(name);
}

std::vector<ProcessorPipeBunch::Element> ProcessorPipeBunch::getPipes() const
{
    if (!runOnlyNames.empty()) {
        std::vector<Element> ret;
        for (auto& element : pipes) {
            if (find(runOnlyNames.begin(), runOnlyNames.end(), element.name) != runOnlyNames.end()) {
                ret.push_back(element);
            }
        }
        return ret;
    } else {
        return pipes;
    }
}

std::vector<std::string> ProcessorPipeBunch::getRunOnlyList() const
{
    return runOnlyNames;
}

void ProcessorPipeBunch::clearRunOnlyList()
{
    runOnlyNames.clear();
}

} // railroad
