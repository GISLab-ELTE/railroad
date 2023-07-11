/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_PROCESSORPIPEBUNCH_H
#define RAILROAD_PROCESSORPIPEBUNCH_H

#include <string>

#include "ProcessorPipe.h"
#include "../helpers/LASClass.h"

namespace railroad
{

class ProcessorPipeBunch
{
public:
    ProcessorPipeBunch *add(const std::string &name, LASClass classification, ProcessorPipe *pipe, const int cloudIndex = -1);

    ProcessorPipeBunch *add(const std::string &name, LASClass classification, std::vector<ProcessorPipe*> pipeVector, const int cloudIndex = -1) ;

    class Element
    {
    public:
        std::string name;
        LASClass classification;
        std::vector<ProcessorPipe*> pipeVector;
        int cloudIndex;
        Element(const std::string &name, LASClass classification, ProcessorPipe *pipe, const int cloudIndex)
            : name(name), classification(classification), pipeVector( std::vector<ProcessorPipe*> () = {pipe}), cloudIndex(cloudIndex)  {};
        Element(const std::string &name, LASClass classification, std::vector<ProcessorPipe*> pipeVector, const int cloudIndex)
            : name(name), classification(classification), pipeVector(pipeVector), cloudIndex(cloudIndex) {};
    };

    std::vector<Element> getPipes() const;
    void addToRunOnlyList(const std::string &name);
    std::vector<std::string> getRunOnlyList() const;
    void clearRunOnlyList();

protected:
    std::vector<std::string> runOnlyNames;
    std::vector<Element> pipes;
};

} // railroad

#endif //RAILROAD_PROCESSORPIPEBUNCH_H
