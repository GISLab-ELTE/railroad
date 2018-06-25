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

namespace railroad
{

class ProcessorPipeBunch
{
public:
    ProcessorPipeBunch *add(const std::string &name, ProcessorPipe *pipe);

    class Element
    {
    public:
        std::string name;
        ProcessorPipe *pipe;

        Element(const std::string &name, ProcessorPipe *pipe) : name(name), pipe(pipe) {};
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
