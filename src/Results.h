/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_RESULTS_H
#define RAILROAD_RESULTS_H

#include <set>
#include <map>
#include <string>
#include <iostream>

namespace railroad
{

template<typename T>
class Results;

template<typename T>
std::ostream &operator<<(std::ostream &out, const Results<T> &results);

template<typename T>
class Results
{
public:
    void add(const std::string &name, const std::map<std::string, T> &result)
    {
        for (auto const &x : result) {
            names.emplace(x.first);
        }
        data[name] = result;
    }

    friend std::ostream &operator<<<T>(std::ostream &out, const Results &results);

protected:
    std::set<std::string> names;
    std::map<std::string, std::map<std::string, T> > data;
};

template<typename T>
std::ostream &operator<<(std::ostream &out, const Results<T> &results)
{
    out << "\t";
    for (auto const &name : results.names) {
        out << name << "\t";
    }
    out << std::endl;
    for (auto row : results.data) {
        out << row.first << "\t";
        for (std::string name : results.names) {
            if (row.second.count(name) > 0) {
                out << row.second[name];
            }
            out << "\t";
        }
        out << std::endl;
    }
    return out;
}

} // railroad

#endif //RAILROAD_RESULTS_H
