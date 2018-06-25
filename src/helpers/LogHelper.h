/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_LOGHELPER_H
#define RAILROAD_LOGHELPER_H

#include <boost/log/trivial.hpp>

namespace railroad
{

#define LOG(lvl) BOOST_LOG_TRIVIAL(lvl)

void initLogger(boost::log::trivial::severity_level minSeverity);

} // railroad

#endif //RAILROAD_LOGHELPER_H
