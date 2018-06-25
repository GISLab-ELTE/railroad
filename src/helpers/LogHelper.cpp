/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <boost/log/utility/setup.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "LogHelper.h"

namespace pt = boost::posix_time;

namespace railroad
{

void logFormatter(const boost::log::record_view &rec, boost::log::formatting_ostream &strm)
{
    auto now = pt::microsec_clock::local_time();
    auto severity = rec[boost::log::trivial::severity];

    if (severity) {
        // Set the color
        switch (severity.get()) {
            case boost::log::trivial::trace:
            case boost::log::trivial::debug:
                strm << "\033[37m";
                break;
            case boost::log::trivial::warning:
                strm << "\033[33m";
                break;
            case boost::log::trivial::error:
            case boost::log::trivial::fatal:
                strm << "\033[31m";
                break;
            default:
                break;
        }
    }

    std::string sLevel = boost::log::trivial::to_string(severity.get());
    std::transform(sLevel.begin(), sLevel.end(), sLevel.begin(), ::toupper);

    strm
        << "["
        << now.time_of_day().hours() << ":"
        << now.time_of_day().minutes() << ":"
        << now.time_of_day().seconds() << "."
        << now.time_of_day().fractional_seconds()
        << "] "
        << "[" << sLevel << "] "
        << rec[boost::log::expressions::smessage];

    // Restore the default color
    if (severity) {
        strm << "\033[0m";
    }
}

void initLogger(boost::log::trivial::severity_level minSeverity)
{
    boost::log::core::get()->set_filter(
        boost::log::trivial::severity >= minSeverity);

    auto fsSink = boost::log::add_console_log(
        std::cout,
        boost::log::keywords::auto_flush = true);

    fsSink->set_formatter(&logFormatter);
}

} // railroad
