/*
 * BSD 3-Clause License
 * Copyright (c) 2018, Máté Cserép & Péter Hudoba
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef RAILROAD_LASCLASS_H
#define RAILROAD_LASCLASS_H

/**
 * Literal for unsigned char.
 *
 * Many LAS fields are stored as unsigned char (1 byte in C++).
 * We define a custom literal ('_uc') so we can handle numbers easier in the code,
 * as C++ has no native literal for unsigned char.
 */
inline constexpr unsigned char operator "" _uc(unsigned long long int value) noexcept
{
    return static_cast<unsigned char>(value);
}

namespace railroad
{

enum class LASClass : unsigned char
{
    // ASPRS defined classifications
    CREATED = 0_uc,
    UNCLASSIFIED = 1_uc,
    GROUND = 2_uc,
    LOW_VEGETATION = 3_uc,
    MEDIUM_VEGETATION = 4_uc,
    HIGH_VEGETATION = 5_uc,
    BUILDING = 6_uc,
    LOW_POINT = 7_uc,
    MODEL_KEY_POINT = 8_uc,
    WATER = 9_uc,
    RAIL = 10_uc,
    ROAD = 11_uc,
    CABLE = 13_uc,
    HIGH_POINT = 18_uc,
    // custom classifications
    POLE = 20_uc,
    CANTILEVER = 21_uc,
    RAIL_TIES = 22_uc
};

} // railroad

#endif //RAILROAD_LASCLASS_H
