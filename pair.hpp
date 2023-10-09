/********************************************************************************
 * @brief Implementation of pairs containing values of any data type.
 ********************************************************************************/
#pragma once

namespace yrgo {
namespace container {

/********************************************************************************
 * @brief Class for implementation of pairs. Each pair consists of two values
 *         The data type of each value can be selected arbitrary.
 ********************************************************************************/
template <typename T1, typename T2>
struct Pair {
    T1 first{};  /* First value of the pair. */
    T2 second{}; /* Second value of the pair. */

    /********************************************************************************
     * @brief Default constructor, creates empty pair.
     ********************************************************************************/
    Pair(void) = default;

    /********************************************************************************
     * @brief Creates pair containing referenced values.
     *
     * @param first
     *        Reference to the first value of the pair.
     * @param second
     *        Reference to the second value of the pair.
     ********************************************************************************/
    Pair(const T1& first, const T2& second) : first{first}, second{second} {}
};

} /* namespace container */
} /* namespace yrgo */