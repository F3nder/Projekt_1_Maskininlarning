/********************************************************************************
 * @brief Functions for implementation of serial transmission via USART.
 ********************************************************************************/
#pragma once

#include <utils.hpp>
#include <stdio.h>

namespace yrgo {
namespace driver {
namespace serial {

/********************************************************************************
 * @brief Initializes serial transmission with specified baud rate.
 *
 * @param baud_rate_kbps
 *        The baud rate (transmission speed) in kilobits per second 
 *        (default = 9600 kbps).
 ********************************************************************************/
void Init(const uint32_t baud_rate_kbps = 9600);

/********************************************************************************
 * @brief Prints text in serial terminal.
 *
 * @param s
 *        Reference to string containing the text to print.
 * @param end 
 *        Characters to print at the end of the transmission.
 ********************************************************************************/
void Print(const char* s, const char* end = "");

namespace {

/********************************************************************************
 * @brief Prints signed integer in serial terminal.
 *
 * @param number
 *        The signed integer to print.
 * @param end
 *        Characters to print at the end of the transmission.
 ********************************************************************************/
template <typename T>
void PrintInteger(const T number, const char* end = "") {
    static_assert(type_traits::is_integral<T>::value, "Invalid type for print of signed integer!");
	char s[50]{'\0'};
	sprintf(s, "%ld", static_cast<int32_t>(number));
	Print(s, end);
}

/********************************************************************************
 * @brief Prints unsigned integer in serial terminal.
 *
 * @param number
 *        The unsigned integer to print.
 * @param end
 *        Characters to print at the end of the transmission.
 ********************************************************************************/
template <typename T>
void PrintUnsigned(const T number, const char* end = "") {
    static_assert(type_traits::is_integral<T>::value, "Invalid type for print of unsigned integer!");
    char s[50]{'\0'};
    sprintf(s, "%lu", static_cast<uint32_t>(number));
    Print(s, end);
}

/********************************************************************************
 * @brief Prints floating point number in serial terminal.
 *
 * @param number
 *        The floating point number to print.
 * @param end
 *        Characters to print at the end of the transmission.
 ********************************************************************************/
template <typename T>
void PrintFloat(const T number, const char* end = "") {
     static_assert(type_traits::is_floating_point<T>::value, "Invalid type for print of floating point number!");
     const int32_t integral{static_cast<int32_t>(number)};
	 const int32_t decimal{integral >= 0 ? utils::Round((number - integral) * 1000) : utils::Round((integral - number) * 1000)};
	 char s[50]{'\0'};
	 sprintf(s, "%ld.%ld", integral, decimal);
	 Print(s, end);
}

/********************************************************************************
 * @brief Prints a new line character in serial terminal.
 ********************************************************************************/
inline void GenerateNewLine(void) { Print("\n"); }

} /* namespace */
} /* namespace serial */
} /* namespace driver */
} /* namespace yrgo */