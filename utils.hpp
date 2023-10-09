/********************************************************************************
 * @brief Contains miscellaneous utility functions for bit operations, 
 *        mathematical operations and generating delays.
 ********************************************************************************/
#pragma once

#define F_CPU 16000000UL

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <type_traits.hpp>

namespace yrgo {
namespace driver {
namespace utils {
namespace {

/********************************************************************************
 * @brief Sets specified bit of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The specified bit to set.
 ********************************************************************************/
template <typename T = uint8_t>
constexpr void Set(volatile T& reg, const uint8_t bit)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid data type used for bit manipulation!");
	reg |= (1ULL << bit);
}

/********************************************************************************
 * @brief Sets specified bits of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The first bit to set.
 * @param bits
 *        The others bits to set.
 ********************************************************************************/
template <typename T = uint8_t, typename... Bits>
constexpr void Set(volatile T& reg, const uint8_t bit, Bits... bits...)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid data type used for bit manipulation!");
    Set(reg, bit);
	Set(reg, bits...);
}

/********************************************************************************
 * @brief Clears specified bit of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The specified bit to clear.
 ********************************************************************************/
template <typename T = uint8_t>
constexpr void Clear(volatile T& reg, const uint8_t bit)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid data type used for bit manipulation!");
    reg &= ~(1ULL << bit);
}

/********************************************************************************
 * @brief Clears specified bits of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The first bit to clear.
 * @param bits
 *        The others bits to clear.
 ********************************************************************************/
template <typename T = uint8_t, typename... Bits>
constexpr void Clear(volatile T& reg, const uint8_t bit, Bits... bits...)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid datatype used for bit manipulation!");
    Clear(reg, bit);
	Clear(reg, bits...);
}

/********************************************************************************
 * @brief Toggles specified bit of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The specified bit to toggle.
 ********************************************************************************/
template <typename T = uint8_t>
constexpr void Toggle(volatile T& reg, const uint8_t bit)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid datatype used for bit manipulation!");
    reg ^= (1ULL << bit);
}

/********************************************************************************
 * @brief Toggles specified bits of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The first bit to toggle.
 * @param bits
 *        The others bits to toggle.
 ********************************************************************************/
template <typename T = uint8_t, typename... Bits>
constexpr void Toggle(volatile T& reg, const uint8_t bit, Bits... bits...)  {
    Toggle(reg, bit);
	Toggle(reg, bits...);
}

/********************************************************************************
 * @brief Reads specified bit of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The specified bit to read.
 * @return
 *        True if the specified bit is high, else false.
 ********************************************************************************/
template <typename T = uint8_t>
constexpr bool Read(const volatile T& reg, const uint8_t bit)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid data type used for bit manipulation!");
    return reg & (1ULL << bit);
}

/********************************************************************************
 * @brief Reads specified bits of selected register.
 *
 * @param reg
 *        The destination register.
 * @param bit
 *        The first bit to read.
 * @param bits
 *        The other bits to read.
 * @return
 *        True if at least one of the specified bits is high, else false.
 ********************************************************************************/
template <typename T = uint8_t, typename... Bits>
constexpr bool Read(const volatile T& reg, const uint8_t bit, Bits... bits...)  {
    static_assert(type_traits::is_unsigned<T>::value, "Invalid data type used for bit manipulation!");
    return Read(reg, bit) | Read(reg, bits...);
}

/********************************************************************************
 * @brief Blocks the calling thread for the specified time measured in seconds.
 *
 * @param delay_time_s
 *        The time to block the thread in seconds.
 ********************************************************************************/
inline void Delay_s(const uint16_t&& delay_time_s) {
    for (uint16_t i{}; i < delay_time_s; ++i) {
	    _delay_ms(1000);
	}
}

/********************************************************************************
 * @brief Blocks the calling thread for the specified time measured in 
 *        milliseconds.
 *
 * @param delay_time_ms
 *        The time to block the thread in milliseconds.
 ********************************************************************************/
inline void Delay_ms(const uint16_t& delay_time_ms) {
    for (uint16_t i{}; i < delay_time_ms; ++i) {
	    _delay_ms(1);
	}
}

/********************************************************************************
 * @brief Blocks the calling thread for the specified time measured in 
 *        microseconds.
 *
 * @param delay_time_us
 *        The time to block the thread in microseconds.
 ********************************************************************************/
inline void Delay_us(const uint16_t&& delay_time_us) {
	for (uint16_t i{}; i < delay_time_us; ++i) {
	    _delay_us(1);
	}
}

/********************************************************************************
 * @brief Calculates the power of specified base and exponent, where
 *
 *                           power = base ^ exponent
 *
 * @param base 
 *        Specified base.
 * @param exponent
 *        Specified exponent.
 * @return
 *        The corresponding power.
 ********************************************************************************/
template <typename T1 = double, typename T2 = double, typename T3 = double>
inline T3 Power(const T1 base, const T2 exponent) {
	static_assert(type_traits::is_arithmetic<T1>::value &&
	              type_traits::is_arithmetic<T2>::value &&
	              type_traits::is_arithmetic<T3>::value,
	              "Calculation of power only possible for arithmetic types!");
	T1 num{1};
	for (uint16_t i{}; i < exponent; ++i) {
	    num *= base;
    }
    return static_cast<T3>(num);
}

/********************************************************************************
 * @brief Enables interrupts globally.
 ********************************************************************************/
inline void GlobalInterruptEnable(void) { asm("SEI"); }

/********************************************************************************
 * @brief Disables interrupts globally.
 ********************************************************************************/
inline void GlobalInterruptDisable(void) { asm("CLI"); }

/********************************************************************************
 * @brief Rounds the specified number to the nearest integer.
 *
 * @param number
 *        The number to round.
 * @return
 *        The corresponding rounded number.
 ********************************************************************************/
template <typename T1 = int32_t, typename T2 = double>
constexpr T1 Round(const T2 value) {
    static_assert(type_traits::is_integral<T1>::value && type_traits::is_arithmetic<T2>::value, 
	              "Rounding only possible for arithmetic types!");
    return static_cast<T1>(value + 0.5);
}

} /* namespace */
} /* namespace utils */
} /* namespace driver */
} /* namespace yrgo */