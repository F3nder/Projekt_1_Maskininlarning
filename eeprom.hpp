/********************************************************************************
 * @brief Contains functions for writing and reading to the EEPROM memory of
 *        the ATMega328P microcontroller. Data is stored as separate bytes.
 ********************************************************************************/
#pragma once

#include <utils.hpp>
#include <type_traits.hpp>

namespace yrgo {
namespace driver {
namespace eeprom {

/********************************************************************************
 * @brief Parameters for the EEPROM memory.
 *
 * @param kAddressWidth
 *        The address width of the EEPROM memory.
 * @param kAddressMin
 *        Minimum permitted EEPROM address.
 * @param kAddressMax
 *        Maximum permitted EEPROM address.
 ********************************************************************************/
static constexpr uint16_t kAddressWidth{1024};
static constexpr uint16_t kAddressMin{0};
static constexpr uint16_t kAddressMax{kAddressWidth - 1};

namespace {
namespace detail {

/********************************************************************************
 * @brief Indicates if specified address is valid for EEPROM write/read, i.e.
 *        the address lies within interval [0, 1023].
 *
 * @param address
 *        The address to check.
 * @return
 *        True if the address is valid, else false.
 ********************************************************************************/
template <typename T = uint8_t>
bool constexpr AddressValid(const uint16_t address) {
	return address <= kAddressWidth - sizeof(T);
}

/********************************************************************************
 * @brief Writes a single byte of data to specified address in EEPROM.
 *
 * @param address
 *        The destination address.
 * @param data
 *        The data to write to the destination address.
 ********************************************************************************/
void WriteByte(const uint16_t address, const uint8_t data) {
    while (utils::Read(EECR, EEPE));
	EEAR = address;
	EEDR = data;
	utils::GlobalInterruptDisable();
	utils::Set(EECR, EEMPE);
	utils::Set(EECR, EEPE);
    utils::GlobalInterruptEnable();
}

/********************************************************************************
 * @brief Reads a single byte of data to specified address in EEPROM.
 *
 * @param address
 *        The address to read from.
 * @return
 *        The data stored at specified address.
 ********************************************************************************/
uint8_t ReadByte(const uint16_t address) {
    while (utils::Read(EECR, EEPE));
	EEAR = address;
	utils::Set(EECR, EERE);
	return EEDR;
}
} /* namespace detail */

/********************************************************************************
 * @brief Writes data to specified address in EEPROM. If more than one byte is
 *        to be written, the other bytes are written to the consecutive addresses 
 *        until all bytes are stored.
 *
 * @param address
 *        The destination address.
 * @param data
 *        The data to write to the destination address.
 * @return
 *        True if the write succeeded, false if an invalid address was specified.
 ********************************************************************************/
template <typename T = uint8_t>
bool Write(const uint16_t address, const T& data) {
    static_assert(type_traits::is_unsigned<T>::value, "EEPROM write only permitted for unsigned data types!");
    if (!detail::AddressValid<T>(address)) return false;
	for (size_t i{}; i < sizeof(T); ++i) {
	    detail::WriteByte(address + i, static_cast<uint8_t>(data >> (8 * i)));
	}
	return true;
}

/********************************************************************************
 * @brief Reads data from specified address in EEPROM. If more than one byte is
 *        to be read, the consecutive addresses are read until all bytes are read.
 *
 * @param address
 *        The destination address.
 * @param data
 *        Reference to variable for storing the data read from specified address.
 * @return
 *        True if the read succeeded, false if an invalid address was specified.
 ********************************************************************************/
template <typename T = uint8_t>
bool Read(const uint16_t address, T& data) {
    static_assert(type_traits::is_unsigned<T>::value, "EEPROM read only permitted for unsigned data types!");
    if (!detail::AddressValid<T>(address)) return false;
	data = {};
	for (size_t i{}; i < sizeof(T); ++i) {
	    data |= static_cast<T>(detail::ReadByte(address + i) << (8 * i));
	}
	return true;
}

} /* namespace */
} /* namespace eeprom */
} /* namespace driver */
} /* namespace yrgo */