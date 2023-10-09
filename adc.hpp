/********************************************************************************
 * @brief Functions for usage of 10-bit AD converter.
 ********************************************************************************/
#pragma once

#include <utils.hpp>
#include <type_traits.hpp>

namespace yrgo {
namespace driver {
namespace adc {

/********************************************************************************
 * @brief Minimum and maximum input values when using the 10-bit ADC.
 ********************************************************************************/
static constexpr uint16_t kMinVal{0};
static constexpr uint16_t kMaxVal{1023};

/********************************************************************************
 * @brief Struct containing pin names for each analog pin A0 - A5.
 ********************************************************************************/
struct Pin {
	static constexpr uint8_t A0{0}; /* Pin A0 = 0 */
	static constexpr uint8_t A1{1}; /* Pin A1 = 1 */
	static constexpr uint8_t A2{2}; /* Pin A2 = 2 */
	static constexpr uint8_t A3{3}; /* Pin A3 = 3 */
	static constexpr uint8_t A4{4}; /* Pin A4 = 4 */
	static constexpr uint8_t A5{5}; /* Pin A5 = 5 */
};

/********************************************************************************
 * @brief Struct containing port names for each analog pin A0 - A5.
 ********************************************************************************/
struct Port {
	static constexpr uint8_t C0{14}; /* PORTC0 = pin 14 */
	static constexpr uint8_t C1{15}; /* PORTC1 = pin 15 */
	static constexpr uint8_t C2{16}; /* PORTC2 = pin 16 */
	static constexpr uint8_t C3{17}; /* PORTC3 = pin 17 */
	static constexpr uint8_t C4{18}; /* PORTC4 = pin 18 */
	static constexpr uint8_t C5{19}; /* PORTC5 = pin 10 */
};

/********************************************************************************
 * @brief Reads analog input from specified pin and returns the corresponding
 *        10-bit digital value 0 - 1023.
 *
 * @param pin
 *        The analog pin to read (A0 - A5, which corresponds to PORTC0 - PORTC5).
 * @return
 *        The corresponding 10-bit digital value 0 - 1023 or 0 if an invalid pin
 *        was selected.
 ********************************************************************************/
uint16_t Read(const uint8_t pin);

/********************************************************************************
 * @brief Reads the analog input of specified pin and calculates the 
 *        corresponding on- and off-time in milliseconds.
 *
 * @param pin
 *        The analog pin to read (A0 - A5, which corresponds to PORTC0 - PORTC5).
 * @param pwm_period_ms
 *        The PWM period (on-time + off_time) in milliseconds.
 * @param pwm_on_time_ms
 *        Reference to variable storing the calculated on time in milliseconds.
 * @param pwm_off_time_ms
 *        Reference to variable storing the calculated off time in milliseconds.
 * @return
 *        False if an invalid pin was specified, else true.
 ********************************************************************************/
bool GetDutyCycleParameters_ms(const uint8_t pin, 
                               const uint8_t pwm_period_ms, 
							   uint8_t& pwm_on_time_ms, 
							   uint8_t& pwm_off_time_ms);

/********************************************************************************
 * @brief Reads the analog input of specified pin and calculates the 
 *        corresponding on- and off-time in microseconds.
 *
 * @param pin
 *        The analog pin to read (A0 - A5, which corresponds to PORTC0 - PORTC5).
 * @param pwm_period_ms
 *        The PWM period (on-time + off_time) microseconds.
 * @param pwm_on_time_ms
 *        Reference to variable storing the calculated on time in microseconds.
 * @param pwm_off_time_ms
 *        Reference to variable storing the calculated off time in microseconds.
 * @return
 *        False if an invalid pin was specified, else true.
 ********************************************************************************/
bool GetDutyCycleParameters_us(const uint8_t pin,
                               const uint16_t pwm_period_us, 
							   uint16_t& pwm_on_time_us, 
							   uint16_t& pwm_off_time_us);

namespace {

/********************************************************************************
 * @brief Initializes AD converter by doing a test conversion.
 ********************************************************************************/
inline void Init(void) { Read(Pin::A0); }

/********************************************************************************
 * @brief Reads the analog input of specified pin and returns the corresponding
 *        duty cycle as a floating-point number between 0 - 1.
 *
 * @param pin
 *        The analog pin to read (A0 - A5, which corresponds to PORTC0 - PORTC5).
 * @return
 *        The corresponding duty cycle as a floting-pointer number between 0 - 1 
 *        or 0 if an invalid pin was selected.
 ********************************************************************************/
inline double GetDutyCycle(const uint8_t pin) { return Read(pin) / static_cast<double>(kMaxVal); }
	
/************************************************************************/
/* Reads the temprature from the tempsensor and converts it.            */
/************************************************************************/
inline double GetTemperature(const uint8_t pin) {
	return Read(pin) / 1023.0 * 5;
}

} /* namespace */
} /* namespace adc */
} /* namespace driver */
} /* namespace yrgo */