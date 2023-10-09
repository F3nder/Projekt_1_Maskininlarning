#include <adc.hpp>

namespace yrgo {
namespace driver {
namespace adc {

static constexpr uint8_t kAdcPortOffset{14};

static constexpr bool PinNumberValid(const uint8_t pin) {
	return (pin >= Pin::A0 && pin <= Pin::A5) || (pin >= Port::C0 && pin <= Port::C5);
}

static constexpr uint8_t PinAdjustedForOffset(const uint8_t pin) {
    return pin <= Pin::A5 ? pin : pin - kAdcPortOffset;
}

uint16_t Read(const uint8_t pin) {
   if (!PinNumberValid(pin)) return 0;
   ADMUX = (1 << REFS0) | PinAdjustedForOffset(pin);
   utils::Set(ADCSRA, ADEN, ADSC, ADPS0, ADPS1, ADPS2);
   while (!utils::Read(ADCSRA, ADIF));
   utils::Set(ADCSRA, ADIF);
   return ADC;
}

bool GetDutyCycleParameters_ms(const uint8_t pin, 
                               const uint8_t pwm_period_ms, 
							   uint8_t& pwm_on_time_ms, 
							   uint8_t& pwm_off_time_ms) {
    if (!PinNumberValid(pin)) return false;
	pwm_on_time_ms = utils::Round<uint8_t>(pwm_period_ms * GetDutyCycle(pin));
	pwm_off_time_ms = pwm_period_ms - pwm_on_time_ms;
	return true;
}

bool GetDutyCycleParameters_us(const uint8_t pin, 
                               const uint16_t pwm_period_us, 
							   uint16_t& pwm_on_time_us,
							   uint16_t& pwm_off_time_us) {
    if (!PinNumberValid(pin)) return false;
	pwm_on_time_us = utils::Round<uint16_t>(pwm_period_us * GetDutyCycle(pin));
	pwm_off_time_us = pwm_period_us - pwm_on_time_us;
	return true;
}


} /* namespace adc */
} /* namespace driver */
} /* namespace yrgo */