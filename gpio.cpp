#include <gpio.hpp>

namespace yrgo {
namespace driver {

static constexpr uint8_t kNumIoPorts{2};

struct Callback {
	struct Index {
	    static constexpr uint8_t kPortB{0};
	    static constexpr uint8_t kPortC{1};
	    static constexpr uint8_t kPortD{2};
	};
	typedef void (*CallbackPtr)(void);
	static CallbackPtr callback_routines[kNumIoPorts];
};

static Callback::CallbackPtr callback_routines[kNumIoPorts]{};

GPIO::Hardware GPIO::pinb_ {
    .dir_reg = &DDRB,
    .port_reg = &PORTB,
    .pin_reg = &PINB,
    .pcmsk_reg = &PCMSK0,
    .pcicr_bit = PCIE0,
    .io_port = IoPort::kB
};

GPIO::Hardware GPIO::pinc_ {
	.dir_reg = &DDRC,
	.port_reg = &PORTC,
	.pin_reg = &PINC,
	.pcmsk_reg = &PCMSK1,
	.pcicr_bit = PCIE1,
    .io_port = IoPort::kC
};

GPIO::Hardware GPIO::pind_ {
	.dir_reg = &DDRD,
	.port_reg = &PORTD,
	.pin_reg = &PIND,
	.pcmsk_reg = &PCMSK2,
	.pcicr_bit = PCIE2,
    .io_port = IoPort::kD
};

uint32_t GPIO::pin_list_{};

bool GPIO::Init(const uint8_t pin, const Direction direction) {
    if (PinNumberValid(pin) && !PinReserved(pin)) {
	    SetIoRegPointers(pin);
	    SetDirection(direction);
		utils::Set(pin_list_, pin);
		return true;
	} else {
	    return false;
	}
}

void GPIO::Disable(void)  {
	utils::Clear(*(hardware_->dir_reg), pin_);
	utils::Clear(*(hardware_->port_reg), pin_);
	utils::Clear(pin_list_, pin_);
	DisableInterrupt();
	hardware_ = nullptr;
	pin_ = 0;
}

void GPIO::EnableInterrupt(void) {
	utils::GlobalInterruptEnable();
	utils::Set(PCICR, hardware_->pcicr_bit);
	utils::Set(*(hardware_->pcmsk_reg), pin_);
}

void GPIO::ToggleInterrupt(void) {
	if (InterruptEnabled()) {
		DisableInterrupt();
	} else {
	    EnableInterrupt();
	}
}

void GPIO::Set(void)  {
	if (utils::Read(*(hardware_->dir_reg), pin_)) {
		utils::Set(*(hardware_->port_reg), pin_);
	}
}

void GPIO::Clear(void) {
	if (utils::Read(*(hardware_->dir_reg), pin_)) {
		utils::Clear(*(hardware_->port_reg), pin_);
	}
}

void GPIO::Toggle(void) {
	if (utils::Read(*(hardware_->dir_reg), pin_)) {
		utils::Set(*(hardware_->pin_reg), pin_);
	}
}

void GPIO::Write(const uint8_t val) {
	if (val) {
		Set();
	} else {
		Clear();
	}
}

void GPIO::Blink(const uint16_t& blink_speed_ms) {
	Toggle();
	utils::Delay_ms(blink_speed_ms);
}

void GPIO::SetCallbackRoutine(void (*callback_routine)(void)) {
	if (hardware_->port_reg == &PORTB) {
	    callback_routines[Callback::Index::kPortB] = callback_routine;
	} else if (hardware_->port_reg == &PORTC) {
	    callback_routines[Callback::Index::kPortC] = callback_routine;
	} else if (hardware_->port_reg == &PORTD) {
	    callback_routines[Callback::Index::kPortD] = callback_routine;
	}
}

void GPIO::SetIoRegPointers(const uint8_t pin)  {
	if (PinConnectedToPortD(pin)) {
		hardware_ = &pind_;
		pin_ = pin;
	} else if (PinConnectedToPortB(pin)) {
		hardware_ = &pinb_;
		pin_ = pin - 8;
	} else if (PinConnectedToPortC(pin)) {
		hardware_ = &pind_;
		pin_ = pin - 14;
	}
}

void GPIO::SetDirection(const enum Direction direction)  {
	if (direction == Direction::kInputPullup) {
		utils::Set(*(hardware_->port_reg), pin_);
	} else if (direction == Direction::kOutput) {
		utils::Set(*(hardware_->dir_reg), pin_);
	}
}

ISR (PCINT0_vect) {
    if (callback_routines[Callback::Index::kPortB]) {
	    callback_routines[Callback::Index::kPortB]();
	}
}

ISR (PCINT1_vect) {
    if (callback_routines[Callback::Index::kPortC]) {
	    callback_routines[Callback::Index::kPortC]();
    }
}

ISR (PCINT2_vect) {
    if (callback_routines[Callback::Index::kPortD]) {
	    callback_routines[Callback::Index::kPortD]();
    }
}

} /* namespace driver */
} /* namespace yrgo */