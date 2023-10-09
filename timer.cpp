#include "timer.hpp"

namespace yrgo {
namespace driver {

struct ControlBits {
	static constexpr uint8_t k0{(1 << CS01)};
	static constexpr uint8_t k1{(1 << CS11) | (1 << WGM12)};
	static constexpr uint8_t k2{(1 << CS21)};
};

struct TimerIndex {
	static constexpr uint8_t k0{0};
	static constexpr uint8_t k1{1};
	static constexpr uint8_t k2{2};
};

struct Callback {  
	typedef void (*CallbackPtr)(void);
	static CallbackPtr callback_routines[Timer::kNumCircuits];
};

struct Counter {
	static volatile uint32_t timer0;
	static volatile uint32_t timer1;
	static volatile uint32_t timer2;
};

Timer::Hardware Timer::timer0_hw_ {
    .counter = &Counter::timer0,
    .mask_reg = &TIMSK0,
    .mask_bit = TOIE0,
    .index = TimerIndex::k0
};

Timer::Hardware Timer::timer1_hw_ {
    .counter = &Counter::timer1,
	.mask_reg = &TIMSK1,
    .mask_bit = OCIE1A,
	.index = TimerIndex::k1
};

Timer::Hardware Timer::timer2_hw_ {
    .counter = &Counter::timer2,
    .mask_reg = &TIMSK2,
    .mask_bit = TOIE2,
	.index = TimerIndex::k2
};

uint8_t Timer::timer_list_{};

volatile uint32_t Counter::timer0{};
volatile uint32_t Counter::timer1{};
volatile uint32_t Counter::timer2{};

Callback::CallbackPtr callback_routines[Timer::kNumCircuits]{};

Timer::Timer(const enum Circuit circuit, const uint16_t elapse_time_ms, const bool start) : circuit_{circuit} {
    InitHardware(hardware_, circuit);
	SetElapseTime_ms(elapse_time_ms);
	if (start) Start();
};

Timer::~Timer(void) {
    DisableHardware(hardware_, circuit_);
	hardware_ = {nullptr};
	circuit_ = {};
	max_count_ = {};
	enabled_ = {};
}

bool Timer::Init(const enum Circuit circuit, const uint16_t elapse_time_ms, const bool start) {
    if (hardware_) return false;	
	circuit_ = circuit;
    if (InitHardware(hardware_, circuit)) {
	    SetElapseTime_ms(elapse_time_ms);
	    if (start) Start();
		return true;
	} else {
	    return false;
	}
}

void Timer::Start(void) { 
    utils::GlobalInterruptEnable();
    if (max_count_) {
	    utils::Set(*(hardware_->mask_reg), hardware_->mask_bit);
	    enabled_ = true;
	}
};

void Timer::Stop(void) { 
    utils::Clear(*(hardware_->mask_reg), hardware_->mask_bit); 
	enabled_ = false; 
};

void Timer::Toggle(void) {
    if (enabled_) {
	    Stop();
	} else {
	    Start();
	}
}

void Timer::Restart(void) {
    *(hardware_->counter) = 0;
    Start();
}

bool Timer::Elapsed(void) {
    if (*(hardware_->counter) < max_count_ || !enabled_) {
	    return false;
	} else {
	    *(hardware_->counter) = 0;
		return true;
	}
}

void Timer::SetElapseTime_ms(const uint16_t elapse_time_ms) {
    if (elapse_time_ms == 0) Stop();
    max_count_ = GetMaxCount(elapse_time_ms);
}

bool Timer::SetCallback(void (*callback_routine)(void)) {
    if (callback_routine) {
	    callback_routines[hardware_->index] = callback_routine;
		return true;
	} else {
	    return false;
	}
}

bool Timer::InitHardware(Hardware* &hardware, const enum Circuit timer_circuit) {
	if (timer_circuit == Timer::Circuit::k0) {
	       if (utils::Read(timer_list_, TimerIndex::k0)) return false;
		   TCCR0B = ControlBits::k0;
		   hardware = &timer0_hw_;
		} else if (timer_circuit == Timer::Circuit::k1) {
		   if (utils::Read(timer_list_, TimerIndex::k1)) return false;
		   TCCR1B = ControlBits::k1;
		   OCR1A = kTimer1MaxCount;
		   hardware = &timer1_hw_;
		} else if (timer_circuit == Timer::Circuit::k2) {
		   if (utils::Read(timer_list_, TimerIndex::k2)) return false;
		   TCCR2B = ControlBits::k2;
		   hardware = &timer2_hw_;
	}
	utils::Set(timer_list_, hardware->index);
	return true;
}

void Timer::DisableHardware(Hardware* &hardware, const enum Circuit timer_circuit) {
	if (timer_circuit == Timer::Circuit::k0) {
		    TCCR0B = 0x00;
			TIMSK0 = 0x00;
		} else if (timer_circuit == Timer::Circuit::k1) {
		    TCCR1B = 0x00;
		    OCR1A = 0x00;
			TIMSK1 = 0x00;
		} else if (timer_circuit == Timer::Circuit::k2) {
		    TCCR2B = 0x00;
			TIMSK2 = 0x00;
	}
	utils::Clear(timer_list_, hardware->index);
	hardware = nullptr;
}

ISR (TIMER0_OVF_vect) {
    Counter::timer0++;
	if (callback_routines[TimerIndex::k0]) {
	    callback_routines[TimerIndex::k0]();
	}
}

ISR (TIMER1_COMPA_vect) {
    Counter::timer1++;
	if (callback_routines[TimerIndex::k1]) {
		callback_routines[TimerIndex::k1]();
	}
}

ISR (TIMER2_OVF_vect) {
    Counter::timer2++;
	if (callback_routines[TimerIndex::k2]) {
		callback_routines[TimerIndex::k2]();
	}
}

} /* namespace driver */
} /* namespace yrgo */