#include <watchdog.hpp>

namespace yrgo {
namespace driver {
namespace watchdog {

namespace {

inline void ResetWatchdogInHardware(void) {
    asm("WDR");
}

inline void ClearWatchdogResetFlag(void) {
   utils::Clear(MCUSR, WDRF);
}

void (*callback)(void){nullptr};

} /* namespace */

void Init(const enum Timeout timeout_ms) {
    utils::GlobalInterruptDisable();
	utils::Set(WDTCSR, WDCE, WDE);
	WDTCSR = static_cast<uint8_t>(timeout_ms);
	utils::GlobalInterruptEnable();
}

void Reset(void) {
    utils::GlobalInterruptDisable();
	ResetWatchdogInHardware();
	ClearWatchdogResetFlag();
	utils::GlobalInterruptEnable();
}

void EnableSystemReset(void) {
    Reset();
	utils::GlobalInterruptDisable();
	utils::Set(WDTCSR, WDCE, WDE);
	utils::Set(WDTCSR, WDE);
	utils::GlobalInterruptEnable();
}

void DisableSystemReset(void) {
    Reset();
    utils::GlobalInterruptDisable();
    utils::Set(WDTCSR, WDCE, WDE);
    utils::Clear(WDTCSR, WDE);
    utils::GlobalInterruptEnable();
}

void EnableInterrupt(void (*callback_routine)(void)) {
   Reset();
   if (callback_routine) {
       callback = callback_routine;
   }
   utils::GlobalInterruptDisable();
   utils::Set(WDTCSR, WDCE, WDE);
   utils::Set(WDTCSR, WDIE);
   utils::GlobalInterruptEnable();
}

void DisableInterrupt(void) {
   Reset();
   utils::GlobalInterruptDisable();
   utils::Set(WDTCSR, WDCE, WDE);
   utils::Clear(WDTCSR, WDIE);
   utils::GlobalInterruptEnable();
}

ISR (WDT_vect) {
    EnableInterrupt();
    if (callback) {
        callback();
    }
}

} /* namespace watchdog */
} /* namespace driver */
} /* namespace yrgo */