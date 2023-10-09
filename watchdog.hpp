/********************************************************************************
 * @brief Contains functions for using the ATmega328P watchdog timer.
 ********************************************************************************/
#pragma once

#include <utils.hpp>

namespace yrgo {
namespace driver {
namespace watchdog {

/********************************************************************************
 * @brief Enum class for selecting the timeout of the watchdog timer.
 ********************************************************************************/
enum class Timeout {
    k16ms = 0,                                         /* 16 ms */
	k32ms = (1 << WDP0),                               /* 32 ms */
	k64ms = (1 << WDP1),                               /* 64 ms */
	k128ms = (1 << WDP1) | (1 << WDP0),                /* 128 ms */
	k256ms = (1 << WDP2),                              /* 256 ms */
	k512ms = (1 << WDP2) | (1 << WDP0),                /* 512 ms */
	k1024ms = (1 << WDP2) | (1 << WDP1),               /* 1024 ms */
	k2048ms = (1 << WDP2) | (1 << WDP1) | (1 << WDP0), /* 2048 ms */
	k4096ms = (1 << WDP3),                             /* 4096 ms */
	k8192ms = (1 << WDP3) | (1 << WDP0)                /* 8192 ms */
};

/********************************************************************************
 * @brief Initializes the watchdog timer and sets the selected timeout.
 *
 * @param timeout
 *        The timeout of the watchdog timer passed as an enumerator of the
 *        Timeout enum class.
 ********************************************************************************/
void Init(const enum Timeout timeout);

/********************************************************************************
 * @brief Resets the watchdog timer. This function must be called before the
 *        watchdog timer elapses, else the system will be reset (or a watchdog
 *        interrupt will be generated, depending on the selected mode of the
 *        watchdog timer).
 ********************************************************************************/
void Reset(void);

/********************************************************************************
 * @brief Enables system reset so that the system will be reset if the watchdog
 *        timer elapses, which occurs if the watchdog timer isn't reset in time.
 ********************************************************************************/
void EnableSystemReset(void);

/********************************************************************************
 * @brief Disables system reset so that the system won't be reset even if the
 *        watchdog timer elapses.
 ********************************************************************************/
void DisableSystemReset(void);

/********************************************************************************
 * @brief Enables watchdog timer interrupt so that an interrupt will occur if
 *        if he watchdog timer elapses, which occurs if isn't reset in time.
 *
 * @param callback_routine
 *        Function pointer to the callback routine to call when the watchdog 
 *        timer elapses (default = nullptr).
 ********************************************************************************/
void EnableInterrupt(void (*callback_routine)(void) = nullptr);

/********************************************************************************
 * @brief Disables watchdog timer interrupt so that no interrupt will occur
 *        if the watchdog timer elapses.
 ********************************************************************************/
void DisableInterrupt(void);

} /* namespace watchdog */
} /* namespace driver */
} /* namespace yrgo */