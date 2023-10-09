/********************************************************************************
 * @brief Class for implementation of hardware timers. The microcontroller
 *        ATmega328P contains three timer circuits Timer 0 - Timer 2, which can 
 *        be accessed and used via the Timer class.
 ********************************************************************************/
#pragma once

#include <utils.hpp>

namespace yrgo {
namespace driver {

class Timer {
  public:

    /********************************************************************************
	 * @brief The number of existing hardware timers for microcontroller ATmega328p.
	 ********************************************************************************/
    static constexpr uint8_t kNumCircuits{3};

	/********************************************************************************
	 * @brief Enumeration class for selecting which timer circuit to use.
	 ********************************************************************************/
    enum class Circuit { k0, /* Timer 0 */
	                     k1, /* Timer 1 */ 
						 k2  /* Timer 2 */
	};

	/********************************************************************************
	 * @brief Default constructor, creates uninitialized timer.
	 ********************************************************************************/
    Timer(void) = default;

	/********************************************************************************
	 * @brief Creates new timer with specified elapse time if the selected circuit
	 *        isn't already reserved.
	 *
	 * @param circuit
	 *        The selected timer circuit (Timer 0 - Timer 2).
	 * @param elapse_time_ms
	 *        The elapse time of the timer in ms.
	 * @param start
	 *        Indicates if the timer is to be started immediately after
	 *        initialization (default = false, i.e. timer not started). 
	 ********************************************************************************/
	Timer(const enum Circuit circuit, const uint16_t elapse_time_ms, const bool start = false);

	~Timer(void);

	/********************************************************************************
	 * @brief Copy constructor deleted.
	 ********************************************************************************/
	Timer(Timer&) = delete;

	/********************************************************************************
	 * @brief Assignment operator deleted.
	 ********************************************************************************/
	Timer& operator=(Timer&) = delete;

	/********************************************************************************
	 * @brief Move constructor deleted.
	 ********************************************************************************/
	Timer(Timer&&) = delete;

	/********************************************************************************
	 * @brief Provides the elapse time of the timer.
	 *
	 * @return
	 *        The elapse time of the timer measured in milliseconds.
	 ********************************************************************************/
	uint32_t ElapseTime_ms(void) const { return static_cast<uint32_t>(max_count_ * kInterruptPeriod_ms); };

	/********************************************************************************
	 * @brief Provides which timer circuit (Timer 0 - Timer 2) is used for the timer.
	 *
	 * @return
	 *        The timer circuit used as an enumerator of enumeration class Circuit.
	 ********************************************************************************/
	enum Circuit HardwareCircuit(void) const { return circuit_; };

	/********************************************************************************
	 * @brief Indicated if the timer is enabled.
	 *
	 * @return
	 *        True if the timer is enabled, else false.
	 ********************************************************************************/
	bool Enabled(void) const { return enabled_; };

	/********************************************************************************
	 * @brief Indicates if the timer is disabled.
	 *
	 * @return
	 *        True if the timer is disabled, else false.
	 ********************************************************************************/
	bool Disabled(void) const { return !enabled_; }

	/********************************************************************************
	 * @brief Initializes timer with specified elapse time if the selected circuit
	 *        isn't already reserved.
	 *
	 * @param circuit
	 *        The selected timer circuit (Timer 0 - Timer 2).
	 * @param elapse_time_ms
	 *        The elapse time of the timer in ms.
	 * @param start
	 *        Indicates if the timer is to be started immediately after
	 *        initialization (default = false, i.e. timer not started).
	 * return
	 *        True upon successful initialization, else false. 
	 ********************************************************************************/
	bool Init(const enum Circuit circuit, const uint16_t elapse_time_ms, const bool start = false);

	/********************************************************************************
	 * @brief Starts timer.
	 ********************************************************************************/
	void Start(void);

	/********************************************************************************
	 * @brief Stops timer.
	 ********************************************************************************/
	void Stop(void);

	/********************************************************************************
	 * @brief Toggles timer.
	 ********************************************************************************/
	void Toggle(void);

    /********************************************************************************
	 * @brief Restarts timer.
	 ********************************************************************************/
     void Restart(void);

	/********************************************************************************
	 * @brief Indicates if the timer has elapsed. If true, the timer is reset.
	 *
	 * @return
	 *        True if the timer has elapsed, else false.
	 ********************************************************************************/
	bool Elapsed(void);

	/********************************************************************************
	 * @brief Starts new elapse time of timer.
	 *
	 * @param elapse_time_ms
	 *        The new elapse time measured in milliseconds.
	 ********************************************************************************/
	void SetElapseTime_ms(const uint16_t elapse_time_ms);

	/********************************************************************************
	 * @brief Sets callback routine for timer.
	 *
	 * @param callback_routine
	 *        Function pointer to specified callback routine.
	 *
	 * @return
	 *        True if the callback routine was set, false if a nullptr was passed.
	 ********************************************************************************/
	bool SetCallback(void (*callback_routine)(void));

  private:

    struct Hardware {
	    volatile uint32_t* const counter;
	    volatile uint8_t* const mask_reg;
	    const uint8_t mask_bit;
		const uint8_t index;
    };

	static constexpr uint16_t kTimer1MaxCount{256};
	static constexpr double kInterruptPeriod_ms{0.128};

    static Hardware timer0_hw_, timer1_hw_, timer2_hw_;
	static uint8_t timer_list_;

    Hardware* hardware_{nullptr};
    enum Circuit circuit_{};
    uint32_t max_count_{};
    bool enabled_{};

	static bool InitHardware(Hardware* &hardware, const enum Circuit timer_circuit);
	static void DisableHardware(Hardware* &hardware, const enum Circuit timer_circuit);

	static uint32_t GetMaxCount(const uint16_t elapse_time_ms) {
	    return elapse_time_ms > 0 ? static_cast<uint32_t>(elapse_time_ms / kInterruptPeriod_ms + 0.5) : 0;
	}
};

} /* namespace driver */
} /* namespace yrgo */