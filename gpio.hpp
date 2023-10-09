/********************************************************************************
 * @brief Class for generic GPIO device usage, such as LEDs and buttons.
 ********************************************************************************/
#pragma once

#include <utils.hpp>

namespace yrgo {
namespace driver {

/********************************************************************************
 * @brief Class for generic GPIO usage, such as LEDs and buttons.
 ********************************************************************************/
class GPIO {
  public:

    /********************************************************************************
	 * @brief Struct containing port names for each pin, where I/O port D consists
	 *        of pin 0 - 7, I/O port B consists of pin 8 - 13 and I/O port C consists
	 *        of pin 14 - 19. 
	 ********************************************************************************/
    struct Port {
	    static constexpr uint8_t D0{0};  /* PORTD0 = pin 0  */
		static constexpr uint8_t D1{1};  /* PORTD1 = pin 1  */
		static constexpr uint8_t D2{2};  /* PORTD2 = pin 2  */
		static constexpr uint8_t D3{3};  /* PORTD3 = pin 3  */
		static constexpr uint8_t D4{4};  /* PORTD4 = pin 4  */ 
		static constexpr uint8_t D5{5};  /* PORTD5 = pin 5  */
		static constexpr uint8_t D6{6};  /* PORTD6 = pin 6  */
		static constexpr uint8_t D7{7};  /* PORTD7 = pin 7  */

		static constexpr uint8_t B0{8};  /* PORTB0 = pin 8  */
		static constexpr uint8_t B1{9};  /* PORTB1 = pin 9  */
		static constexpr uint8_t B2{10}; /* PORTB2 = pin 10 */
		static constexpr uint8_t B3{11}; /* PORTB3 = pin 11 */
		static constexpr uint8_t B4{12}; /* PORTB4 = pin 12 */
		static constexpr uint8_t B5{13}; /* PORTB5 = pin 13 */

		static constexpr uint8_t C0{14}; /* PORTC0 = pin 14 */
		static constexpr uint8_t C1{15}; /* PORTC1 = pin 15 */
		static constexpr uint8_t C2{16}; /* PORTC2 = pin 16 */
		static constexpr uint8_t C3{17}; /* PORTC3 = pin 17 */
		static constexpr uint8_t C4{18}; /* PORTC4 = pin 18 */
		static constexpr uint8_t C5{19}; /* PORTC5 = pin 10 */
	};

    /********************************************************************************
	 * @brief Enumeration class for selectin GPIO direction.
	 *
	 * @param kInput
	 *        Input without internal pull-up resistor enabled (tri-state).
	 * @param kInputPullup
	 *        Input with internal pull-up resistor enabled.
	 * @param kOutput
	 *        Output.
	 ********************************************************************************/
    enum class Direction{kInput, kInputPullup, kOutput};

    /********************************************************************************
     * @brief Enumeration class for selecting I/O port.
     *
     * @param kB
     *        I/O port B.
     * @param kC
     *        I/O port C.
     * @param kD
     *        I/O port D.
     ********************************************************************************/
     enum class IoPort {kB = PCIE0, kC = PCIE1, kD = PCIE2};

	/********************************************************************************
	 * @brief Creates uninitialized device.
	 ********************************************************************************/
    GPIO(void) = default;

	/********************************************************************************
	 * @brief Initializes device with specified parameters.
	 *
	 * @param pin
     *        The PIN number of the device, either ATmega328P port numbers or
	 *        Arduino pin numbers or
	 * @param direction
	 *        The direction of the device.
	 ********************************************************************************/
	GPIO(const uint8_t pin, const Direction direction) { Init(pin, direction); }

	/********************************************************************************
	 * @brief Copy constructor deleted.
	 ********************************************************************************/
	GPIO(GPIO&) = delete;

	/********************************************************************************
	 * @brief Assignment operator deleted.
	 ********************************************************************************/
	GPIO& operator=(GPIO&) = delete;

	/********************************************************************************
	 * @brief Move constructor deleted.
	 ********************************************************************************/
	GPIO(GPIO&&) = delete;

	/********************************************************************************
	 * @brief Disable device before deletion.
	 ********************************************************************************/
	~GPIO(void)  { Disable(); }

	/********************************************************************************
	 * @brief Provides the pin number of the device.
	 *
	 * @return
	 *        The pin number of the device.
	 ********************************************************************************/
	uint8_t operator()(void) { return pin_; }

	/********************************************************************************
	 * @brief Indicates if specified pin is reserved by an existing device.
	 *
	 * @return
	 *        True if specified pin is reserved, else false.
	 ********************************************************************************/
	static constexpr bool PinReserved(const uint8_t pin) {
		return pin <= kPinMax ? utils::Read(pin_list_, pin) : false;
	}

    /********************************************************************************
	 * @brief Provides the I/O port the device is connected to.
	 *
	 * @return
	 *        The I/O port the device is connected to as an enumerator of the
     *        enum class IoPort.
	 ********************************************************************************/
     enum IoPort GetIoPort(void) const { return hardware_->io_port; };

	/********************************************************************************
	 * @brief Initializes device with specified parameters.
	 *
	 * @param pin
     *        The PIN number of the device, either ATmega328P port numbers or
	 *        Arduino pin numbers or
	 * @param direction
	 *        The direction of the device.
	 * @return
	 *        True upon successful initialization, else false.
	 ********************************************************************************/
	bool Init(const uint8_t pin, const Direction direction);

	/********************************************************************************
	 * @brief Disables device so that the pin can be used by another process.
	 ********************************************************************************/
	void Disable(void);

	/********************************************************************************
	 * @brief Enables pin change interrupt for device.
	 ********************************************************************************/
	void EnableInterrupt(void);

	/********************************************************************************
	 * @brief Disables pin change interrupt for device.
	 ********************************************************************************/
	void DisableInterrupt(void) { utils::Clear(*(hardware_->pcmsk_reg), pin_); }

    /********************************************************************************
     * @brief Enables pin change interrupts for entire I/O port the device is
     *        connected to.
     ********************************************************************************/
    void EnableInterruptsOnIoPort(void) { EnableInterruptsOnIoPort(GetIoPort()); }

    /********************************************************************************
     * @brief Disables pin change interrupts for entire I/O port the device is
     *        connected to.
     ********************************************************************************/
    void DisableInterruptsOnIoPort(void) { DisableInterruptsOnIoPort(GetIoPort()); }

	/********************************************************************************
	 * @brief Indicates if pin change interrupt is enabled for device.
	 *
	 * @return
	 *        True if pin change interrupt is enabled, else false.
	 ********************************************************************************/
	bool InterruptEnabled(void) const { return utils::Read(*(hardware_->pcmsk_reg), pin_); }

	/********************************************************************************
	 * @brief Toggles pin change interrupt for device.
	 ********************************************************************************/
	void ToggleInterrupt(void);

	/********************************************************************************
	 * @brief Sets high output for device.
	 *        Note: This operation is only permitted for pins set to output.
	 ********************************************************************************/
	void Set(void);

	/********************************************************************************
	 * @brief Sets low output for device.
	 *        
	 * @note This operation is only permitted for pins set to output.
	 ********************************************************************************/
	void Clear(void);

	/********************************************************************************
	 * @brief Toggles output for device.
	 *        
	 * @note This operation is only permitted for pins set to output.
	 ********************************************************************************/
	void Toggle(void);

	/********************************************************************************
	 * @brief Reads input of device.
	 * 
	 * @return
	 *        True if the input signal is high, else false.
	 ********************************************************************************/
	bool Read(void) const { return utils::Read(*(hardware_->pin_reg), pin_); }

	/********************************************************************************
	 * @brief Sets specified output of device.
	 *
	 * @note This operation is only permitted for pins set to output.
	 *
	 * @param val
	 *        The output value to set (interpreted as 0 or 1).
	 ********************************************************************************/
	void Write(const uint8_t val);

	/********************************************************************************
	 * @brief Blinks output of device with specified blink speed in ms.
	 *
	 * @note This operation is only permitted for pins set to output.
	 *
	 * @param blink_speed_ms
	 *        The blink speed measured in milliseconds.
	 ********************************************************************************/
	void Blink(const uint16_t& blink_speed_ms);

	/********************************************************************************
	 * @brief Sets callback routine for device. This callback routine is shared
	 *        betweens all pins on the same port.
	 *
	 * @param callback_routine
	 *        Function pointer to the specified callback routine.
	 ********************************************************************************/
	void SetCallbackRoutine(void (*callback_routine)(void));

    /********************************************************************************
     * @brief Enables pin change interrupts on the specified I/O port.
     *
     * @param io_port
     *        The I/O port to enable pin change interrupts on.
     ********************************************************************************/
    static void EnableInterruptsOnIoPort(const enum IoPort io_port) { 
        utils::GlobalInterruptEnable();
        utils::Set(PCICR, static_cast<uint8_t>(io_port));
    }

    /********************************************************************************
     * @brief Disables pin change interrupts on the specified I/O port.
     *
     * @param io_port
     *        The I/O port to disable pin change interrupts on.
     ********************************************************************************/
    static void DisableInterruptsOnIoPort(const enum IoPort io_port) {  
        utils::Clear(PCICR, static_cast<uint8_t>(io_port));
    }

  private:

    static constexpr uint8_t kNumPins{20};
    static constexpr uint8_t kPinMin{0};
	static constexpr uint8_t kPinMax{kNumPins - 1};

	struct Hardware {
	    volatile uint8_t* const dir_reg;
	    volatile uint8_t* const port_reg;
	    volatile uint8_t* const pin_reg;
	    volatile uint8_t* const pcmsk_reg;
		const uint8_t pcicr_bit;
        const enum IoPort io_port;
	};

	static Hardware pinb_, pinc_, pind_;
	static uint32_t pin_list_;
	Hardware* hardware_{nullptr};
	uint8_t pin_{};

	static bool PinNumberValid(const uint8_t pin) {
	    return pin >= kPinMin && pin <= kPinMax;
	}

	static constexpr bool PinConnectedToPortD(const uint8_t pin) {
	    return pin >= Port::D0 && pin <= Port::D7;
	}

	static constexpr bool PinConnectedToPortB(const uint8_t pin) {
		return pin >= Port::B0 && pin <= Port::B5;
	}

	static constexpr bool PinConnectedToPortC(const uint8_t pin) {
		return pin >= Port::C0 && pin <= Port::C5;
	}

	void SetIoRegPointers(const uint8_t pin);
	void SetDirection(const enum Direction direction);
};

} /* namespace driver */
} /* namespace yrgo */