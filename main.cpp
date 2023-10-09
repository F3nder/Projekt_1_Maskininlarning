/********************************************************************************
 * @brief Demonstration of GPIO device drivers in C++.
 ********************************************************************************/
#include <drivers.hpp> 
#include <vector.hpp>
#include <lin_reg.hpp>

using namespace yrgo::driver;
namespace ctr = yrgo::container;

/********************************************************************************
 * @brief Devices used in the embedded system.
 *
 * @param button1
 *        Button used to toggle the LED between blinking and being disabled.
 * @param timer0
 *        Timer used to reduced the effect of contact bounces when pressing
 *        the button.
 * @param timer1
 *        Timer used to toggle the LED every 100 ms when enabled.
 ********************************************************************************/
static GPIO button1{13, GPIO::Direction::kInputPullup};
static Timer timer0{Timer::Circuit::k0, 300};
static Timer timer1{Timer::Circuit::k1, 60000};
static yrgo::LinReg model{};

namespace {
	
void PredictTemperature(yrgo::LinReg& model, const uint8_t temp_pin) {
	const auto input{adc::GetTemperature(temp_pin)}; /* 0 - 5 V, the x-value :*/
	const auto temp{model.Predict(input)}; /* Predicted temperature. */
	serial::Print("Temperature: ");
	serial::PrintFloat<double>(temp, "\n");
}

/********************************************************************************
 * @brief Callback routine called when button1 is pressed or released.
 *        Every time button1 is pressed, timer1 is toggled, which indirectly
 *        predicts the temp (since timer1 is responsible for predicting the temp).
 *        Pin change interrupts are disabled for 300 ms on the button's I/O port
 *        to reduce the effects of contact bounces.
 ********************************************************************************/
void ButtonCallback(void) {
    button1.DisableInterruptsOnIoPort();
    timer0.Start();
	if (button1.Read()) {
		PredictTemperature(model, 2);
		timer1.Restart();
	}
}

/********************************************************************************
 * @brief Enabled pin change interrupts on the button's I/O port 300 ms after
 *        press or release to reduce the effects of contact bounces.
 ********************************************************************************/
void Timer0Callback(void) {
    if (timer0.Elapsed()) {
        timer0.Stop();
	    button1.EnableInterruptsOnIoPort();
	}
}

/********************************************************************************
 * @brief Predicts the temp when timer1 elapsed, which is every 60 s when enabled.
 ********************************************************************************/
void Timer1Callback(void) {
    if (timer1.Elapsed()) {
		PredictTemperature(model, 2);
    }
}

/********************************************************************************
 * @brief Sets callback routines, enabled pin change interrupt on button1 and
 *        enables the watchdog timer in system reset mode. Loads the trainging 
 *		  data and trains the model.
 ********************************************************************************/
inline void Setup(void) {
	ctr::Vector<double> inputs{{ 0.0 ,1.0 ,2.0 ,3.0 ,4.0 ,5.0 }};
	ctr::Vector<double> outputs{{ -50.0 ,50.0 ,150.0 ,250.0 ,350.0 }};
	model.LoadTrainingData(inputs, outputs);
	model.Train(1000);

	button1.SetCallbackRoutine(ButtonCallback);
	timer0.SetCallback(Timer0Callback);
	timer1.SetCallback(Timer1Callback);
	timer1.Start();

	button1.EnableInterrupt();
	watchdog::Init(watchdog::Timeout::k1024ms);
	watchdog::EnableSystemReset();
	
	serial::Init();
}

} /* namespace */

/********************************************************************************
 * @brief Perform a setup of the system, then running the program as long as
 *        voltage is supplied. The hardware is interrupt controlled, hence the
 *        while loop is almost empty. If the program gets stuck anywhere, the
 *        watchdog timer won't be reset in time and the program will then restart.
 ********************************************************************************/
int main(void)
{
    Setup();
	
    while (1) 
    {
	    watchdog::Reset();
    }
	return 0;
}