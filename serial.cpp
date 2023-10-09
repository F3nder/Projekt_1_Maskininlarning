#include <serial.hpp>

namespace yrgo {
namespace driver {
namespace serial {

static constexpr char kCarriageReturn{'\r'};
static constexpr char kNewLine{'\n'};

namespace {

constexpr uint16_t GetBaudRateValue(const uint32_t baud_rate_kbps) {
	return baud_rate_kbps == 9600 ? 103 : utils::Round<uint16_t>(F_CPU / 16.0 * baud_rate_kbps);
}

void PrintChar(const char c) {
	while (utils::Read(UCSR0A, UDRE0) == 0);
	UDR0 = c;
}

void PrintString(const char* s) {
    for (const char* i{s}; *i; ++i) {
        PrintChar(*i);
        if (*i == kCarriageReturn) {
	         PrintChar(kNewLine);
        }
    }
}

} /* namespace */

void Init(const uint32_t baud_rate_kbps) {
    static bool serial_initialized{false};
	if (serial_initialized) return;
	utils::Set(UCSR0B, TXEN0);
	utils::Set(UCSR0C, UCSZ00, UCSZ01);
	UBRR0 = GetBaudRateValue(baud_rate_kbps);
	UDR0 = '\r';
	serial_initialized = true;
}

void Print(const char* s, const char* end) {
    PrintString(s);
	PrintString(end);
}

} /* namespace serial */
} /* namespace driver */
} /* namespace yrgo */