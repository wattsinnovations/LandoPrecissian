#include <global_include.hpp>

uint64_t millis()
{
	uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
				  now().time_since_epoch()).count();
	return ms;
}

uint64_t micros()
{
	uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
				  now().time_since_epoch()).count();
	return us;
}
