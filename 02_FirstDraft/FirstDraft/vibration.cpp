#include "vibration.h"

vibration::vibration()
{
	_amplitude = 0;
}

vibration::~vibration() {}

void vibration::add(int pin, int heading_offset)
{
	_pin = pin;
	_heading_offset = heading_offset;

	pinMode(_pin, OUTPUT);
}