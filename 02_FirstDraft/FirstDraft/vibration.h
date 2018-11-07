#pragma once

#include <Arduino.h>

class vibration
{
public:
	vibration();
	~vibration();
	void add(int pin, int heading_offset);
	void setAmplitude(int amplitude);

private:
	int _pin;
	int _amplitude;
	int _heading_offset;
};