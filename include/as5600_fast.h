#ifndef AS5600_FAST
#define AS5600_FAST

#include "as5600.pio.h"
#include <firmware.hpp>
#include <AS5600.h>
#include <spwm.pio.h>
#include <as5600.pio.h>

void AS5600_Start_ContinuousSampling(TwoWire* Wire, int _sda_pin, int _scl_pin, PIO _pio, int pio_sm);
uint16_t AS5600_LatestAngle();
uint32_t AS5600_LatestTimestamp();

#endif