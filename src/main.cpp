#include <firmware.hpp>
#include <AS5600.h>
#include <as5600.pio.h>

#include "as5600_fast.h"
#include "pwm_gen.hpp"

/* Motor Parameters */
int motor_poles = 7;

// Open Loop SPWM with PIO
SPWMGen pwm = SPWMGen();

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_MOTOR_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_EN, 1);

    set_sys_clock_khz(200000, false);

    // offload I2C angle updates through PIO+DMA
    AS5600_Start_ContinuousSampling(&Wire1, 6, 7, pio1, 0);

    pwm.setupPWMGen(pio0);
}


void set_speed(int target_rpm) {
    // ef = electrical freq, mf = mechanical frequency 
    // ef / 7 = mf 
    // mf * 60 = rpm 
    // ef / 7 * 60 = rpm 
    // ef = 7 * rpm / 60
    pwm.setElectricalFreq(motor_poles*target_rpm / 60.0f);
}

void setup1() {}
void loop1() {
    uint16_t angle = AS5600_LatestAngle();
    Serial.printf(">Angle:%d\n", angle);
    delay(100);
}

void loop() {
    for (int i = 1; i < 10; i++) {
        set_speed(i*100);
        delay(1000);
    }
}