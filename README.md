# rp2040-pio-spwm
Simple center aligned SPWM generation with PIO and PIO+DMA angle sensing with AS5600. 

I wanted to explore PIO possibilities. I2C Angle reads are done every 35uS continuously by PIO+DMA. 25kHz open loop SPWM also easily possible. Useful for inverters, motor control, etc.
