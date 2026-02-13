
#include "as5600_fast.h"

static volatile uint32_t latest_word;
static volatile uint32_t latest_timestamp;
int one = 1;
static int sm;
static PIO pio;
static int sda_pin;
static int scl_pin;

void _DMA_Init() {
    // This function sets up DMA to continuously stream from the PIO's RX fifo
    // into the latest_word uint32_t location

    int data_chan = dma_claim_unused_channel(true);
    int timestamp_chan = dma_claim_unused_channel(true);
    int ctrl_chan = dma_claim_unused_channel(true);

    dma_channel_config dma_c = dma_channel_get_default_config(data_chan);

    channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_c, false);
    channel_config_set_write_increment(&dma_c, false);              // <-- key
    channel_config_set_dreq(&dma_c, pio_get_dreq(pio1, sm, false));  // RX pacing
    channel_config_set_chain_to(&dma_c, timestamp_chan);

    dma_channel_configure(data_chan, &dma_c,
                          (void*)&latest_word,
                          (void*)&pio1->rxf[0],
                          1,  
                          false);

    dma_channel_config tc = dma_channel_get_default_config(timestamp_chan);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_32);
    channel_config_set_read_increment(&tc, false);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_chain_to(&tc, ctrl_chan);

    // This alias register: writing loads TRANS_COUNT and starts the channel
    const volatile uint32_t* timerawl_reg= &timer_hw->timerawl;

    dma_channel_configure(
        timestamp_chan, &tc,
        (void*)&latest_timestamp,
        (void*)timerawl_reg, // write to data channel's count+start register
        1,
        false
    );
    
    dma_channel_config cc = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_chain_to(&cc, data_chan);


    // This alias register: writing loads TRANS_COUNT and starts the channel
    volatile uint32_t *data_count_trig = &dma_hw->ch[data_chan].al1_transfer_count_trig;

    dma_channel_configure(
        ctrl_chan, &cc,
        (void*)data_count_trig, // write to data channel's count+start register
        (void*)&one,            // value = 1
        1,
        false
    );

     dma_start_channel_mask(1u << data_chan);
}

void _PIO_Init() {
    /* Configure PIO program */

    uint offset = pio_add_program(pio, &as5600_program);
    pio_sm_config c = as5600_program_get_default_config(offset);

    // Allow PIO to control GPIO pin 
    pio_gpio_init(pio, sda_pin);
    pio_gpio_init(pio, scl_pin);

    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Requires system freq of 200MHz. Results in 1MHz Fast Plus I2C speeds
    sm_config_set_clkdiv(&c, 52);

    // Register Pins for PIO
    sm_config_set_sideset_pins(&c, scl_pin);
    sm_config_set_out_pins(&c, sda_pin, 1);
    sm_config_set_set_pins(&c, sda_pin, 1);
    sm_config_set_in_pins(&c, sda_pin);
    sm_config_set_sideset_pins(&c, scl_pin);
    sm_config_set_in_shift(&c, false, true, 16);

    uint32_t both_pins = (1u << sda_pin) | (1u << scl_pin);
    pio_sm_set_pins_with_mask(pio, sm, both_pins, both_pins);
    pio_sm_set_pindirs_with_mask(pio, sm, both_pins, both_pins);

    // Invert Output Enable to simulate Open Drain
    // Output Enable determines whether the pin is input or output 
    // This makes it so that if the pindir is 1 (normally output), it will 
    // actually be reversed and set to input. By setting it to input, 
    // the line gets "released" and pulled up by the hardware I2C pullups.
    // If Pindir is 0, OE Invert will make the pin an Output pin. The Output
    // value will be set to zero, so it will pull the line down. 
    // This basically just makes it nicer to to set pin direction 0,1 -> low,high
    gpio_set_oeover(scl_pin, GPIO_OVERRIDE_INVERT);
    gpio_set_oeover(sda_pin, GPIO_OVERRIDE_INVERT);
    // Set to Zero
    pio_sm_set_pins_with_mask(pio, sm, 0, both_pins);

    // Start PIO Machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    pio->txf[0] = 0xffffff36; 
}

void AS5600_Start_ContinuousSampling(TwoWire* Wire, int _sda_pin, int _scl_pin, PIO _pio, int pio_sm) {
    sm = pio_sm;
    pio = _pio;
    scl_pin = _scl_pin;
    sda_pin = _sda_pin;

    /* Configure Sensor over regular I2C */
    AS5600 asl(Wire); 

    Wire->setSCL(scl_pin);
    Wire->setSDA(sda_pin);
    Wire->setClock(1000000);
    Wire->begin();
    asl.begin(4);

    asl.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    int b = asl.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);

    asl.setOutputMode(2);
    asl.setPWMFrequency(3);

    uint16_t angle = asl.readAngle();
    Wire->end();

    _DMA_Init();
    _PIO_Init();

}

uint16_t AS5600_LatestAngle() {
    return latest_word;
}

uint32_t AS5600_LatestTimestamp() {
    return latest_timestamp;
}

void benchmark() {
    int max_dt = 0;
    int max2_dt = 0;
    int max3_dt = 0;

    int last_ts = 0;

    //delay(50);
    int time_start_ms = millis();
    int warmup_time = 500;
    uint32_t over_50 = 0;
    uint32_t over_80 = 0;
    int i = 0;
    int max_i = 0;

    while (millis() - time_start_ms < 8000) {
        int angle = AS5600_LatestAngle();
        int ts = AS5600_LatestTimestamp();

        if (millis() - time_start_ms > warmup_time) {
            int dt = ts - last_ts;
            if (dt > max_dt)      { max_i = i; max3_dt = max2_dt; max2_dt = max_dt; max_dt = dt; }
            else if (dt > max2_dt){ max3_dt = max2_dt; max2_dt = dt; }
            else if (dt > max3_dt){ max3_dt = dt; }

            if (dt > 50) over_50++;
            if (dt > 80){
                over_80++;
                pio_sm_set_enabled(pio1, 0, false);
                break;
            } 
            i++;
        }

        last_ts = ts;
    };


    while (1) {
        Serial.printf("Max dts: %d, %d, %d, over 50: %d, over 80: %d; maxi: %d\n", max_dt, max2_dt, max3_dt, over_50, over_80, max_i);
        delay(1000);
    }
}