#include <pwm_gen.hpp>
#include <spwm.pio.h>

#define PIO_PWM_LO_CYCLE_OVERHEAD_PER_PERIOD (10)
#define PIO_PWM_HI_CYCLE_OVERHEAD_PER_PERIOD (6)

#define PHASE_A_SM (0)
#define PHASE_B_SM (1)
#define PHASE_C_SM (2)


static uint32_t fs_hz = 25000;   // update rate (PWM update rate)

// ---- Config ----
#define LUT_BITS   10
#define LUT_SIZE   (1u << LUT_BITS)   // 1024
#define SIN_SCALE  32767              // int16 amplitude

// 120deg and 240deg offsets in 32-bit phase turns
#define PHASE_120  0x55555555u        // 2^32 / 3
#define PHASE_240  0xAAAAAAABu        // 2*2^32 / 3


void __isr pio0_irq0_handler(void);
void sin_lut_init(void);
void set_electrical_freq(float fe_hz);
static inline int16_t sin_from_phase(uint32_t p);
void start_spwm_pio();

static int16_t sin_lut[LUT_SIZE];

static uint32_t phase = 0;
static uint32_t phase_inc = 0;
static PIO irqPio;

const uint32_t isr_start_n = 100;
uint32_t isr_start[isr_start_n] = {0};
uint32_t isr_end[isr_start_n] = {0};
int isr_i = 0;


static inline int16_t sin_from_phase(uint32_t p)
{
    return sin_lut[p >> (32 - LUT_BITS)];
}

inline void apply_dutycycle(int sm, float duty_cycle) {
    int cycles_per_period = 2000;
    int cycles_per_half = 1000;
    int cycles_halfhigh = (int) (duty_cycle*cycles_per_half); 
    cycles_halfhigh &= ~1;
    int cycles_halflow = cycles_per_half - cycles_halfhigh;

    int16_t halflo = cycles_halflow - PIO_PWM_LO_CYCLE_OVERHEAD_PER_PERIOD/2;
    int16_t halfhi = cycles_halfhigh - PIO_PWM_HI_CYCLE_OVERHEAD_PER_PERIOD/2;

    if (halflo <= 0) {
        halflo = 0;
        halfhi = cycles_per_half  - PIO_PWM_LO_CYCLE_OVERHEAD_PER_PERIOD/2 - PIO_PWM_HI_CYCLE_OVERHEAD_PER_PERIOD/2;
    }
    if (halfhi <= 0) {
        halfhi = 0;
        halflo = cycles_per_half - PIO_PWM_LO_CYCLE_OVERHEAD_PER_PERIOD/2 - PIO_PWM_HI_CYCLE_OVERHEAD_PER_PERIOD/2;
    }

    irqPio->txf[sm] = ((uint32_t)halfhi<<16) | (uint32_t)halflo;
}

void __isr pio0_irq0_handler(void) {
    pio_interrupt_clear(irqPio, 0);
    if (isr_i < isr_start_n)
        isr_start[isr_i] = micros();


    phase += phase_inc;

    float sa = (float)sin_from_phase(phase) / SIN_SCALE;
    float sb = (float)sin_from_phase(phase + PHASE_120) / SIN_SCALE;
    float sc = (float)sin_from_phase(phase + PHASE_240) / SIN_SCALE;

    float m = 1.0f;
    float duty_a = 0.5f + 0.5f * m * sa;
    float duty_b = 0.5f + 0.5f * m * sb;
    float duty_c = 0.5f + 0.5f * m * sc;

    apply_dutycycle(PHASE_A_SM, duty_a);
    apply_dutycycle(PHASE_B_SM, duty_b);
    apply_dutycycle(PHASE_C_SM, duty_c);
    if (isr_i < isr_start_n)
        isr_end[isr_i] = micros();
    isr_i++;
}

void SPWMGen::start_spwm_pio() {
    // Add IRQ handler 
    // irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
    irq_add_shared_handler(PIO0_IRQ_0,
                       pio0_irq0_handler,
                       PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    pio_interrupt_clear(pio, 0);

    uint offset = pio_add_program(pio, &spwm_program);

    // Sets up state machine and wrap target. This function is automatically
    // generated in blink.pio.h.
    pio_sm_config c = spwm_program_get_default_config(offset);
    int pin = PIN_PHASE_A;

    // Allow PIO to control GPIO pin (as output)
    pio_gpio_init(pio, PIN_PHASE_A);
    pio_gpio_init(pio, PIN_PHASE_B);
    pio_gpio_init(pio, PIN_PHASE_C);
    pio_gpio_init(pio, PIN_PHASE_C+1);
    //sm_config_set_set_pins(&c, pin, 1);
    // 200Mhz system clock / 4 (divider) / 2000 cycles per pio program -> 25khz pwm 
    sm_config_set_clkdiv(&c, 4);

    sm_config_set_sideset_pins(&c, PIN_PHASE_A);
    pio_sm_set_consecutive_pindirs(pio, PHASE_A_SM, PIN_PHASE_A, 2, true);
    pio_sm_init(pio, PHASE_A_SM, offset, &c);

    sm_config_set_sideset_pins(&c, PIN_PHASE_B);
    pio_sm_set_consecutive_pindirs(pio, PHASE_B_SM, PIN_PHASE_B, 2, true);
    pio_sm_init(pio, PHASE_B_SM, offset, &c);

    sm_config_set_sideset_pins(&c, PIN_PHASE_C);
    pio_sm_set_consecutive_pindirs(pio, PHASE_C_SM, PIN_PHASE_C, 2, true);
    pio_sm_init(pio, PHASE_C_SM, offset, &c);


    uint32_t mask = (1u << PHASE_A_SM) | (1u << PHASE_B_SM) | (1u << PHASE_C_SM);

    pio_enable_sm_mask_in_sync(pio0, mask);
}

void sin_lut_init(void)
{
    for (int i = 0; i < LUT_SIZE; i++)
    {
        float a = 2.0f * M_PI * i / LUT_SIZE;
        sin_lut[i] = (int16_t)(sinf(a) * SIN_SCALE);
    }
}

int SPWMGen::setupPWMGen(PIO _pio) {
    pio = _pio;
    irqPio = _pio;
    sin_lut_init();
    start_spwm_pio();
}
    
uint32_t SPWMGen::setElectricalFreq(uint32_t fe_hz) {
    phase_inc = (uint32_t)(fe_hz * 4294967296.0 / fs_hz);
}