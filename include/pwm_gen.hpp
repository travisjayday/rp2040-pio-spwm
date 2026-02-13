#include <firmware.hpp>

class PWMGen {
public:
    virtual ~PWMGen() = default;

    virtual int setupPWMGen(PIO _pio) = 0;
    virtual uint32_t setElectricalFreq(uint32_t fe_hz) = 0;
};

class SPWMGen : public PWMGen {
private:
    void start_spwm_pio();
    PIO pio;
public:

    int setupPWMGen(PIO _pio) override;
    uint32_t setElectricalFreq(uint32_t fe_hz) override;

};

class SVPWMGen : public PWMGen {
private:
    PIO pio;
public:
    int setupPWMGen(PIO _pio) override;
    uint32_t setElectricalFreq(uint32_t fe_hz) override;
};
