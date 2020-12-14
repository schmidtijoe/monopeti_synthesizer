#pragma once
#include "../test/i2c_t3.h"

class Mux {
public:
    Mux();
    void setAnalogOut(int a_pin);
    int getChValue (int ch_no) const;
    bool hasChanged() const;
    void setAddressPins(int p1, int p2, int p3);
    static void setMultiplexerChannel (int ch_no);
    void read (int ch_no);

private:
    int muxAddrPins[3];
    int chx_values[8]{};
    int analog_out_pin{};
    bool state_change{false};
};
