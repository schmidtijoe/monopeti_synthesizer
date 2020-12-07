#pragma once
#include <Adafruit_MCP23017.h>

class Mux {
public:
    Mux();
    void setAnalogOut(int a_pin);
    void setAddrRead(bool toggle_read, int ch_no, Adafruit_MCP23017 &MCP_DIO);
    int getChValue (int ch_no) const;
    bool hasChanged() const;
    void setAddressPins(int p1, int p2, int p3);

private:
    void setMultiplexerChannel (int ch_no, Adafruit_MCP23017 &MCP_DIO);
    void read (int ch_no);

private:
    int muxAddrPins[3];
    int chx_values[8]{};
    int analog_out_pin{};
    bool state_change{false};
};
