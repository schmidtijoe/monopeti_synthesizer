#include "mux.h"
#include "Arduino.h"
#include <i2c_t3.h>

Mux::Mux()
    : muxAddrPins{8, 9, 10}, chx_values(), analog_out_pin(), state_change()
{}
// number of multiplexer modules for analog read ins, default values from design

void Mux::setAnalogOut(int a_pin)
{
    // set the output pin of the object
    analog_out_pin = a_pin;
}

int Mux::getChValue(int ch_no) const
{
    return chx_values[ch_no];
}

bool Mux::hasChanged() const
{
    return state_change;
}

void Mux::setAddressPins(int p1, int p2, int p3)
{
    muxAddrPins[0] = p1;
    muxAddrPins[1] = p2;
    muxAddrPins[2] = p3;
}

void Mux::setMultiplexerChannel(int ch_no)
{
    // were on reg B of mux MCP DIO and can set the bit straight away per channel number
    Wire.beginTransmission(0x21);
    Wire.write(0x13); //gpio register B
    Wire.write(ch_no);
    Wire.endTransmission();
}

void Mux::read(int ch_no)
{
    auto read_value = analogRead(analog_out_pin);
    if (read_value != chx_values[ch_no]) {
        chx_values[ch_no] = read_value;
        state_change = true;
    }
    else state_change = false;
}
