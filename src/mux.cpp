#include "mux.h"
#include "Arduino.h"
#include <Adafruit_MCP23017.h>

Mux::Mux()
    : muxAddrPins{8, 9, 10}, chx_values(), analog_out_pin(), state_change()
{}
// number of multiplexer modules for analog read ins, default values from design

void Mux::setAnalogOut(int a_pin)
{
    // set the output pin of the object
    analog_out_pin = a_pin;
}

void Mux::setAddrRead(bool toggle_read, int ch_no, Adafruit_MCP23017 &MCP_DIO)
{
    if (toggle_read) {
        // read values
        read(ch_no);
    }
    else {
        // set addr pins
        setMultiplexerChannel(ch_no, MCP_DIO);
    }
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

void Mux::setMultiplexerChannel(int ch_no, Adafruit_MCP23017 &MCP_DIO)
{
    // calculate 3 bit number from channel number to set addr pins high or low and address mux channel
    auto bit_calc = ch_no;
    bool pin_HiLo;
    for (auto pin_idx : muxAddrPins) {
        pin_HiLo = static_cast<bool>(bit_calc % 2);
        bit_calc /= 2;
        MCP_DIO.digitalWrite(pin_idx, pin_HiLo);
    }
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
