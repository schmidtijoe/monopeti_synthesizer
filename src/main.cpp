#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include "wiring.h"
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

/**
 * Global variables
 */
// keybed
Adafruit_MCP23017 MCP_KEYS;     // gpio expander handling keyboard
constexpr u_int8_t NO_OF_KEYS = 64;     // total number of keys, i.e. toggle switches

// analog reading
constexpr u_int8_t READ_RES_BIT_DEPTH = 10;  // bit depth of reading resolution
int RES_RANGE = (int) (2 << READ_RES_BIT_DEPTH) - 1;  // analog reading range, dependent on bit depth
constexpr u_int8_t NO_OF_MODULES = 4;      // number of multiplexer modules for analog read ins
int AIN_PINS_MUX[NO_OF_MODULES] = {35, 34, 33, 31};
bool TOGGLE_MUX_SET_READ = false;
int INDEX_MUX_READ = 0;
elapsedMillis UPDATE_TIMER;
unsigned int UPDATE_INTERVAL = 2;       // [ms] reading interval for analog inputs through multiplexers

// note & midi interfacing
constexpr byte HALF_VEL = 90;
bool TOGGLE_LOOP = false;       // toggles note searching for mono-keyboard use

// digital outputs
Adafruit_MCP23017 MCP_DIO;      // gpio expander, handling multiplexer switches and LED controls

/**
 * Audio System design tool
 */
// GUItool: begin automatically generated code
AudioSynthWaveform       LFO1;      //xy=55,76.00000381469727
AudioAmplifier           Volume;           //xy=122.00000762939453,552.0000076293945
AudioEffectFade          vel_fade_midi;          //xy=175.0000228881836,343.0000057220459
AudioEffectFade          vel_change_low;          //xy=179.00001525878906,274.0000047683716
AudioEffectFade          vel_change_high;          //xy=180.00000762939453,309.00000381469727
AudioEffectEnvelope      env_pitch;      //xy=182,76
AudioEffectDelay         delay_fx;         //xy=266.4444389343262,595.5555419921875
AudioSynthKarplusStrong  pink;        //xy=331,153.00000190734863
AudioSynthWaveformModulated OSC1;   //xy=335.00000381469727,26.000003337860107
AudioSynthWaveformModulated OSC2;   //xy=337.00000762939453,67.00000667572021
AudioSynthWaveformModulated OSC3;   //xy=337,110.00001430511475
AudioSynthWaveform       LFO_ring;      //xy=345.00000762939453,351.0000057220459
AudioSynthWaveformDc     dc_ring;            //xy=345.00000762939453,392.0000057220459
AudioMixer4              velocity_env;         //xy=379.00000762939453,296.00000381469727
AudioMixer4              mix_delay1;         //xy=417.00000381469727,555.7999954223633
AudioMixer4              mix_delay2;         //xy=419.00000381469727,623.7999954223633
AudioMixer4              mix_ring_wet;         //xy=493,381
AudioEffectMultiply      ring;      //xy=548.0000076293945,294.00000381469727
AudioMixer4              mix_osc;         //xy=549.0000076293945,66.00000762939453
AudioSynthWaveform       LFO2;      //xy=608.0000152587891,445.22223234176636
AudioMixer4              mix_delay_wet;         //xy=610.4444274902344,517.466682434082
AudioEffectEnvelope      env_filter;      //xy=668.0000114440918,338.0000057220459
AudioAmplifier           filter_attenuator;           //xy=704.0000114440918,296.00000381469727
AudioOutputI2S           i2s2;           //xy=749.7500133514404,33.416707038879395
AudioEffectFreeverbStereo freeverbs;     //xy=772.5555686950684,516.000020980835
AudioEffectEnvelope      ADSR_vol;      //xy=826.0000114440918,93
AudioFilterStateVariable LPF;        //xy=866.0000133514404,306.0000057220459
AudioConnection          patchCord1(LFO1, env_pitch);
AudioConnection          patchCord2(Volume, delay_fx);
AudioConnection          patchCord3(Volume, 0, mix_delay_wet, 0);
AudioConnection          patchCord4(vel_fade_midi, 0, velocity_env, 2);
AudioConnection          patchCord5(vel_change_low, 0, velocity_env, 0);
AudioConnection          patchCord6(vel_change_high, 0, velocity_env, 1);
AudioConnection          patchCord7(delay_fx, 0, mix_delay1, 0);
AudioConnection          patchCord8(delay_fx, 1, mix_delay1, 1);
AudioConnection          patchCord9(delay_fx, 2, mix_delay1, 2);
AudioConnection          patchCord10(delay_fx, 3, mix_delay1, 3);
AudioConnection          patchCord11(delay_fx, 4, mix_delay2, 0);
AudioConnection          patchCord12(delay_fx, 5, mix_delay2, 1);
AudioConnection          patchCord13(pink, 0, mix_osc, 3);
AudioConnection          patchCord14(OSC1, 0, mix_osc, 0);
AudioConnection          patchCord15(OSC2, 0, mix_osc, 1);
AudioConnection          patchCord16(OSC3, 0, mix_osc, 2);
AudioConnection          patchCord17(LFO_ring, 0, mix_ring_wet, 0);
AudioConnection          patchCord18(dc_ring, 0, mix_ring_wet, 1);
AudioConnection          patchCord19(velocity_env, 0, ring, 0);
AudioConnection          patchCord20(mix_delay1, 0, mix_delay_wet, 1);
AudioConnection          patchCord21(mix_delay2, 0, mix_delay_wet, 2);
AudioConnection          patchCord22(mix_ring_wet, 0, ring, 1);
AudioConnection          patchCord23(ring, filter_attenuator);
AudioConnection          patchCord24(mix_osc, 0, i2s2, 0);
AudioConnection          patchCord25(mix_osc, 0, i2s2, 1);
AudioConnection          patchCord26(LFO2, env_filter);
AudioConnection          patchCord27(mix_delay_wet, freeverbs);
AudioConnection          patchCord28(env_filter, 0, LPF, 1);
AudioConnection          patchCord29(filter_attenuator, 0, LPF, 0);
AudioConnection          patchCord30(ADSR_vol, vel_change_low);
AudioConnection          patchCord31(ADSR_vol, vel_change_high);
AudioConnection          patchCord32(ADSR_vol, vel_fade_midi);
AudioConnection          patchCord33(LPF, 0, Volume, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=885.0000076293945,183.00000762939453
// GUItool: end automatically generated code

/**
 * Class description for mapping keybed to notes, saving:
 * note value
 * velocity value
 * state of button
 * timer last pressed if on
 */
class Note
{
public:
    Note() : push_state(false), note_value(0), velocity_value(0), last_press_timer(0) {   };

    void setNoteValues(byte no_val, byte vel_val)
    {
        note_value = no_val;
        velocity_value = vel_val;
    }

    // We declare this method "const" because it doesn't "change" the note object. It
    // just returns information.
    bool isPressed() const
    {
        return push_state;
    }

    void setPush(bool state)
    {
        /** If we already have the new state, which means either a key was pressed before and
         * is still pressed now, or a key was not pressed before and is not pressed now either,
         * then we simply do nothing.
         */
        if (state == push_state) {
            return;
        }

        // Now, we are sure that the state of the key has changed...
        push_state = state;

        /**
         * ... that means if the key is pressed now, we not only set its state to true, but
         * we also start the timer on it.
         */
        if (push_state) {
            last_press_timer = micros();
            // send midi on event
        }
            /**
             * Otherwise, when the key was released, we reset the timer to 0
             */
        else {
            last_press_timer = 0;
            // send midi off event
        }
    }

    unsigned int getTime() const
    {
        return last_press_timer;
    }

    byte get_note() const
    {
        return note_value;
    }

    byte get_velocity() const
    {
        return velocity_value;
    }

private:
    bool push_state;
    byte note_value;
    byte velocity_value;
    unsigned int last_press_timer;
};

// create array of notes and current note
Note notes[NO_OF_KEYS];
static Note CURRENT_NOTE;

/**
 * multiplexer modules as own objects
 */
class Mux{
public:
    Mux() : chx(), analogOutPin(0) {   };

    void setAnalogIn(int pin_no){
        analogOutPin = pin_no;
    }

    void setAddressPins(int ch_no) {
        bool bit;
        int bit_calc;
        // set mcp pins for mux address control:
        bit_calc = ch_no;
        for (int mux_idx = 8; mux_idx < 11; mux_idx++) {
            // this turns the index number which runs from 0 to 7 into 3 bit binary format to feed the address pins d0-2 of the mux HIGH or LOW
            bit = static_cast<bool>(bit_calc % 2);
            bit_calc = (int) bit_calc / 2;
            MCP_DIO.digitalWrite(mux_idx, bit);
        }
    }

    void readValue(int ch_no) {
        int read_value;
        read_value = abs(analogRead(analogOutPin) - RES_RANGE);
        if (chx[ch_no] != read_value) chx[ch_no] = read_value;
    }

    int getReadValue(int ch_no) const{
        return chx[ch_no];
    }

private:
    int chx[8];
    int analogOutPin;
};

// create array of muxs for mux modules
Mux mux_modules[NO_OF_MODULES];

/**
 * functions
 */
// testing/debugging functions
// test function to display keyboard output
void note_to_serial(const Note &name_note) {
    auto mod_value = name_note.get_note() % 12;
    switch(mod_value) {
        case 0:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tC \t");
            Serial.println(name_note.get_velocity());
            break;
        case 1:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tC# \t");
            Serial.println(name_note.get_velocity());
            break;
        case 2:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tD \t");
            Serial.println(name_note.get_velocity());
            break;
        case 3:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tD# \t");
            Serial.println(name_note.get_velocity());
            break;
        case 4:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tE \t");
            Serial.println(name_note.get_velocity());
            break;
        case 5:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tF \t");
            Serial.println(name_note.get_velocity());
            break;
        case 6:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tF# \t");
            Serial.println(name_note.get_velocity());
            break;
        case 7:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tG \t");
            Serial.println(name_note.get_velocity());
            break;
        case 8:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tG# \t");
            Serial.println(name_note.get_velocity());
            break;
        case 9:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tA \t");
            Serial.println(name_note.get_velocity());
            break;
        case 10:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tA# \t");
            Serial.println(name_note.get_velocity());
            break;
        case 11:
            Serial.println("Note pressed: \tvalue \tvelocity: ");
            Serial.print("\tH \t");
            Serial.println(name_note.get_velocity());
            break;
        default:
            break;
    }
}

// initialize note struct -> give correct note numbers and velocities to elements
void init_key_notes()
{
    /* init key note_matrix
     * identified by in and output values when set and read via the mcp
     * each identifier is mapped to a note and velocity value
     * start time and logic value are initialized with 0
     */

    for (int out_idx = 0; out_idx < 8; out_idx++) {
        for (int in_idx = 0; in_idx < 8; in_idx++) {
            int idx = 8 * out_idx + in_idx;
            switch (out_idx) {
                case 0:
                    notes[idx].setNoteValues(56 + in_idx, HALF_VEL);
                    break;
                case 1:
                    notes[idx].setNoteValues(64 + in_idx, HALF_VEL);
                    break;
                case 2:
                    notes[idx].setNoteValues(48 + in_idx, HALF_VEL);
                    break;
                case 3:
                    notes[idx].setNoteValues(48 + in_idx, 127);
                    break;
                case 4:
                    notes[idx].setNoteValues(56 + in_idx, 127);
                    break;
                case 5:
                    notes[idx].setNoteValues(64 + in_idx, 127);
                    break;
                case 6:
                    notes[idx].setNoteValues(72 + in_idx, 127);
                    break;
                case 7:
                    notes[idx].setNoteValues(72 + in_idx, HALF_VEL);
                    break;
                default:
                    break;
            }
        }
    }
}

// read keybed repetitively in loop, make sure this is fast! just updates the struct with key states and timers
void keybed_read()
{
    for (int out_idx = 8; out_idx < 16; out_idx++) {
        // set mcp pin high
        MCP_KEYS.digitalWrite(out_idx, HIGH);

        // run through read
        for (int in_idx = 0; in_idx < 8; in_idx++) {
            auto total_idx = (out_idx - 8) * 8 + in_idx;
            auto key_state = static_cast<bool>(MCP_KEYS.digitalRead(in_idx));
            notes[total_idx].setPush(key_state);
        }

        // reset pin
        MCP_KEYS.digitalWrite(out_idx, LOW);

    }
}

// identifies note to play
void note_update()
{
    /**
     * need function that sends:
     * - > note on event when no note pressed previously (even if the last note off event is the same note!!!)
     * - > note off event for all notes if no note is pressed anymore
     * - > note pitch and velocity change if note goes off but others are still pressed
     *
     * what happens if note is on and new note is pressed?          -> new on event  condition A -> note.ispressed true & highest getTime()
     * what happens if no note is played and new is pressed?        -> new on event   condition A -> note.ispressed true & highest getTime()
     * what happens if note goes off and other is on?               -> pitch & vel change, no new event (extra condition, extra toggle? one more loop til off?)
     * what happens if note goes off and no other is on             -> new off event
     * what happens with velocity changes?                          -> no play events
     */
    for (const auto &note: notes) {
        if (note.isPressed() && (note.getTime() > CURRENT_NOTE.getTime())) {
            if (note.get_note() == CURRENT_NOTE.get_note()) {
                // same note value
                Serial.println("no new event");
                // only velocity change, no new note event
            }
            CURRENT_NOTE = note;
            // send play event
            note_to_serial(note);
        }

        // in case were in search mode, ie toggled loop to find other pressed notes
        if (note.isPressed() && TOGGLE_LOOP) {
            if (note.get_note() == CURRENT_NOTE.get_note()) {
                // same note value
                // only velocity change, no new note event
                Serial.println("no new event");
            }
            CURRENT_NOTE = note;
            note_to_serial(note);
            // change pitch & velocity
            // reset toggle
            TOGGLE_LOOP = false;
        }

        // in case were on current note and it switches to off
        if ((note.get_note() == CURRENT_NOTE.get_note()) && (note.get_velocity() == CURRENT_NOTE.get_velocity()) && !note.isPressed() && CURRENT_NOTE.isPressed()) {
            TOGGLE_LOOP = !TOGGLE_LOOP;
            if (!TOGGLE_LOOP) {
                // one loop through other notes and nothing changed:
                CURRENT_NOTE.setPush(false);  // makes sure timer is reset -> solves case for same note going off and on again
                Serial.println("last off event");
                // send stop event
            }
        }
    }
}

/**
 * Multiplexer, setting, reading
 */
void init_multiplexer() {
    for (int mux_idx=0; mux_idx<NO_OF_MODULES; mux_idx++) {
        mux_modules[mux_idx].setAnalogIn(AIN_PINS_MUX[mux_idx]);
    }
 }

void mux_update() {
    if (UPDATE_TIMER >= UPDATE_INTERVAL) {
        UPDATE_TIMER = 0;
        if (!TOGGLE_MUX_SET_READ) {
            // Set multiplexer address (enough for one module -> all share same address line)
            mux_modules[0].setAddressPins(INDEX_MUX_READ);
            TOGGLE_MUX_SET_READ = true;     // setup read
        }
        else {
            // read values
            }
    }
}

/**
 * void mux_update() {
    if (read_toggle == 0) {
        mux_set(reading_index);
    }
    else {
        mux_1_control(reading_index);
        mux_2_control(reading_index);
        mux_3_control(reading_index);
        mux_4_control(reading_index);
    }
    read_toggle++;
    if (read_toggle >=3) {
        read_toggle=0;
        reading_index++;
    }
    if (reading_index >=8) reading_index = 0;
}
*/

/**
 * program
 */
void setup()
{
    // setup hardware
    Serial.begin(9600);
    AudioMemory(300);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);
    analogReadResolution(READ_RES_BIT_DEPTH);  // also here, 12 bit would mean 0 - 4096 value range in analog read

    // initialize keys
    init_key_notes();
    // initialize mux objects
    init_multiplexer();

    // initialize mcps
    MCP_KEYS.begin();
    for (int out_idx = 8; out_idx <= 15; out_idx++) {
        MCP_KEYS.pinMode(out_idx, OUTPUT);
        MCP_KEYS.digitalWrite(out_idx, LOW);
    }
    for (int in_idx = 0; in_idx <= 7; in_idx++) {
        MCP_KEYS.pinMode(in_idx, INPUT);
    }

    MCP_DIO.begin(1);
    for (int mux_idx = 8; mux_idx < 11; mux_idx++) {
        MCP_DIO.pinMode(mux_idx, OUTPUT);
    }
}

void loop()
{
    keybed_read();
    note_update();
}