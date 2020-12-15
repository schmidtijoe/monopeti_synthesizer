#include <Arduino.h>
#include <i2c_t3.h>
// include audio system design tool code
#include "audio_system.h"
#include "note.h"
#include "mux.h"
#include <Bounce.h>

/**
 * Global variables
 */
 // DEBUG
elapsedMicros DEBUG_TIMER_MUX;
elapsedMillis DEBUG_TIMER_KEYS;

// keybed
const int keyOutPins[8] = {25, 26, 27, 28, 29, 30, 3, 4};  // keyboard out pins on digital io rail
constexpr u_int8_t MCP_KEYS_ADDR = 0x20;
constexpr u_int8_t MCP_DIO_ADDR = 0x21;
constexpr u_int8_t NO_OF_KEYS = 64;     // total number of keys, i.e. toggle switches

// analog reading
constexpr u_int8_t READ_RES_BIT_DEPTH = 8;  // bit depth of reading resolution
int RES_RANGE = (int) pow(2, READ_RES_BIT_DEPTH) - 1;  // analog reading range, dependent on bit depth
elapsedMicros UPDATE_TIMER;

// multiplexers
unsigned int UPDATE_INTERVAL = 100;       // [us] reading interval for analog inputs through multiplexers

constexpr u_int8_t NO_OF_MODULES = 4;
int AIN_PINS_MUX[NO_OF_MODULES] = {35, 34, 33, 32};
bool TOGGLE_MUX_SET_READ = false;       // toggle between setting and reading mux
int MUX_READ_INDEX = 0;

// digital IO
elapsedMillis UPDATE_TIMER_2;  // no need to do this every loop
unsigned int UPDATE_INTERVAL_2 = 5;  // [ms]
elapsedMillis SYNC_TIMER;
// pins -> have to switch to MCP keys IO capacity here
u_int8_t dioRead = 0;
u_int8_t dioState[8] = {0, 0, 0, 0, 0, 0, 0, 0};
u_int8_t dioComp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

const int pinLedSync = 25;
Bounce pinOctRight = Bounce(26, 10);
Bounce pinOctLeft = Bounce(27, 10);
const int pinTempoLeft = 28;
const int pinTempoRight = 29;
Bounce pinTempoTap = Bounce(30, 10);
const int pinEnvLeft = 3;
const int pinEnvRight = 4;
int octControlToggle = 0;
bool toggleLedSync = false;

// note & midi interfacing
constexpr byte HALF_VEL = 90;
bool TOGGLE_LOOP = false;       // toggles note searching for mono-keyboard use

// controls
// osc
int oscWaveState[3] = {0, 0, 0};
short oscWaveForms[4] = {WAVEFORM_SINE, WAVEFORM_SQUARE, WAVEFORM_TRIANGLE, WAVEFORM_SAWTOOTH};
float detune = 0.0;
int octave = 0;
int subOsc2 = 0;
int subOsc3 = 0;
// volume
float vol = 0.0;
const int fadeTime = 15;        // fade timer upon velocity changes = shortest envelope ramp

// envelopes
AudioEffectEnvelope* ccEnvelope;     // pointer

// create array of notes and current note
Note notes[NO_OF_KEYS];
static Note CURRENT_NOTE;
const Note NULL_NOTE;

Mux multiplexer_modules[NO_OF_MODULES];

/**
 * functions
 */
// calculate frequency given midi note value
float midi2freq(int note_number){
    float temp = static_cast<float>(note_number - 69) / static_cast<float>(12);
    return static_cast<float>(440 * pow(2, temp));
}


// controls
void setEnvelopeDefault(AudioEffectEnvelope* envDefault) {
    envDefault->attack(15.0);
    envDefault->decay(15.0);
    envDefault->sustain(1);
    envDefault->release(15.0);
}

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

// initialize MCPs over i2c
void init_mcps() {
    // -- KEYS --
    // B register
    Wire.beginTransmission(MCP_KEYS_ADDR);
    Wire.write(0x01);
    // pin assignment -> input oct toggle 12, input env switch 34, input tempo switch 56, output tempo led 7, input tempo tap 8
    // ie: 10111111
    Wire.write(0xbf);
    Wire.endTransmission();
    // need pull up
    Wire.beginTransmission(MCP_KEYS_ADDR);
    Wire.write(0x0d);
    Wire.write(0xbf);
    Wire.endTransmission();
    // all other default to input (ie. A register)

    // -- DIO --
    // set A register
    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x00); // IODIR A register
    // first 5 are output for led control, last 3 are input from sub switches -> 00000111 = 0x03
    Wire.write(0x03); // set B output pins
    Wire.endTransmission();

    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x0c); // pull up register A
    // pull up first 3 pins -> 00000111 -> 0x03
    Wire.write(0x03);
    Wire.endTransmission();

    // set oct toggle high:
    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x12); // gpio register A
    // set 5th pin high other low -> 00100000 = 0x20
    Wire.write(0x20);
    Wire.endTransmission();

    // set B register
    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x01); //IODIR B Register
    // first 3 are output pins for mux addressing, last pin is input from sub switch -> 10000000 = 0x80
    Wire.write(0x80);
    Wire.endTransmission();

    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x0d); // pull up register B
    // pull up last pin -> 10000000 -> 0x80
    Wire.write(0x80);
    Wire.endTransmission();
}

// initialize note struct -> give correct note numbers and velocities to elements
void init_key_notes() {
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
void keybed_read() {
    u_int8_t keyInput = 0;
    for (int out_idx = 0; out_idx < 8; out_idx++) {
        // set matrix out pin high
        digitalWrite(keyOutPins[out_idx], HIGH);
        // run through read
        // read the inputs of bank A
        Wire.beginTransmission(MCP_KEYS_ADDR);
        Wire.write(0x12);  // bank A
        Wire.endTransmission();
        Wire.requestFrom(MCP_KEYS_ADDR, 1);
        keyInput=Wire.read();

        for (int in_idx = 0; in_idx < 8; in_idx++) {
            auto total_idx = out_idx * 8 + in_idx;
            auto key_state = static_cast<bool>(keyInput % 2);
            notes[total_idx].setPush(key_state);
            keyInput /= 2;
        }
        // reset pin
        digitalWrite(keyOutPins[out_idx], LOW);
    }
}

// set frequencies, play notes
void setOsc(const Note & note2set, const int * octave2set, const int * sub1, const int * sub2) {
    // calculate freq from midi note and set
    OSC1.frequency(midi2freq(*octave2set * 12 + note2set.get_note()));
    OSC2.frequency(midi2freq(*octave2set * 12 + *sub1 * 12 + note2set.get_note()));
    OSC3.frequency(midi2freq(*octave2set * 12 + *sub2 * 12 + note2set.get_note()));
    pink.noteOn(midi2freq(*octave2set * 12 + note2set.get_note()), 0.8);
    if (note2set.get_velocity() == 127) {
        vel_change_high.fadeIn(fadeTime);
        vel_change_low.fadeOut(fadeTime);
    }
    else {
        vel_change_high.fadeOut(fadeTime);
        vel_change_low.fadeIn(fadeTime);
    }
}

void playNote(const Note note2play) {
    // set OSCs
    setOsc(note2play, &octave, &subOsc2, &subOsc3);
    // start envelopes
}

void stopNote() {
    // set OSCs zero note?
    setOsc(NULL_NOTE, &octave, &subOsc2, &subOsc3);
    pink.noteOff(0.8);
    // stop envelopes
}

// identifies note to play
void note_update() {
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
            setOsc(note, &octave, &subOsc2, &subOsc3);
            if (note.get_note() == CURRENT_NOTE.get_note()) {
                // same note value
                Serial.println("no new event");
                // Velocity change

            }
            else {
                // send play event
                playNote(note);
            }
            note_to_serial(note);
            CURRENT_NOTE = note;
        }

        // in case were in search mode, ie toggled loop to find other pressed notes
        if (note.isPressed() && TOGGLE_LOOP) {
            setOsc(note, &octave, &subOsc2, &subOsc3);      // changes pitch and velocity,
            if (note.get_note() == CURRENT_NOTE.get_note()) {
                // same note value -> only need a solution if pitch change to same pitch is faulty
                // Velocity change
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
                stopNote();
            }
        }
    }
}

/**
 * Multiplexer, setting, reading
 */
void init_mux() {
    for (int idx=0; idx<NO_OF_MODULES; idx++) {
        multiplexer_modules[idx].setAnalogOut(AIN_PINS_MUX[idx]);
    }
}

void muxControlChange(int mux_no, int ch_no, int value) {
    float ccValue = static_cast<float>(1) - static_cast<float>(value) / static_cast<float>(RES_RANGE);  // 0 to 1
    int waveform;
    switch (mux_no) {
        case 0:
            switch (ch_no) {
                case 0:
                    // ADSR - volume, attack
                    ADSR_vol.attack(static_cast<float>(8000.0 * ccValue + 15.0));   // min 15ms to 8 sec
//                    Serial.print("0 - Value change: ");
//                    Serial.println(static_cast<float>(8000.0 * ccValue + 15.0));
                    break;
                case 1:
                    // ADSR - volume, decay
                    ADSR_vol.decay(static_cast<float>(8000.0 * ccValue + 15.0));    // min 15ms to 8 sec
//                    Serial.print("1 - Value change: ");
//                    Serial.println(static_cast<float>(8000.0 * ccValue + 15.0));
                    break;
                case 2:
                    // ADSR - volume, sustain
                    ADSR_vol.sustain(ccValue);  // 0 to 1
                    // Serial.print("2 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 3:
                    // ADSR - volume, release
                    ADSR_vol.release(static_cast<float>(8000.0 * ccValue + 15.0));  // 15 ms to 8 sec
                    // Serial.print("3 - Value change: ");
                    // Serial.println(static_cast<float>(8000.0 * ccValue + 15.0));
                    break;
                case 4:
                    // ADSR - modulation, attack
                    ccEnvelope->attack(static_cast<float>(8000.0 * ccValue + 15.0));   // min 15ms to 8 sec
                    // Serial.print("4 - Value change: ");
                    // Serial.println(static_cast<float>(8000.0 * ccValue + 15.0));
                    break;
                case 5:
                    // ADSR - modulation, decay
                    // Serial.print("5 - Value change: ");
                    // Serial.println(static_cast<float>(8000.0 * ccValue + 15.0));
                    ccEnvelope->decay(static_cast<float>(8000.0 * ccValue + 15.0));    // min 15ms to 8 sec
                    break;
                case 6:
                    // ADSR - modulation, sustain
                    // Serial.print("6 - Value change: ");
                    // Serial.println(ccValue);
                    ccEnvelope->sustain(ccValue);  // 0 to 1
                    break;
                case 7:
                    // ADSR - modulation, release
                    // Serial.print("7 - Value change: ");
                    // Serial.println(static_cast<float>(8000.0 * ccValue + 15.0));
                    ccEnvelope->release(static_cast<float>(8000.0 * ccValue + 15.0));  // 15 ms to 8 sec
                    break;
                default:
                    break;
            }
            break;
        case 1:
            // second multiplexer controls - Mixer
            switch (ch_no) {
                case 0:
                    // OSC1 volume
                    mix_osc.gain(0, static_cast<float>(0.25) * ccValue);
//                    Serial.print("0 - Value change: ");
//                    Serial.println(static_cast<float>(0.25) * ccValue);
                    break;
                case 1:
                    // OSC1 wave
                    waveform = static_cast<int>(map(ccValue * 22, 0, 15, 0, 4));
                    // switch only upon change (mux object change detection in finer detail)
//                    Serial.print("1 - Value change: ");
//                    Serial.println(waveform);
                    if (waveform != oscWaveState[0]) {
                        oscWaveState[0] = waveform;
                        OSC1.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 2:
                    // OSC2 volume
                    mix_osc.gain(1, static_cast<float>(0.25) * ccValue);
//                    Serial.print("2 - Value change: ");
//                    Serial.println(static_cast<float>(0.25) * ccValue);
                    break;
                case 3:
                    // OSC2 wave
                    waveform = static_cast<int>(map(ccValue * 22, 0, 15, 0, 4));
                    // switch only upon change (mux object change detection in finer detail)
//                    Serial.print("3 - Value change: ");
//                    Serial.println(waveform);
                    if (waveform != oscWaveState[1]) {
                        oscWaveState[1] = waveform;
                        OSC2.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 4:
                    // OSC3 volume
                    mix_osc.gain(2, static_cast<float>(0.25) * ccValue);
//                    Serial.print("4 - Value change: ");
//                    Serial.println(static_cast<float>(0.25) * ccValue);
                    break;
                case 5:
                    // OSC3 wave
                    waveform = static_cast<int>(map(ccValue * 22, 0, 15, 0, 4));
                    // switch only upon change (mux object change detection in finer detail)
//                    Serial.print("5 - Value change: ");
//                    Serial.println(waveform);
                    if (waveform != oscWaveState[2]) {
                        oscWaveState[2] = waveform;
                        OSC3.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 6:
                    // OSC4 volume
                    mix_osc.gain(3, static_cast<float>(0.25) * ccValue);
                    // Serial.print("6 - Value change: ");
                    // Serial.println(static_cast<float>(0.25) * ccValue);
                    break;
                case 7:
                    // detune
                    detune = ccValue;
                    // Serial.print("7 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                default:
                    break;
            }
            break;

        case 2:
            // third multiplexer controls - lfo, lpf
            switch (ch_no) {
                case 0:
                    // lfo 1 depth
                    // Serial.print("0 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 1:
                    // lfo 2 depth
                    // Serial.print("1 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 2:
                    // lfo 3 depth
                    // Serial.print("2 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 3:
                    // lpf resonance
                    // Serial.print("3 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 4:
                    // lfo 1 speed
                    // Serial.print("4 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 5:
                    // lfo 2 speed
                    // Serial.print("5 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 6:
                    // lfo 3 speed
                    // Serial.print("6 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 7:
                    // lpf cutoff
                    // Serial.print("7 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                default:
                    break;
            }
            break;

        case 3:
            // fourth multiplexer controls - fx
            switch (ch_no) {
                case 0:
                    // Serial.print("0 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 1:
                    // Serial.print("1 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 3:
                    // Serial.print("2 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 4:
                    // Serial.print("3 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 6:
                    // Serial.print("4 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                case 7:
                    // Serial.print("5 - Value change: ");
                    // Serial.println(ccValue);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

}

void muxReadUpdate(const bool setRead) {

    Mux* mpxcc;
    if (!setRead) {
        Mux::setMultiplexerChannel(MUX_READ_INDEX);
    }
    else {
        for (int idx = 0; idx < NO_OF_MODULES; idx++) {
            mpxcc = &multiplexer_modules[idx];
            // setup read of mux object
            mpxcc->read(MUX_READ_INDEX);
            // to do: implement filter for read!!

            // only trigger control changes when they occur
            if (mpxcc->hasChanged()) {
                muxControlChange(idx, MUX_READ_INDEX, mpxcc->getChValue(MUX_READ_INDEX));
            }
        }
        MUX_READ_INDEX++;
        // reset index
        if (MUX_READ_INDEX >= 8) MUX_READ_INDEX = 0;
    }
}

/**
 * D IO control
 * normal breakout pins,
 * LED sync -> p# 25
 * octave switch -> p# 26,27
 * tempo switch -> p# 28,29
 * tap tempo -> p# 30
 * Envelope switch -> p# 3,4
 */
void setOctToggleLed(int toggleValue) {
    // have to reset register A MCP DIO output depending on the toggle value
    // 0 value is: want 00100000 written to register -> 0x40
    int stateToWrite = 0x20;
    while (toggleValue>0) {
        stateToWrite = stateToWrite << 1;
        toggleValue--;
    }
    while (toggleValue<0) {
        stateToWrite = stateToWrite >> 1;
        toggleValue++;
    }
    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x12);
    Wire.write(stateToWrite);
    Wire.endTransmission();
}

void switchDioControlChange(const unsigned int * timingInterval, const unsigned int timingSync) {
    if (UPDATE_TIMER_2 >= *timingInterval) {
        // read pins
        Wire.beginTransmission(MCP_KEYS_ADDR);
        Wire.write(0x13); // bank B
        Wire.endTransmission();
        Wire.requestFrom(MCP_KEYS_ADDR, 1); // request 1 byte
        dioRead = Wire.read();

        for (int dioIdx=0; dioIdx<8; dioIdx++) {
            dioComp[dioIdx] = dioRead % 2;
            dioRead = dioRead >> 1;
        }
        // check octave toggle
        // left
        if (dioComp[0] != dioState[0]) {
            // left oct toggle changed
            if (dioComp[0] == 0) {
                // switch on
                if (octave > -2) {
                    // lower octave by 1
                    octave--;
                    setOctToggleLed(octave);
                }
            }
            dioState[0] = dioComp[0];
        }
        // right
        if (dioComp[1] != dioState[1]) {
            // left oct toggle changed
            if (dioComp[1] == 0) {
                // switch on
                if (octave < 2) {
                    // raise octave by one
                    octave++;
                    setOctToggleLed(octave);
                }
            }
            dioState[1] = dioComp[1];
        }

        // check enevlope switch
        // left
        if (dioComp[2] == 0 && dioState[2] != dioComp[2]) {
            ccEnvelope = &env_filter;
            setEnvelopeDefault(&env_pitch);
            setEnvelopeDefault(&env_off);
        }

        //right
        else if (dioComp[3] == 0 && dioState[3] != dioComp[3]) {
            ccEnvelope = &env_pitch;
            setEnvelopeDefault(&env_filter);
            setEnvelopeDefault(&env_off);
        }
        else {
            ccEnvelope = &env_off;
            setEnvelopeDefault(&env_filter);
            setEnvelopeDefault(&env_pitch);
        }
        dioState[2] = dioComp[2];
        dioState[3] = dioComp[3];

        // check tempo switch
        if (dioComp[4] == 0 && dioComp[4] != dioState[4]) {
            // sync to midi
        }
        else if (dioComp[5] == 0 && dioComp[5]!= dioState[5]) {
            // sync to knob
        }
        else {
            // sync to tap
        }
        dioState[4] = dioComp[4];
        dioState[5] = dioComp[5];
        dioState[7] = dioComp[7];
    }
    // sync LED switch
    if (SYNC_TIMER >= timingSync) {
        SYNC_TIMER = 0;
        Wire.beginTransmission(MCP_KEYS_ADDR);
        Wire.write(0x13); // bank B
        Wire.write(0x40); // pin 7 high
        Wire.endTransmission();
        toggleLedSync = true;
    }
    if (SYNC_TIMER >= 50 && toggleLedSync) {
        // blink light for 50 ms
        Wire.beginTransmission(MCP_KEYS_ADDR);
        Wire.write(0x13); // bank B
        Wire.write(0x00); // pin all low
        Wire.endTransmission();
        toggleLedSync = false;
    }
}

void aioControlChange() {
    // make this timing dependent?
    vol = static_cast<float>(analogRead(A1)) / static_cast<float>(RES_RANGE);
    Volume.gain(vol);
}
/**
 * program
 */
void setup()
{   delay(4000);
    // setup hardware
    Wire.begin(I2C_MASTER, MCP_KEYS_ADDR, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_DMA);
    Wire.begin(I2C_MASTER, MCP_DIO_ADDR, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_DMA);

    init_mcps();

    Serial.begin(9600);
    AudioMemory(100);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);

    // ADCs
    pinMode(A13, INPUT);
    pinMode(A14, INPUT);
    pinMode(A15, INPUT);        // pin no 34
    pinMode(A16, INPUT);       // pin no 35

    analogReadResolution(READ_RES_BIT_DEPTH);  // also here, 12 bit would mean 0 - 4096 value range in analog read
    analogReadAveraging(8);

    // initialize keys
    for (auto pinIdx : keyOutPins){
        pinMode(pinIdx, OUTPUT);
        digitalWrite(pinIdx, LOW);
    }
    init_key_notes();
    // initialize mux objects
    init_mux();

    // Initialize OSC
    OSC1.begin(1.0, 0.0, oscWaveForms[oscWaveState[0]]);
    OSC2.begin(1.0, 0.0, oscWaveForms[oscWaveState[1]]);
    OSC3.begin(1.0, 0.0, oscWaveForms[oscWaveState[2]]);
    pink.noteOn(0, 0.8);
}

void loop()
{
    DEBUG_TIMER_KEYS = 0;
    // set up mux channel
    muxReadUpdate(false);
    // keybed read
    keybed_read();
    note_update();
    // read mux
    muxReadUpdate(true);
    switchDioControlChange(&UPDATE_INTERVAL_2, 670);
    aioControlChange();
}