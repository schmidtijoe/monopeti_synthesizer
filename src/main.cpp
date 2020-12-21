#include <Arduino.h>
#include <i2c_t3.h>
// include audio system design tool code
#include "audio_system.h"
#include "note.h"
#include "mux.h"
#include <MIDI.h>
#include <Bounce.h>

/**
 * Global variables
 */
// DEBUG
// elapsedMicros DEBUG_TIMER_MUX;

// MIDI IO
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
int MIDI_CHANNEL = 1;

// keybed
const int keyOutPins[8] = {25, 26, 27, 28, 29, 30, 3, 4};  // keyboard out pins on digital io rail
constexpr u_int8_t MCP_KEYS_ADDR = 0x20;
constexpr u_int8_t MCP_DIO_ADDR = 0x21;
constexpr u_int8_t NO_OF_KEYS = 64;     // total number of keys, i.e. toggle switches

// analog reading
constexpr u_int8_t READ_RES_BIT_DEPTH = 8;  // bit depth of reading resolution
int RES_RANGE = (int) pow(2, READ_RES_BIT_DEPTH) - 1;  // analog reading range, dependent on bit depth
const int analogPotPins[5] = {A20, A21, A22, A6, A7};   // ring depth/speed/wet, portamento, mod

// multiplexers
constexpr u_int8_t NO_OF_MODULES = 4;
int AIN_PINS_MUX[NO_OF_MODULES] = {35, 34, 33, 32};

// digital IO
elapsedMillis SYNC_TIMER;
// pins -> have to switch to MCP keys IO capacity here
u_int8_t dioRead = 0;
u_int8_t dioState[8] = {0, 0, 0, 0, 0, 0, 0, 0};
u_int8_t dioComp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
Bounce tempoTap = Bounce(36, 10);

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
// modulation
const int modOctRange = 2;  // number of octaves frequency modulation can maximally shift the wave
const int modPhRange = 180;  // modulation signal can change the phase that many degrees max
// volume
float vol = 0.0;
const int fadeTime = 15;        // fade timer upon velocity changes = shortest envelope ramp
// ring
float ringDepth = 0;
float ringSpeed = 0;
float ringWet = 0;
// delay
int delayTaps = 1;
// portamento
unsigned int portamentoCountTo = 2;  // set number of loops to update portamento
float activeFreq = 440;
float desiredFreq = 440;
float freqInc = 0;
unsigned int portamentoCounter = 0;
// create array of notes and current note
Note notes[NO_OF_KEYS];
static Note CURRENT_NOTE;

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

void setEffectMixOnOff(const bool OnOff, AudioMixer4* envelopeMixer) {
    if (OnOff) {
        // set mixer turn off direct signal and on enveloped signal
        envelopeMixer->gain(0, 0);
        envelopeMixer->gain(1, 1);
        envelopeMixer->gain(2, 0);
        envelopeMixer->gain(3, 0);

    }
    else {
        //set mixer to bypass envelope
        envelopeMixer->gain(0, 1);
        envelopeMixer->gain(1, 0);
        envelopeMixer->gain(2, 0);
        envelopeMixer->gain(3, 0);

    }

}

void setModOsc(int mode) {
    if (mode == 0) {
        OSC1.frequencyModulation(modOctRange);
        OSC2.frequencyModulation(modOctRange);
        OSC3.frequencyModulation(modOctRange);
    }
    else {
        OSC1.phaseModulation(modPhRange);
        OSC2.phaseModulation(modPhRange);
        OSC3.phaseModulation(modPhRange);
    }
}

void setDelayDecay(unsigned int taps, const float msDecay) {
    for (unsigned int delIdx=0; delIdx<taps; delIdx++){
        delay_fx.delay(delIdx, float(1 + delIdx) * msDecay);
    }
}

// Debug
/**
 * testing/debugging functions
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
*/

// initialize MCPs over i2c
void init_MCPs() {
    // -- KEYS --
    // B register
    Wire.beginTransmission(MCP_KEYS_ADDR);
    Wire.write(0x01);
    // pin assignment -> input oct toggle 12, input env switch 34, input 56 sub osc2, input sub osc3 78
    // ie: 11111111
    Wire.write(0xff);
    Wire.endTransmission();
    // need pull up
    Wire.beginTransmission(MCP_KEYS_ADDR);
    Wire.write(0x0d);
    Wire.write(0xff);
    Wire.endTransmission();
    // all other default to input (ie. A register)

    // -- DIO --
    // set A register
    Wire.beginTransmission(MCP_DIO_ADDR);
    Wire.write(0x00); // IODIR A register
    //  all are output for led controls
    Wire.write(0x00); // set B output pins
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
    // first 3 are output pins for mux addressing
    Wire.write(0x00);
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

// initialize hard coded objects
void init_sound() {
    // initialize mixers
    for (int chIdx = 0; chIdx<4; chIdx++) {
        mix_osc.gain(chIdx, 0.25);
    }

    velocity_env.gain(0, static_cast<float>(HALF_VEL) / static_cast<float>(127));
    velocity_env.gain(1, 1.0);
    velocity_env.gain(2, 0);
    velocity_env.gain(3, 0);

    vel_change_high.fadeIn(15);

    Volume.gain(0.5);

    // Initialize OSC
    OSC1.begin(1.0, 0.0, oscWaveForms[oscWaveState[0]]);
    OSC2.begin(1.0, 0.0, oscWaveForms[oscWaveState[1]]);
    OSC3.begin(1.0, 0.0, oscWaveForms[oscWaveState[2]]);
    pink.amplitude(1);

    // initialize envelopes
    setEnvelopeDefault(&ADSR_vol);
    setEnvelopeDefault(&env_pitch);
    setEnvelopeDefault(&env_filter);
    setEffectMixOnOff(false, &env_filter_onoff);
    setEffectMixOnOff(false, &env_pitch_onoff);

    // lfos
    //vca (3)
    lfoVca.begin(1.0, 0.0, WAVEFORM_SINE);
    lfoVca.offset(1.0);
    // 1
    LFO1.begin(1.0, 0.0, WAVEFORM_TRIANGLE);
    setModOsc(0); // initialize with freq modulation
    // 2
    LFO2.begin(1.0, 0.0, WAVEFORM_TRIANGLE);

    // ring modulator
    setEffectMixOnOff(false, &mix_ring_wet);
    dc_ring.amplitude(1);
    LFO_ring.begin(1.0, 0.0, WAVEFORM_SAWTOOTH);

    // lpf
    LPF.octaveControl(2);
    LPF.resonance(0.7);
    LPF.frequency(10000);
    filter_attenuator.gain(0.85); // try and error overshooting with resonance without attenuation can cause clipping

    // FX
    // delay
    setDelayDecay(2, 20);
    setEffectMixOnOff(false, &mix_delay_wet);
    mix_delay.gain(0,0.7);
    mix_delay.gain(1, 0.5);
    mix_delay.gain(2,0.3);
    mix_delay.gain(3, 0.1);

    // reverb
    setEffectMixOnOff(false, &mix_verb_wet_r);
    setEffectMixOnOff(false, &mix_verb_wet_l);
    freeverbs.roomsize(0);
    freeverbs.damping(0);
}

/** Multiplexer, setting, reading
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
                    break;
                case 1:
                    // ADSR - volume, decay
                    ADSR_vol.decay(static_cast<float>(8000.0 * ccValue + 15.0));    // min 15ms to 8 sec
                    break;
                case 2:
                    // ADSR - volume, sustain
                    ADSR_vol.sustain(ccValue);  // 0 to 1
                    break;
                case 3:
                    // ADSR - volume, release
                    ADSR_vol.release(static_cast<float>(8000.0 * ccValue + 15.0));  // 15 ms to 8 sec
                    break;
                case 4:
                    // ADSR - modulation, attack
                    env_pitch.attack(static_cast<float>(8000.0 * ccValue + 15.0));   // min 15ms to 8 sec
                    env_filter.attack(static_cast<float>(8000.0 * ccValue + 15.0));   // min 15ms to 8 sec
                    break;
                case 5:
                    // ADSR - modulation, decay
                    env_pitch.decay(static_cast<float>(8000.0 * ccValue + 15.0));    // min 15ms to 8 sec
                    env_filter.decay(static_cast<float>(8000.0 * ccValue + 15.0));    // min 15ms to 8 sec
                    break;
                case 6:
                    // ADSR - modulation, sustain
                    env_pitch.sustain(ccValue);  // 0 to 1
                    env_filter.sustain(ccValue);
                    break;
                case 7:
                    // ADSR - modulation, release
                    env_pitch.release(static_cast<float>(8000.0 * ccValue + 15.0));  // 15 ms to 8 sec
                    env_filter.release(static_cast<float>(8000.0 * ccValue + 15.0));  // 15 ms to 8 sec
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
                    break;
                case 1:
                    // OSC1 wave
                    waveform = static_cast<int>(map(ccValue * 22, 0, 15, 0, 4));
                    // switch only upon change (mux object change detection in finer detail)
                    if (waveform != oscWaveState[0]) {
                        oscWaveState[0] = waveform;
                        OSC1.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 2:
                    // OSC2 volume
                    mix_osc.gain(1, static_cast<float>(0.25) * ccValue);
                    break;
                case 3:
                    // OSC2 wave
                    waveform = static_cast<int>(map(ccValue * 22, 0, 15, 0, 4));
                    // switch only upon change (mux object change detection in finer detail)
                    if (waveform != oscWaveState[1]) {
                        oscWaveState[1] = waveform;
                        OSC2.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 4:
                    // OSC3 volume
                    mix_osc.gain(2, static_cast<float>(0.25) * ccValue);
                    break;
                case 5:
                    // OSC3 wave
                    waveform = static_cast<int>(map(ccValue * 22, 0, 15, 0, 4));
                    // switch only upon change (mux object change detection in finer detail)
                    if (waveform != oscWaveState[2]) {
                        oscWaveState[2] = waveform;
                        OSC3.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 6:
                    // OSC4 volume
                    mix_osc.gain(3, static_cast<float>(0.25) * ccValue);
                    break;
                case 7:
                    // detune
                    detune = ccValue;
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
                    LFO1.amplitude(ccValue);
                    break;
                case 1:
                    // lfo 2 depth
                    LFO2.amplitude(ccValue);
                    break;
                case 2:
                    lfoVca.amplitude(static_cast<float>(0.01+ccValue));  // 0 to 1
                    break;
                case 3:
                    // lpf resonance
                    LPF.resonance(float(4 * ccValue + 0.7));  // 0.7 to 4.7
                    break;
                case 4:
                    // lfo 1 speed
                    LFO1.frequency(static_cast<float>(40 * ccValue)); // 0 to 40 Hz
                    break;
                case 5:
                    // lfo 2 speed
                    LFO2.frequency(static_cast<float>(40 * ccValue)); // 0 to 40 Hz
                    break;
                case 6:
                    // lfo 3 speed
                    lfoVca.frequency(ccValue * 10);     // 0 to 20 Hz limited by update loop
                    break;
                case 7:
                    // lpf cutoff
                    LPF.frequency(10000 * ccValue);   // 0 to 10kHz
                    break;
                default:
                    break;
            }
            break;

        case 3:
            // fourth multiplexer controls - fx
            switch (ch_no) {
                case 0:
                    // reverb dampening
                    freeverbs.damping(ccValue); // 0 to 1
                    break;
                case 1:
                    // reverb wet dry
                    mix_verb_wet_l.gain(0, 1 - ccValue);
                    mix_verb_wet_l.gain(1, ccValue);
                    mix_verb_wet_r.gain(0, 1- ccValue);
                    mix_verb_wet_r.gain(1, ccValue);
                    break;
                case 4:
                    // reverb room size
                    freeverbs.roomsize(ccValue); // 0 to 1
                    break;

                case 3:
                    // delay wet
                    mix_delay_wet.gain(0, 1 - ccValue);
                    mix_delay_wet.gain(1, ccValue);
                    break;
                case 6:
                    // delay decay time max 1600 ms / 4 = 400 ms
                    setDelayDecay(delayTaps, float(400 * ccValue + 1));
                    break;
                case 7:
                    // delay size
                    if (delayTaps != map(int(ccValue*100), 0 , 100, 1, 4)) {
                        delayTaps = map(int(ccValue*100), 0 , 100, 1, 4);
                        Serial.print("5 - Value change: ");
                        Serial.println(delayTaps);
                        for (int delIdx=7; delIdx>delayTaps; delIdx--) {
                            delay_fx.disable(delIdx);
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

}

// set frequencies, play notes
void setOsc(const float freq2set, const int * octave2set, const int * sub1, const int * sub2) {
    // calculate freq from midi note and set
    OSC1.frequency(freq2set * static_cast<float>(pow(2,*octave2set)));
    OSC2.frequency(freq2set * static_cast<float>(pow(2, *octave2set + *sub1)));
    OSC3.frequency(freq2set * static_cast<float>(pow(2, *octave2set + *sub2)));
}

void setVelocity(const Note note2set) {
    // set velocity
    if (note2set.get_velocity() == 127) {
        vel_change_high.fadeIn(fadeTime);
        vel_change_low.fadeOut(fadeTime);
    }
    else if (note2set.get_velocity() == HALF_VEL) {
        vel_change_low.fadeIn(fadeTime);
        vel_change_high.fadeOut(fadeTime);
    }
}

void portamentoStart(Note note2set) {
    // sets Shift increment and counter
    desiredFreq = midi2freq(note2set.get_note());
    Serial.print("desired Frequency: ");
    Serial.println(desiredFreq);
    portamentoCounter = 0;
    if (portamentoCountTo < 1) {
        activeFreq = desiredFreq;
        freqInc = 0;
    }
    else {
        Serial.print("count to: ");
        Serial.println(portamentoCountTo);
        freqInc = static_cast<float> (desiredFreq - activeFreq) / static_cast<float>(portamentoCountTo);
    }
    setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
}

void portamentoUpdate() {
    if (portamentoCounter<=portamentoCountTo) {
        activeFreq += freqInc;
        portamentoCounter++;
        setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        Serial.print("active freq: ");
        Serial.println(activeFreq);
    }
    else {
        if (activeFreq != desiredFreq) {
            activeFreq = desiredFreq;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        }
    }
}

void playNote(Note note2play) {
    // set OSCs
    portamentoStart(note2play);
    //const Note note2play
    // setOsc(note2play, &octave, &subOsc2, &subOsc3);
    // start envelopes
    ADSR_vol.noteOn();
    env_pitch.noteOn();
    env_filter.noteOn();
}

void stopNote() {
    // stop envelopes
    ADSR_vol.noteOff();
    env_pitch.noteOff();
    env_filter.noteOff();
}

// identifies note to play
void note_update() {
    /**
     * need function that sends:
     * - > note on event when no note pressed previously (even if the last note off event is the same note!!!)
     * - > note off event for all notes if no note is pressed anymore
     * - > note pitch and velocity change if note goes off but others are still pressed
     *
     * what happens if note is on and new note is pressed?          -> new on event  condition A -> note.isPressed true & highest getTime()
     * what happens if no note is played and new is pressed?        -> new on event   condition A -> note.isPressed true & highest getTime()
     * what happens if note goes off and other is on?               -> pitch & vel change, no new event (extra condition, extra toggle? one more loop til off?)
     * what happens if note goes off and no other is on             -> new off event
     * what happens with velocity changes?                          -> no play events
     */
    for (const auto &note: notes) {
        // new note pressed
        if (note.isPressed() && (note.getTime() > CURRENT_NOTE.getTime())) {
            setVelocity(note);
            // setOsc(note, &octave, &subOsc2, &subOsc3);
            if (!(note.get_note() == CURRENT_NOTE.get_note() && CURRENT_NOTE.isPressed())) {
                playNote(note);
            }
            CURRENT_NOTE = note;
        }

        // in case were in search mode, ie toggled loop to find other pressed notes
        if (note.isPressed() && TOGGLE_LOOP) {
            setVelocity(note);
            // setOsc(note, &octave, &subOsc2, &subOsc3);      // changes pitch and velocity,
            portamentoStart(note);
            CURRENT_NOTE = note;
            // reset toggle
            TOGGLE_LOOP = false;
        }

        // in case were on current note and it switches to off
        if ((note.get_note() == CURRENT_NOTE.get_note()) && (note.get_velocity() == CURRENT_NOTE.get_velocity()) && !note.isPressed() && CURRENT_NOTE.isPressed()) {
            TOGGLE_LOOP = !TOGGLE_LOOP;
            if (!TOGGLE_LOOP) {
                // one loop through other notes and nothing changed:
                CURRENT_NOTE.setPush(false);  // makes sure timer is reset -> solves case for same note going off and on again
                // send stop event
                stopNote();
            }
        }
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
    // 0 value is: want 00100000 written to register -> 0x20
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

void switchDioControlChange(const unsigned int timingSync) {
    // read pins
    Wire.beginTransmission(MCP_KEYS_ADDR);
    Wire.write(0x13); // bank B
    Wire.endTransmission();
    Wire.requestFrom(MCP_KEYS_ADDR, 1u); // request 1 byte
    dioRead = Wire.read();

    for (unsigned char & dioIdx : dioComp) {
        dioIdx = dioRead % 2;
        dioRead = dioRead >> 1;
    }
    // check octave toggle
    // left
    if (dioComp[0] != dioState[0]) {
        // left oct toggle changed
        if (dioComp[0] == 0) {
            // switch on
            if (octave < 2) {
                // raise octave by one
                octave++;
                setOctToggleLed(octave);
                setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
            }
        }
        dioState[0] = dioComp[0];
    }
    // right
    if (dioComp[1] != dioState[1]) {
        // left oct toggle changed
        if (dioComp[1] == 0) {
            // switch on
            if (octave > -2) {
                // lower octave by 1
                octave--;
                setOctToggleLed(octave);

                setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
            }
        }
        dioState[1] = dioComp[1];
    }

    // check envelope switch
    // left
    if (dioComp[2] == 0 && dioState[2] != dioComp[2]) {
        setEffectMixOnOff(true, &env_filter_onoff);
        setEffectMixOnOff(false, &env_pitch_onoff);
    }
    //right
    else if (dioComp[3] == 0 && dioState[3] != dioComp[3]) {
        setEffectMixOnOff(false, &env_filter_onoff);
        setEffectMixOnOff(true, &env_pitch_onoff);
    }
    else if (dioComp[2] == 1 && dioComp[3] == 1 && (dioComp[3] != dioState[3] || dioComp[2] != dioState[2])){
        setEffectMixOnOff(false, &env_filter_onoff);
        setEffectMixOnOff(false, &env_pitch_onoff);
    }
    dioState[2] = dioComp[2];
    dioState[3] = dioComp[3];

    // check sub switches
    // sub 3
    if (dioComp[4] == 0) {
        // osc2 sub left
        if (subOsc3 != -1) {
            subOsc3 = -1;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        };
    }
    else if (dioComp[5] == 0) {
        // osc2 sub off
        if (subOsc3 != 0) {
            subOsc3 = 0;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        }
    }
    else {
        // sub 3 right
        if (subOsc3 != 1) {
            subOsc3 = 1;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        }
    }
    // sub 2
    if (dioComp[6] == 0) {
        // osc2 sub off
        if (subOsc2 != 0) {
            subOsc2 = 0;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        }
    }
    else if (dioComp[7] == 0) {
        // osc2 sub right
        if (subOsc2 != 1) {
            subOsc2 = 1;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        }
    }
    else {
        // sub 2 left
        if (subOsc2 != -1) {
            subOsc2 = -1;
            setOsc(activeFreq, &octave, &subOsc2, &subOsc3);
        }
    }

    // sync LED switch
    // LED at last pin of A bank DIO MCP
    int stateToWrite = 0x20;
    int octaveState = octave;

    if (SYNC_TIMER >= timingSync) {
        SYNC_TIMER = 0;
        while (octaveState>0) {
            stateToWrite = stateToWrite << 1;
            octaveState--;
        }
        while (octaveState<0) {
            stateToWrite = stateToWrite >> 1;
            octaveState++;
        }
        stateToWrite += 1;
        Wire.beginTransmission(MCP_DIO_ADDR);
        Wire.write(0x12);
        Wire.write(stateToWrite);
        Wire.endTransmission();
        toggleLedSync = true;
    }
    if (SYNC_TIMER >= 50 && toggleLedSync) {
        // blink light for 50 ms
        while (octaveState>0) {
            stateToWrite = stateToWrite << 1;
            octaveState--;
        }
        while (octaveState<0) {
            stateToWrite = stateToWrite >> 1;
            octaveState++;
        }
        Wire.beginTransmission(MCP_DIO_ADDR);
        Wire.write(0x12);
        Wire.write(stateToWrite);
        Wire.endTransmission();
        toggleLedSync = false;
    }
}

void aioControlChange() {
    unsigned int portaComp;
    // make this timing dependent?
    vol = static_cast<float>(analogRead(A1)) / static_cast<float>(RES_RANGE);
    Volume.gain(vol);

    // ring modulator
    ringDepth = static_cast<float>(analogRead(analogPotPins[4])) / static_cast<float>(RES_RANGE); // 0 to 1
    ringSpeed = static_cast<float>(analogRead(analogPotPins[3]));   // @ 8 bit ~ 0 to 250
    ringWet = static_cast<float>(analogRead(analogPotPins[2])) / static_cast<float>(RES_RANGE);  // 0 to 1
    // depth
    LFO_ring.amplitude(float(0.9 * ringDepth + 0.1)); // make not drop to 0 for wet pot to make more sense
    // speed
    LFO_ring.frequency(ringSpeed);
    // wet
    mix_ring_wet.gain(0, 1 - ringWet);
    mix_ring_wet.gain(1, ringWet);
    // portamento
    portaComp = analogRead(analogPotPins[1]) -1;
    if (portaComp != portamentoCountTo) {
        portamentoCountTo = portaComp;
    }
    // mod pot

    // rest digital ins, pins 36,37,38
    // tempo tap
    tempoTap.update();
    if (tempoTap.fallingEdge()) {
        // tap tempo
    }
    // tempo switch
    if (digitalRead(37) == LOW) {
        // left
        // sync tempo to midi
    }
    else if (digitalRead(38) == LOW) {
        // right
        // sync tempo to tap
    }
    else {
        // sync tempo to pots (vca)
    }


}

void muxReadUpdate(const bool setRead, const int setChannel) {

    Mux* muxCC;
    if (!setRead) {
        Mux::setMultiplexerChannel(setChannel);
    }
    else {
        for (int idx = 0; idx < NO_OF_MODULES; idx++) {
            muxCC = &multiplexer_modules[idx];
            // setup read of mux object
            muxCC->read(setChannel);
            // to do: implement filter for read!!

            // only trigger control changes when they occur
            if (muxCC->hasChanged()) {
                muxControlChange(idx, setChannel, muxCC->getChValue(setChannel));
            }
        }
    }
}

/** keybed and notes
 */
// read keybed repetitively in loop, make sure this is fast! just updates the struct with key states and timers
void keybed_read() {
    u_int8_t keyInput;
    for (int out_idx = 0; out_idx < 8; out_idx++) {
        // set matrix out pin high
        digitalWrite(keyOutPins[out_idx], HIGH);
        // set mux pins
        muxReadUpdate(false, out_idx);

        // run through read
        // read the inputs of bank A
        Wire.beginTransmission(MCP_KEYS_ADDR);
        Wire.write(0x12);  // bank A
        Wire.endTransmission();
        Wire.requestFrom(MCP_KEYS_ADDR, 1u);
        keyInput=Wire.read();

        for (int in_idx = 0; in_idx < 8; in_idx++) {
            auto total_idx = out_idx * 8 + in_idx;
            auto key_state = static_cast<bool>(keyInput % 2);
            notes[total_idx].setPush(key_state);
            keyInput /= 2;
        }
        // read mux values
        muxReadUpdate(true, out_idx);
        // reset pin
        digitalWrite(keyOutPins[out_idx], LOW);
    }
}

/** MIDI IO
 */
void midiPushNote(__unused byte midiChannel, byte midiNote, byte midiVelocity) {
    if (midiVelocity > 0 && midiVelocity < 91) {
        for (auto &note: notes) {
            if (note.get_note() == midiNote && note.get_velocity() == 90) {
                // map all incoming midi notes with velocity between 1 and 91 to our velocity 90 notes
                note.setPush(true);
            }
        }
    }
    else if (midiVelocity > 90) {
        for (auto &note: notes) {
            if (note.get_note() == midiNote && note.get_velocity() == 127) {
                // map all incoming midi notes with velocity between bigger 90 to our 127 velocity notes
                note.setPush(true);
            }
        }
    }
    else {
        // velocity 0 means midi sends note off
        for (auto &note: notes) {
            if (note.get_note() == midiNote) {
                // set all notes (90 and 127 off)
                note.setPush(false);
            }
        }
    }
    // rest should be handled by our note_update function deciding which to play
}

void init_midi() {
    MIDI.begin(MIDI_CHANNEL);
    MIDI.setHandleNoteOn(midiPushNote);
    MIDI.setHandleNoteOff(midiPushNote);
}

void fancyOctLightsSetup() {
    int lightsSetList[9] = {0,1,2,1,0,-1,-2,-1,0};
    for (auto togIdx : lightsSetList) {
        setOctToggleLed(togIdx);
        delay(200);
    }
}
/**
 * program
 */
void setup()
{
    Serial.println("___ALLOCATE___");
    AudioMemory(670);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);

    // setup hardware
    Serial.begin(9600);
    Serial.println("___SETUP HARDWARE___");
    Wire.begin(I2C_MASTER, MCP_KEYS_ADDR, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.begin(I2C_MASTER, MCP_DIO_ADDR, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    init_midi();

    // A ins
    pinMode(A13, INPUT);
    pinMode(A14, INPUT);
    pinMode(A15, INPUT);        // pin no 34
    pinMode(A16, INPUT);       // pin no 35
    // D ins
    pinMode(36, INPUT_PULLUP);
    pinMode(37, INPUT_PULLUP);
    pinMode(38, INPUT_PULLUP);

    analogReadResolution(READ_RES_BIT_DEPTH);  // also here, 12 bit would mean 0 - 4096 value range in analog read
    analogReadAveraging(8);

    Serial.println("___INIT MCPs___");
    init_MCPs();

    Serial.println("___INIT KEYS___");
    // initialize keys
    for (auto pinIdx : keyOutPins){
        pinMode(pinIdx, OUTPUT);
        digitalWrite(pinIdx, LOW);
    }
    Serial.println("___INIT KEYNOTES___");

    init_key_notes();
    // initialize mux objects
    Serial.println("___INIT MUX___");
    init_mux();
    // initialize sound objects
    Serial.println("___INIT SOUND___");
    init_sound();

    // some light switching fancy upon start
    fancyOctLightsSetup();
    Serial.println("___DONE___");

}

void loop()
{
    // keybed read - including mux
    keybed_read();
    // midi read, pushes the same note objects on off
    MIDI.read();
    // handle note objects
    note_update();
    portamentoUpdate();
    switchDioControlChange(670);
    aioControlChange();
//    Serial.print(" max memory usage: ");
//    Serial.println(AudioMemoryUsageMax());
}