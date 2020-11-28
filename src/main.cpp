#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include "wiring.h"
// include audio system design tool code
#include "audio_system.h"

/**
 * Global variables
 */
// keybed
Adafruit_MCP23017 MCP_KEYS;     // gpio expander handling keyboard
constexpr u_int8_t NO_OF_KEYS = 64;     // total number of keys, i.e. toggle switches

// analog reading
constexpr u_int8_t READ_RES_BIT_DEPTH = 10;  // bit depth of reading resolution
int RES_RANGE = (int) (2 << READ_RES_BIT_DEPTH) - 1;  // analog reading range, dependent on bit depth
elapsedMillis UPDATE_TIMER;
// multiplexers
unsigned int UPDATE_INTERVAL = 2;       // [ms] reading interval for analog inputs through multiplexers
constexpr u_int8_t NO_OF_MODULES = 4;
int AIN_PINS_MUX[NO_OF_MODULES] = {35, 34, 33, 31};
bool TOGGLE_MUX_SET_READ = false;       // toggle between setting and reading mux
int MUX_READ_INDEX = 0;

// note & midi interfacing
constexpr byte HALF_VEL = 90;
bool TOGGLE_LOOP = false;       // toggles note searching for mono-keyboard use

// digital outputs
Adafruit_MCP23017 MCP_DIO;      // gpio expander, handling multiplexer switches and LED controls

// controls
// osc
int oscWaveState[3] = {0, 0, 0};
short oscWaveForms[4] = {WAVEFORM_SINE, WAVEFORM_SQUARE, WAVEFORM_TRIANGLE, WAVEFORM_SAWTOOTH};
bool ENVELOPE_TARGET_SWITCH = false;

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

class Mux {
public:
    Mux() : MUX_ADDR_PINS(), chx_values(), analog_out_pin(), state_change() {};
    // number of multiplexer modules for analog read ins, default values from design
    const int MUX_ADDR_PINS[3]{8, 9, 10};

    void setAnalogOut(int a_pin) {
        // set the output pin of the object
        analog_out_pin = a_pin;
    }

    void setAddrRead(bool toggle_read, int ch_no) {
        if (toggle_read) {
            // read values
            read(ch_no);
        }
        else {
            // set addr pins
            setAddrPins(ch_no);
        }
    }

    int getChValue (int ch_no) const {
        return chx_values[ch_no];
    }

    bool hasChanged() const {
        return state_change;
    }

private:
    int chx_values[8]{};
    int analog_out_pin{};
    bool state_change{false};

    void setAddrPins (int ch_no) {
        // calculate 3 bit number from channel number to set addr pins high or low and address mux channel
        auto bit_calc = ch_no;
        bool bit;
        for (int idx : MUX_ADDR_PINS) {
            bit = static_cast<bool>(bit_calc % 2);
            bit_calc /= 2;
            MCP_DIO.digitalWrite(idx, bit);
        }
    }

    void read (int ch_no) {
        auto read_value = abs(RES_RANGE - analogRead(analog_out_pin));
        if (read_value != chx_values[ch_no]) {
            chx_values[ch_no] = read_value;
            state_change = true;
        }
        else state_change = false;
    }
};

Mux multiplexer_modules[NO_OF_MODULES];

/**
 * functions
 */
// controls
void setEnvelopeDefault(AudioEffectEnvelope envDefault) {
    envDefault.attack(15.0);
    envDefault.decay(15.0);
    envDefault.sustain(1);
    envDefault.release(15.0);
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
void init_mux() {
    for (int idx=0; idx<NO_OF_MODULES; idx++) {
        multiplexer_modules[idx].setAnalogOut(AIN_PINS_MUX[idx]);
    }
}

void muxControlChange(int mux_no, int ch_no, int value) {
    float ccValue = static_cast<float>(value) / static_cast<float>(RES_RANGE);  // 0 to 1
    int waveform;
    switch (mux_no) {
        case 0:
            // first multiplexer controls - ADSR
            AudioEffectEnvelope ccEnvelope;
            if (ENVELOPE_TARGET_SWITCH) {
                ccEnvelope = env_filter;
                setEnvelopeDefault(env_pitch);
            }
            else {
                ccEnvelope = env_pitch;
                setEnvelopeDefault(env_filter);
            }
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
                    ccEnvelope.attack(static_cast<float>(8000.0 * ccValue + 15.0));   // min 15ms to 8 sec
                    break;
                case 5:
                    // ADSR - modulation, decay
                    ccEnvelope.decay(static_cast<float>(8000.0 * ccValue + 15.0));    // min 15ms to 8 sec
                    break;
                case 6:
                    // ADSR - modulation, sustain
                    ccEnvelope.release(ccValue);  // 0 to 1
                    break;
                case 7:
                    // ADSR - modulation, release
                    ccEnvelope.release(static_cast<float>(8000.0 * ccValue + 15.0));  // 15 ms to 8 sec
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
                    mix_osc.gain(0, 0.25 * ccValue);
                    break;
                case 1:
                    // OSC1 wave
                    waveform = map(static_cast<int>(ccValue) * RES_RANGE, 0, RES_RANGE, 0, 3);
                    // switch only upon change (mux object change detection in finer detail)
                    if (waveform != oscWaveState[0]) {
                        oscWaveState[0] = waveform;
                        OSC1.begin(oscWaveForms[waveform]);
                    }
                    break;
                case 2:
                    // OSC2 volume
                    mix_osc.gain(1, 0.25 * ccValue);
                    break;
                case 3:
                    // waveform & sub switch OSC2
                    waveform = 0;
                    break;
                case 4:
                    break;
                case 5:
                    break;
                case 6:
                    break;
                case 7:
                    break;
                default:
                    break;
            }
            break;

        case 2:
            // third multiplexer controls - lfo, lpf
            break;

        case 3:
            // fourth multiplexer controls - fx
            break;

        default:
            break;
    }

}


void mux_update() {
    if (UPDATE_TIMER >= UPDATE_INTERVAL) {
        UPDATE_TIMER = 0;
        // set up loop
        if (!TOGGLE_MUX_SET_READ) {
            for (auto mpx: multiplexer_modules) {
                mpx.setAddrRead(false, MUX_READ_INDEX);
            }
        }

        // reading loop
        else {
            for (int idx=0; idx<NO_OF_MODULES; idx++) {
                multiplexer_modules[idx].setAddrRead(true, MUX_READ_INDEX);
                // only trigger control changes when they occur
                if (multiplexer_modules[idx].hasChanged()) {
                    muxControlChange(idx, MUX_READ_INDEX, multiplexer_modules[idx].getChValue(MUX_READ_INDEX));
                }
            }
            MUX_READ_INDEX++;
        }
        TOGGLE_MUX_SET_READ = !TOGGLE_MUX_SET_READ;     // switch between setup and read

        // reset index
        if (MUX_READ_INDEX >= 8) MUX_READ_INDEX = 0;
    }
}

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
    init_mux();

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