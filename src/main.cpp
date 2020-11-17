#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include <elapsedMillis.h>
#include <Audio.h>

// global defs
#define MCP_KEYS_ADDRESS 0
#define no_of_keys 64

// global vars
Adafruit_MCP23017 mcp_keys;
byte half_vel = 90;

// teensy audio system design tool code
AudioControlSGTL5000    sgtl5000_1;
// declare Note structure, micros or elapsed micros here?!?!
struct Note {
    bool push_state;
    byte note_value;
    byte velocity_value;
    elapsedMicros start_time;
} notes[no_of_keys];

// initialize
void init_key_notes() {
    /* init key note_matrix
     * identified by in and output values when set and read via the mcp
     * each identifier is mapped to a note and velocity value
     * start time and logic value are initialized with 0
     */
    for (int out_idx = 0; out_idx < 8; out_idx++) {
        for (int in_idx = 0; in_idx < 8; in_idx++) {
            int idx = 8 * out_idx + in_idx;
            notes[idx].push_state = false;
            notes[idx].start_time = 0;
            switch (out_idx) {
                case 0:
                    notes[idx].velocity_value = half_vel;
                    notes[idx].note_value = 56 + in_idx;
                    break;
                case 1:
                    notes[idx].velocity_value = half_vel;
                    notes[idx].note_value = 64 + in_idx;
                    break;
                case 2:
                    notes[idx].velocity_value = half_vel;
                    notes[idx].note_value = 48 + in_idx;
                    break;
                case 3:
                    notes[idx].velocity_value = 127;
                    notes[idx].note_value = 48 + in_idx;
                    break;
                case 4:
                    notes[idx].velocity_value = 127;
                    notes[idx].note_value = 56 + in_idx;
                    break;
                case 5:
                    notes[idx].velocity_value = 127;
                    notes[idx].note_value = 64 + in_idx;
                    break;
                case 6:
                    notes[idx].velocity_value = 127;
                    notes[idx].note_value = 72 + in_idx;
                    break;
                case 7:
                    notes[idx].velocity_value = half_vel;
                    notes[idx].note_value = 72 + in_idx;
                    break;
            }
        }
    }
}

// read keybed repetitively in loop, make sure this is fast! just updates the struct with key states and timers
void keybed_read() {
    bool key_state;
    int total_idx;
    for (int out_idx=8; out_idx<16; out_idx++) {
        // set mcp pin high
        mcp_keys.digitalWrite(out_idx, HIGH);

        // run through read
        for (int in_idx=0; in_idx<8; in_idx++) {
            total_idx = (out_idx - 8) * 8 + in_idx;
            key_state = mcp_keys.digitalRead(in_idx);
            if (key_state != notes[total_idx].push_state) {
                if (key_state) {
                    // set starting time when note pressed
                    // send key on?
                    notes[total_idx].start_time = elapsedMicros();
                }
                else // send key off?
                notes[total_idx].push_state = key_state;
            }
        }
    }
}

void setup() {
    // setup hardware
    AudioMemory(500);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);
    analogReadResolution(a_read_res);  // also here, 12 bit would mean 0 - 4096 value range in analog read
    // initialize keys
    init_key_notes();

    // initialize mcps
    mcp_keys.begin(MCP_KEYS_ADDRESS);
    for (int out_idx=8; out_idx<=15; out_idx++) {
        mcp_keys.pinMode(out_idx, OUTPUT);
        mcp_keys.digitalWrite(out_idx, LOW);
    }
    for (int in_idx=0; in_idx<=7; in_idx++) {
        mcp_keys.pinMode(in_idx, INPUT);
    }
}

void loop() {
  keybed_read();
}