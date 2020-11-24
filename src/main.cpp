#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include <elapsedMillis.h>
#include <Audio.h>

// global defs
#define no_of_keys 64
#define a_read_res 10  // bit depth of reading resolution

// global vars
Adafruit_MCP23017 mcp_keys;
byte half_vel = 90;
int res_range = (int)(2 << a_read_res) - 1;  // analod reading range, dependent on bit depth
byte note;
byte velocity;

// teensy audio system design tool code
AudioControlSGTL5000    sgtl5000_1;
// declare Note structure, micros or elapsed micros here?!?!

class Note {
public:
    bool push_state;
    byte note_value;
    byte velocity_value;
    elapsedMicros last_press_timer;

    // constructor
    Note() = default;
    Note(bool state, byte no_val, byte vel_val) {
        push_state = state;
        note_value = no_val;
        velocity_value = vel_val;
    }
};
Note* notes = new Note[no_of_keys];

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
            switch (out_idx) {
                case 0:
                    notes[idx] = Note(false, 56 + in_idx, half_vel);
                    break;
                case 1:
                    notes[idx] = Note(false, 64 + in_idx, half_vel);
                    break;
                case 2:
                    notes[idx] = Note(false, 48 + in_idx, half_vel);
                    break;
                case 3:
                    notes[idx] = Note(false, 48 + in_idx, 127);
                    break;
                case 4:
                    notes[idx] = Note(false, 56 + in_idx, 127);
                    break;
                case 5:
                    notes[idx] = Note(false, 64 + in_idx, 127);
                    break;
                case 6:
                    notes[idx] = Note(false, 72 + in_idx, 127);
                    break;
                case 7:
                    notes[idx] = Note(false, 72 + in_idx, half_vel);
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
                notes[total_idx].push_state = key_state;
                if (key_state) {
                    // set starting time when note pressed
                    note = notes[total_idx].note_value;
                    velocity = notes[total_idx].velocity_value;
                    notes[total_idx].last_press_timer = elapsedMicros();
                }
                else {
                    notes[total_idx].last_press_timer = 0;
                    int note_compare = 0;
                    unsigned int note_compare_timer = 200000;  // basically ignores notes pressed for 200 sek :D
                    for (int search_idx = 0; search_idx < no_of_keys && search_idx != total_idx; search_idx++) {
                        // look for notes still pressed:
                        if (notes[search_idx].push_state) {
                            // find the one with shortest last press duration
                            if (notes[search_idx].last_press_timer < note_compare_timer) {
                                note_compare = notes[search_idx].note_value;
                                note_compare_timer = notes[search_idx].last_press_timer;
                            }
                        }
                    }
                }
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
    mcp_keys.begin();
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