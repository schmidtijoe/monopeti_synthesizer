#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include "wiring.h"
#include <Audio.h>

// global values
// We don't use defines anymore, because these are just "text replacements". Instead, we use
// compile-time values that have a type and can be checked properly. The compiler will optimize
// this heavily, so there is no runtime overhead.

constexpr u_int8_t no_of_keys = 64;
constexpr u_int8_t a_read_res = 10;  // bit depth of reading resolution
constexpr byte half_vel = 90;

// global vars
Adafruit_MCP23017 mcp_keys;

int res_range = (int) (2 << a_read_res) - 1;  // analog reading range, dependent on bit depth

// teensy audio system design tool code
AudioControlSGTL5000 sgtl5000_1;

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
            last_press_timer = elapsedMicros();
        }
            /**
             * Otherwise, when the key was released, we reset the timer to 0
             */
        else {
            last_press_timer = 0;
        }
    }

    elapsedMicros getTime() const
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
    elapsedMicros last_press_timer;
};

/**
 * PS: We don't want "new" here, because this means we allocate the memory ourselves. If
 * we allocate it ourselves, then we need to free the memory at some point or it will remain
 * alive when the program crashes. Since we know the number of keys from the beginning, we
 * can declare this array using [..] notation and let the compiler take care of the memory.
 *
 * Also, this already allocates all the note-objects and during initialization, we only want to "change"
 * them to have the right note_val and vel_val, but not create new note-objects.
 */
Note notes[no_of_keys];
static Note NOTE_TO_PLAY;

// initialize
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
                    notes[idx].setNoteValues(56 + in_idx, half_vel);
                    break;
                case 1:
                    notes[idx].setNoteValues(64 + in_idx, half_vel);
                    break;
                case 2:
                    notes[idx].setNoteValues(48 + in_idx, half_vel);
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
                    notes[idx].setNoteValues(72 + in_idx, half_vel);
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
        mcp_keys.digitalWrite(out_idx, HIGH);

        // run through read
        for (int in_idx = 0; in_idx < 8; in_idx++) {
            auto total_idx = (out_idx - 8) * 8 + in_idx;
            auto key_state = static_cast<bool>(mcp_keys.digitalRead(in_idx));
            notes[total_idx].setPush(key_state);
        }
    }
}

void play_note()
{
    // Find the key that was pressed latest. Basically, this finds the key that is pressed which has
    // the minimal timer-value.
    for (const auto &note : notes) {

        if (NOTE_TO_PLAY.get_velocity() == note.get_velocity() && NOTE_TO_PLAY.get_note() == note.get_note()) {
            if (!note.isPressed()) {
                NOTE_TO_PLAY.setPush(false);
            }
        }

        if (note.isPressed() && (!NOTE_TO_PLAY.isPressed() || NOTE_TO_PLAY.getTime() < note.getTime())) {
            NOTE_TO_PLAY = note;
        }
    }

    /** Now play the note. We need to check if the key is pressed because if no note on the keyboard is
     * pressed, then NOTE_TO_PLAY will be the "fake" note of the declaration "Note NOTE_TO_PLAY;"
     */
    if (NOTE_TO_PLAY.isPressed()) {
        const auto note_value = NOTE_TO_PLAY.get_note();
        const auto vel_value = NOTE_TO_PLAY.get_velocity();
        // Now just send this to the audio..
    }
    else {
        // turn all notes off
    }
}

void setup()
{
    // setup hardware
    AudioMemory(500);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);
    analogReadResolution(a_read_res);  // also here, 12 bit would mean 0 - 4096 value range in analog read
    // initialize keys
    init_key_notes();

    // initialize mcps
    mcp_keys.begin();
    for (int out_idx = 8; out_idx <= 15; out_idx++) {
        mcp_keys.pinMode(out_idx, OUTPUT);
        mcp_keys.digitalWrite(out_idx, LOW);
    }
    for (int in_idx = 0; in_idx <= 7; in_idx++) {
        mcp_keys.pinMode(in_idx, INPUT);
    }
}

void loop()
{
    keybed_read();
    play_note();
}