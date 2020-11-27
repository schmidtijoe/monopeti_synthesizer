#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include "wiring.h"
#include <Audio.h>

/**
 * Global variables
 */
// keybed
Adafruit_MCP23017 MCP_KEYS;     // gpio expander handling keyboard
constexpr u_int8_t NO_OF_KEYS = 64;     // total number of keys, i.e. toggle switches

// analog reading
constexpr u_int8_t READ_RES_BIT_DEPTH = 10;  // bit depth of reading resolution
int RES_RANGE = (int) (2 << READ_RES_BIT_DEPTH) - 1;  // analog reading range, dependent on bit depth

// note & midi interfacing
constexpr byte HALF_VEL = 90;
bool TOGGLE_LOOP = false;       // toggles note searching for mono-keyboard use

// teensy audio system design tool code
AudioControlSGTL5000 sgtl5000_1;

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
     * what happens with velocity changes?                          -> no play events. not yet incorporated
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

    // initialize mcps
    MCP_KEYS.begin();
    for (int out_idx = 8; out_idx <= 15; out_idx++) {
        MCP_KEYS.pinMode(out_idx, OUTPUT);
        MCP_KEYS.digitalWrite(out_idx, LOW);
    }
    for (int in_idx = 0; in_idx <= 7; in_idx++) {
        MCP_KEYS.pinMode(in_idx, INPUT);
    }
}

void loop()
{
    keybed_read();
    note_update();
}