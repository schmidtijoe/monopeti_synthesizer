#pragma once

#include "wiring.h"

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
    Note();

    void setNoteValues(byte no_val, byte vel_val);

    // We declare this method "const" because it doesn't "change" the note object. It
    // just returns information.
    bool isPressed() const;
    void setPush(bool state);

    unsigned int getTime() const;
    byte get_note() const;
    byte get_velocity() const;

private:
    bool push_state;
    byte note_value;
    byte velocity_value;
    unsigned int last_press_timer;
};