#include "note.h"
#include "wiring.h"


Note::Note()
    : push_state(false), note_value(0), velocity_value(0), last_press_timer(0)
{}

void Note::setNoteValues(byte no_val, byte vel_val)
{
    note_value = no_val;
    velocity_value = vel_val;
}

bool Note::isPressed() const
{
    return push_state;
}

void Note::setPush(bool state)
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

unsigned int Note::getTime() const
{
    return last_press_timer;
}

byte Note::get_note() const
{
    return note_value;
}

byte Note::get_velocity() const
{
    return velocity_value;
}

