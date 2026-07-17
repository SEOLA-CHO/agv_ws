#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include <stdbool.h>

typedef enum {
    RELAY_ON = 0,
    RELAY_STOPPING,
    RELAY_OFF
} relay_state_t;

void relay_control_init(void);
void relay_set_enabled(bool enabled);
bool relay_is_enabled(void);
bool relay_motion_allowed(void);
bool relay_is_stopping(void);
void relay_note_zero_command_cycle(void);

#endif /* RELAY_CONTROL_H */
