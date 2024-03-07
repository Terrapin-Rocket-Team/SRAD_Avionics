

#ifndef FEEDBACK_H
#define FEEDBACK_H

enum PIN_TO_WRITE {
    #ifdef LED_BUILTIN
    LED_BUILTIN = LED_BUILTIN,
    #else
    LED_BUILTIN = 13,
    #endif
    BUZZER = 33,
};

class Feedback {
    PIN_TO_WRITE BUZZER_PIN;
};

#endif