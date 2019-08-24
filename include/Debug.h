#define GREENFLIGHT_DEBUG_H

#ifdef GREENFLIGHT_DEBUG_H
    #define DEBUGL(x) Serial.println(x)
    #define DEBUG(x) Serial.print(x)
#else
    #define DEBUGL(x)
    #define DEBUG(x)
#endif
