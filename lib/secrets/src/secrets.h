#ifndef SECRETS
#define SECRETS
#include <Arduino.h>
//#define USE_SAMPLE

struct wifiList {
    const char* ssid;
    const char* psk;
};

extern const struct wifiList APlist[];
extern const size_t APlen;
extern const char* pwHash;

#endif