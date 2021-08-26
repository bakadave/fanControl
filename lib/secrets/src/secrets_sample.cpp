/*! @brief make a copy of this file with the name of "secrets.cpp" and fill in any number of lines to add access points to the project
    alternatively: uncomment the line #define USE_SAMPLE in the secrets.h header file to use this file
*/

#ifdef USE_SAMPLE
#include "secrets.h"

struct wifiList APlist[] = {
    {.ssid = "SSID_1", .psk = "password_1"},
    {.ssid = "SSID_2", .psk = "password_2"}
    // etc.
};

const size_t APlen = sizeof(APlist)/sizeof(APlist[0]);  // length of array calculated for easier handling
const char* pwHash = "OTA_password_MD5_hash";           // Arduino OTA password hash for safe storage

#endif