// Example use of ESP_EEPROM library for ESP8266
//
// Normally writing to the 'emulated' EEPROM on ESP8266 requires an erase of the flash page used to hold
// the EEPROM data followed by a re-write of the changed data.
// The erasure takes a significant amount of time (10s of ms) and during this time
// interrupts must be blocked.
// In some cases this interferes with the sketch operation (e.g. it produces a noticeable
// blackout/flash for any PWM controlled lights as ESP8266 PWM relies on interrupts)
//
// The ESP_EEPROM library writes each new version of the EEPROM data to a new area until flash
// is full and so avoids wiping flash until necessary
//
// It's best for use when there are only a few things to save in EEPROM
// (i.e. total the size of the saved info is much smaller than the available flash size)
//

#include <ESP_EEPROM.h>

const int BLUE_LED_PIN = 2;

// The neatest way to access variables stored in EEPROM is using a structure
#pragma pack(push, 1)
struct EEPROM_struct {
    bool fan_psu;   //1 byte
    bool USB_en;    //1 byte
    float K_p;      //2 bytes
    float K_i;      //2 bytes
    float K_d;      //2 bytes
    float cal_a;    //4 bytes
    float cal_b;    //4 bytes
    float nom_vol;  //4 bytes
    char hname[16] = "fanControl_table"; //16 bytes
    uint16_t t_curve[3][2] = {{25,0}, {30, 500}, {40, 1500}};
} eepromVar, eepromVar2;
#pragma pack(pop)

void setup() {
  // Remember to set your serial monitor to 74880 baud
  // This odd speed will show ESP8266 boot diagnostics too
  Serial.begin(9600);
  Serial.println();

  // Set up the initial (default) values for what is to be stored in EEPROM
  eepromVar.fan_psu = false;
  eepromVar.USB_en = true;
  eepromVar.K_p = 1.0;
  eepromVar.K_i = 1.0;
  eepromVar.K_d = 0.1;
  eepromVar.cal_a = 4.571274;
  eepromVar.cal_b = - 22.811258;
  eepromVar.nom_vol = 12.0;
//  eepromVar.hname = "fanControl_table";
//   eepromVar.t_curve = {{25,0}, {30, 500}, {40, 1500}};
//   eepromVar.t_curve[0][0] = 25;
//   eepromVar.t_curve[0][1] = 0;
//   eepromVar.t_curve[1][0] = 30;
//   eepromVar.t_curve[1][1] = 500;
//   eepromVar.t_curve[2][0] = 40;
//   eepromVar.t_curve[2][1] = 1500;

  // All the library functions are accessed via the EEPROM object created when
  // you include the library header ESP_EEPROM.h

  // The library needs to know what size you need for your EEPROM variables
  // Using a structure makes this easy.

  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(EEPROM_struct));

  // Check if the EEPROM contains valid data from another run
  // If so, overwrite the 'default' values set up in our struct
  /*if(EEPROM.percentUsed()>=0) {
    EEPROM.get(0, eepromVar1);
    eepromVar1.anInteger++;     // make a change to our copy of the EEPROM data
    Serial.println("EEPROM has data from a previous run.");
    Serial.print(EEPROM.percentUsed());
    Serial.println("% of ESP flash space currently used");
  } else {
    Serial.println("EEPROM size changed - EEPROM data zeroed - commit() to make permanent");
  }*/

  //
  // (some code that might change the EEPROM data)
  //

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromVar);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "Commit OK" : "Commit failed");

  // Get EEPROM data into our local copy
  // For this example, a different struct variable is used 
  EEPROM.get(0, eepromVar2);

  Serial.print("EEPROM data read, fan_PSU - ");
  Serial.println(eepromVar.fan_psu);
  Serial.print("EEPROM data read, USB_en - ");
  Serial.println(eepromVar.USB_en);
  Serial.print("EEPROM data read, K_p - ");
  Serial.println(eepromVar.K_p, 3);
  Serial.print("EEPROM data read, K_i - ");
  Serial.println(eepromVar.K_i, 3);
  Serial.print("EEPROM data read, K_d - ");
  Serial.println(eepromVar.K_d, 3);
  Serial.print("EEPROM data read, cal_a - ");
  Serial.println(eepromVar.cal_a, 6);
  Serial.print("EEPROM data read, cal_b - ");
  Serial.println(eepromVar.cal_b, 6);
  Serial.print("EEPROM data read, nom_vol - ");
  Serial.println(eepromVar.nom_vol, 2);
  Serial.print("EEPROM data read, hostname - ");
  for (int i = 0; i < hostname_length; i++)
      Serial.print(eepromVar.hname[i]);
  Serial.println("");
  Serial.print("EEPROM data read, t_curve - ");
  for(int i = 0; i < 3; i++) {
      Serial.print("[");
          for(int j = 0; j < 2; j++) {
              Serial.print(eepromVar.t_curve[i][j]);
              if (j==0) Serial.print(",");
          }
      Serial.print("]");
  }
  Serial.println("");
}


void loop() {
  // do nothing
}
