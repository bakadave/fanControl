/*!
*   @link https://github.com/cyberponk/PWM_Signal_Analyzer_for_Arduino/blob/master/PWM_Signal_Analyzer_for_Arduino.ino
*
*   @note to enable ArduinOTA through Windows Firewall, temporarily enable "Display a notification to the user when a program is blocked from receiving inbound connections." and allow there.
*   Trying to chase down the python executable is messy otherwise.
*
*
*   @link ESP-12E pinout https://github.com/r00tGER/NodeMCU-ESP12E-pinouts
*/

#define _TASK_TIMECRITICAL

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TaskScheduler.h>
#include <ModbusIP_ESP8266.h>
#include "secrets.h"
#include <ESP_EEPROM.h>
#include <ArduinoJson.h>

/*! @brief how to receive variables from platformio.ini
*   @link https://community.platformio.org/t/setting-variables-in-code-using-build-flags/17167/5
*/
#define ST(A) #A
#define STR(A) ST(A)

#define LED_ON 0x00
#define LED_OFF 0x01

constexpr uint32_t BAUDRATE = 9600;
constexpr uint32_t PWMfreq = 25000;
constexpr uint8_t rpmPin = 13;      //GPIO13
constexpr uint8_t pwmPin = 14;      //GPIO14
constexpr uint8_t onewireBUS = 4;   //GPIO4
constexpr uint8_t PowEn = 12;       //GPIO12
constexpr uint8_t USBpin = 5;       //GPIO5
constexpr uint8_t Vmon = A0;        //ADC
constexpr uint8_t resolution = 9;
constexpr unsigned long numRegs = 12;
constexpr uint8_t slaveID = 1;
constexpr uint16_t RPMcalcPeriodMS = 400;
const uint16_t port = atoi(STR(PORT));
const char default_hostname[] = STR(MDNS_NAME);
constexpr size_t hostname_length = 16;

String inputSerialString;
bool serialReady = false;
bool serialStreamStarted = false;
static unsigned long lastRefreshTime_Serial = 0;
uint16_t buff_size = 512;
constexpr uint16_t serial_timeout_ms = 500;

volatile uint8_t halfRevs = 0;
uint16_t pwm = 0;
uint16_t setPoint_RPM = 0;
uint16_t voltage = 0;
char mDNS_name[hostname_length];
bool USBstate = false;
bool voltageEnable = false;
bool safeMode = true;
float temperature = 0;
DeviceAddress tempAddr;

struct EEPROM_struct {
    bool fan_psu = true;   //1 byte
    bool USB_en = false;    //1 byte
    float K_p = 1.0;    //2 bytes
    float K_i = 1.0;    //2 bytes
    float K_d = 0.1;    //2 bytes
    float cal_a = 1.0;    //4 bytes
    float cal_b = 0.0;    //4 bytes
    float nom_vol;  //4 bytes
    char hname[hostname_length]; //16 bytes
    uint16_t t_curve[3][2];
} eepromVar;

struct Kalman {
    private:
    float Pc = 0.0;
    float G = 0.0;
    float P = 1.3;
    float Xp = 0.0;
    float Zp = 0.0;
    float Xe = 0.0;

    public:
    float varMeas = 1;  // variance determined using excel and reading samples of raw sensor data
    float varProcess = 1e-2;

    float addMeasurement(float meas) {
        Pc = P + varProcess;
        G = Pc/(Pc + varMeas);    // kalman gain
        P = (1-G)*Pc;
        Xp = Xe;
        Zp = Xp;
        Xe = G*(meas-Zp)+Xp;   // the kalman estimate of the sensor voltage

        return Xe;
    }
} RPMfilter;

//! @link https://forum.arduino.cc/t/pid-self-balancing-robot/626526
struct PIDcontroller {
    private:
    unsigned long previousTime = 0;
    uint16_t *setPoint;
    int lastError, cumError;

    public:
    float Kp;
    float Ki;
    float Kd;

    void init (uint16_t* _setPoint) {
        setPoint = _setPoint;
    }

    uint16_t compute(uint16_t input, unsigned int elapsedTime) {
        int error = *setPoint - input;
        cumError += error * ((float)elapsedTime/1000);
        float rateError = (error - lastError) / ((float)elapsedTime/1000);

        int16_t output = Kp * error + Ki * cumError + Kd * rateError;

        lastError = error;

        output = output < 0 ? 0 : output;
        output = output > 1023 ? 1023 : output;

        if (output == 0 && *setPoint != 0)
            output = 20;

        return output;
    }
} pid;

IRAM_ATTR void rpmISR();
void calculateRPM_callback();
void measureTemp_callback();
void measureVoltages_callback();
void USBsense_callback();
void POWenable_callback();
void setRPM_callback();
void setup_wifi(const wifiList* APlist, const size_t len);
void setup_OTA();
void handleSerial_callback();
void serialWatchdog_callback();
void serialDisable_callback();
void parseSerial();
void printEEPROM();

OneWire oneWire(onewireBUS);
DallasTemperature ds(&oneWire);
ESP8266WiFiMulti wifiMulti;
ModbusIP modbus;
Scheduler ts;
DynamicJsonDocument dataJsonBuffer(buff_size);

Task calculateRPM(RPMcalcPeriodMS, TASK_FOREVER, &calculateRPM_callback);
Task measureTemp(5000, TASK_FOREVER, &measureTemp_callback);
Task measureVoltages(250, TASK_FOREVER, &measureVoltages_callback);
Task USBsense(500, TASK_FOREVER, &USBsense_callback);
Task POWenable(250, TASK_FOREVER, &POWenable_callback);
Task setRPM(1000, TASK_FOREVER, &setRPM_callback);
Task handleSerial(100, TASK_FOREVER, &handleSerial_callback);
Task serialWatchdog(50, TASK_FOREVER, &serialWatchdog_callback);
Task disableSerial(120000L, TASK_ONCE, &serialDisable_callback);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_ON);
    pinMode(rpmPin, INPUT);
    pinMode(Vmon, INPUT);
    pinMode(USBpin, INPUT);
    pinMode(PowEn, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    analogWriteFreq(PWMfreq);
    analogWriteResolution(10);  /*! @link https://github.com/esp8266/Arduino/pull/7456 */
    analogWrite(pwmPin, pwm);
    attachInterrupt(digitalPinToInterrupt(rpmPin), rpmISR, CHANGE);
    inputSerialString.reserve(buff_size);

    Serial.begin(9600);

    EEPROM.begin(sizeof(EEPROM_struct));
    ts.init();

    if (EEPROM.percentUsed() >= 0) {
        EEPROM.get(0, eepromVar);
        printEEPROM();
        memcpy(mDNS_name, eepromVar.hname, hostname_length);

        ts.addTask(calculateRPM);
        ts.addTask(setRPM);
        //ts.addTask(disableSerial);
        if (eepromVar.USB_en)
            ts.addTask(USBsense);
        if (!eepromVar.fan_psu)
            ts.addTask(POWenable);
    }
    else {
        Serial.println("Error EEPROM not initialised. Entering calibration mode");
        memcpy(mDNS_name, default_hostname, hostname_length);
    }
    ts.addTask(measureVoltages);
    ts.addTask(handleSerial);
    ts.addTask(serialWatchdog);
    ts.enableAll();

    setup_wifi(APlist, APlen);
    setup_OTA();
    MDNS.begin(mDNS_name);

    modbus.server();
    for (size_t i = 0; i < numRegs; i++)
        modbus.addHreg(i);
    modbus.Hreg(3, pwm);

    //init PID controller for fan RPM
    pid.init(&setPoint_RPM);
    pid.Kp = eepromVar.K_p;
    pid.Ki = eepromVar.K_i;
    pid.Kd = eepromVar.K_d;
    modbus.Hreg(0, setPoint_RPM);

    modbus.Hreg(5, pid.Kp * 1000);
    modbus.Hreg(6, pid.Ki * 1000);
    modbus.Hreg(7, pid.Kd * 1000);
    modbus.Hreg(9, 2);

    ds.begin();
    if (ds.getDeviceCount() == 1) {
        ds.getAddress(tempAddr, 0);
        ds.setResolution(tempAddr, resolution);
        ds.requestTemperaturesByAddress(tempAddr);
        ds.setWaitForConversion(false); //enable async operation
        ts.addTask(measureTemp);
        measureTemp.enable();
    }
    digitalWrite(LED_BUILTIN, LED_OFF);
}

void loop() {
    ts.execute();

    wifiMulti.run();
    ArduinoOTA.handle();
    modbus.task();
    yield();
}

void handleSerial_callback() {
    while (Serial.available()) {
        char c = Serial.read();

        if (inputSerialString.length() == buff_size) {
            inputSerialString = F("");
            serialReady = false;
            serialStreamStarted = false;
            Serial.println("ERROR: Serial buffer overflow.");
            return;
        }
        if (c == '{') {
            serialStreamStarted = true;
            lastRefreshTime_Serial = millis();
        }

        if (serialStreamStarted) {
            if (c == '}') {
                inputSerialString += c;
                serialReady = true;
                serialStreamStarted = false;
            }
        else
            inputSerialString += c;
        }
    }

    if (serialReady) {
        parseSerial();
        inputSerialString = F("");
        serialReady = false;
    }

    return;
}

void serialWatchdog_callback() {
  if (serialStreamStarted && millis() >= lastRefreshTime_Serial + serial_timeout_ms) {
        Serial.println("Serial timeout");
        inputSerialString = F("");
		serialReady = false;
        serialStreamStarted = false;
    }
}

void serialDisable_callback() {
    handleSerial.disable();
    serialWatchdog.disable();
    return;
}

void parseSerial() {
    dataJsonBuffer.clear();

    DeserializationError err = deserializeJson(dataJsonBuffer, inputSerialString);
    if (err) {
        Serial.println(String(F("DESERIALIZATION failed with code ")) += err.c_str());
        return;
    }

    if (dataJsonBuffer.containsKey("fan_psu")) {
        eepromVar.fan_psu = dataJsonBuffer["fan_psu"];
    }
    if (dataJsonBuffer.containsKey("USB_en")) {
        eepromVar.USB_en = dataJsonBuffer["USB_en"];
    }
    if (dataJsonBuffer.containsKey("K_p")) {
        pid.Kp = dataJsonBuffer["K_p"];
        eepromVar.K_p = pid.Kp;
        modbus.Hreg(5, pid.Kp * 1000);
    }
    if (dataJsonBuffer.containsKey("K_i")) {
        pid.Ki = dataJsonBuffer["K_i"];
        eepromVar.K_i = pid.Ki;
        modbus.Hreg(6, pid.Ki * 1000);
    }
    if (dataJsonBuffer.containsKey("K_d")) {
        pid.Kd = dataJsonBuffer["K_d"];
        eepromVar.K_d = pid.Kd;
        modbus.Hreg(7, pid.Kd * 1000);
    }
    if (dataJsonBuffer.containsKey("cal_a")) {
        eepromVar.cal_a = dataJsonBuffer["cal_a"];
    }
    if (dataJsonBuffer.containsKey("cal_b")) {
        eepromVar.cal_b = dataJsonBuffer["cal_b"];
    }
    if (dataJsonBuffer.containsKey("nom_vol")) {
        eepromVar.nom_vol = dataJsonBuffer["nom_vol"];
    }
    if (dataJsonBuffer.containsKey("hname")) {
        String _hostname = dataJsonBuffer["hname"];
        if (_hostname.length() <= hostname_length && _hostname.length() > 0) {
            _hostname.toCharArray(eepromVar.hname, _hostname.length() + 1);
        }
    }
    if (dataJsonBuffer.containsKey("t_curve")) {
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 2; j++) {
                eepromVar.t_curve[i][j] = dataJsonBuffer["t_curve"][i][j];
            }
        }
    }

    EEPROM.put(0, eepromVar);
    EEPROM.commit();
    EEPROM.get(0, eepromVar);
    printEEPROM();
}

void printEEPROM() {
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
    for (uint16_t i = 0; i < hostname_length; i++)
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
    return;
}

void setRPM_callback() {
    if (eepromVar.fan_psu == false && voltageEnable == false) {
        setPoint_RPM = 0;
        modbus.Hreg(0, setPoint_RPM);
        return;
    }

    if (eepromVar.USB_en == true && USBstate == false) {
        setPoint_RPM = 0;
        modbus.Hreg(0, setPoint_RPM);
        return;
    }

    if (temperature < eepromVar.t_curve[0][0]) {
        setPoint_RPM = eepromVar.t_curve[0][1];
        modbus.Hreg(0, setPoint_RPM);
        return;
    }
    if (temperature < eepromVar.t_curve[1][0]) {
        setPoint_RPM = eepromVar.t_curve[1][1];
        modbus.Hreg(0, setPoint_RPM);
        return;
    }
    if (temperature >= eepromVar.t_curve[1][0] && temperature < eepromVar.t_curve[2][0]) {
        uint16_t lin_scaler = (eepromVar.t_curve[2][1] - eepromVar.t_curve[1][1]) / (eepromVar.t_curve[2][0] - eepromVar.t_curve[1][0]);
        setPoint_RPM = eepromVar.t_curve[1][1] + (temperature - eepromVar.t_curve[1][0]) * lin_scaler;
        modbus.Hreg(0, setPoint_RPM);
        return;
    }
    if (temperature >= eepromVar.t_curve[2][0]) {
        setPoint_RPM = eepromVar.t_curve[2][1];
        modbus.Hreg(0, setPoint_RPM);
        return;
    }
}

IRAM_ATTR void rpmISR() {
    halfRevs++;
}

void measureTemp_callback() {
    temperature = ds.getTempC(tempAddr);
    modbus.Hreg(4, (temperature * 1000.0));
    ds.requestTemperatures();
    return;
}

void calculateRPM_callback() {
    uint8_t _revs = halfRevs;
    halfRevs = 0;
    float rpm;              //TODO shouldn't be float
    uint16_t deltaT = RPMcalcPeriodMS + calculateRPM.getStartDelay();

    rpm = (_revs / ((float)deltaT / 1000.0)) * 60.0 / 4;
    modbus.Hreg(2, rpm);    //raw RPM
    rpm = RPMfilter.addMeasurement(rpm);
    modbus.Hreg(1, rpm);    //filtered RPM

    pid.Kp = (float)modbus.Hreg(6) / 1000;
    pid.Ki = (float)modbus.Hreg(7) / 1000;
    pid.Kd = (float)modbus.Hreg(8) / 1000;

    pwm = pid.compute(rpm, deltaT);
    analogWrite(pwmPin, pwm);
    modbus.Hreg(3, pwm);
}

void measureVoltages_callback() {
    int val = analogRead(Vmon);
    val = eepromVar.cal_a * val + eepromVar.cal_b;
    voltage = val;
    modbus.Hreg(10, voltage);
    return;
}

void USBsense_callback() {
    USBstate = digitalRead(USBpin);
    modbus.Hreg(9, USBstate);
    return;
}

//! @note Check if voltage is whithin 10% of nominal voltage. nom_vol needs to be multipled by 100 because voltage is stored as integer.
void POWenable_callback() {
    if(voltage > (eepromVar.nom_vol * 0.9 * 100.0) && voltage < (eepromVar.nom_vol * 1.1 * 100)) {
        voltageEnable = true;
        digitalWrite(PowEn, voltageEnable);
        modbus.Hreg(8, voltageEnable);
        return;
    }
    voltageEnable = false;
    digitalWrite(PowEn, voltageEnable);
    modbus.Hreg(8, voltageEnable);
    return;
}

void setup_wifi(const wifiList* APs, const size_t len) {
	delay(10);

	//settings stolen from Tasmota, supposed to increase reliablity
	WiFi.persistent(false);
	WiFi.disconnect(true);
	delay(200);
	WiFi.mode(WIFI_STA);

    for(uint8_t i = 0; i < len; i++)
       wifiMulti.addAP(APs[i].ssid, APlist[i].psk);

    //! Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
	Serial.println();
	Serial.print("Adding SSIDs from the list and connecting to the strongest one.");

	while (wifiMulti.run() != WL_CONNECTED)
        delay(500);

	randomSeed(micros());
	WiFi.hostname(mDNS_name);

	Serial.println();
	Serial.println("WiFi connected");
    Serial.print("SSID: ");
    Serial.print(WiFi.SSID());
	Serial.println(" IP address: ");
	Serial.println(WiFi.localIP());

    return;
}

/*! OTA
*   @url: https://community.platformio.org/t/arduino-ota-on-esp8266/11823/3
*/
void setup_OTA() {
	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else
		    type = "filesystem";
		//! @note if updating FS this would be the place to unmount FS using FS.end()
		Serial.println("Start updating " + type);
	});

    ArduinoOTA.setPort(port);
	ArduinoOTA.setPasswordHash(pwHash);

	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
	});

	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR)
		    Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR)
		    Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR)
		    Serial.println("Receive Failed");

		else if (error == OTA_END_ERROR)
		    Serial.println("End Failed");
		});

	ArduinoOTA.begin();
}