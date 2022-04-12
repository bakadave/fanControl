/*!
    @link https://github.com/cyberponk/PWM_Signal_Analyzer_for_Arduino/blob/master/PWM_Signal_Analyzer_for_Arduino.ino

    @note to enable ArduinOTA through Windows Firewall, temporarily enable "Display a notification to the user when a program is blocked from receiving inbound connections." and allow there.
    Trying to chase down the python executable is messy otherwise.
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

/*! @brief how to receive variables from platformio.ini
    @link https://community.platformio.org/t/setting-variables-in-code-using-build-flags/17167/5
    @link ESP-12E pinout https://github.com/r00tGER/NodeMCU-ESP12E-pinouts
*/
#define ST(A) #A
#define STR(A) ST(A)

#define LED_ON 0x00
#define LED_OFF 0x01

constexpr uint32_t BAUDRATE = 9600;
constexpr uint32_t PWMfreq = 25000;
constexpr uint8_t rpmPin = D6;      //GPIO12
constexpr uint8_t pwmPin = D7;      //GPIO13
constexpr uint8_t onewireBUS = D2;  //GPIO4
constexpr uint8_t PowEn = D5;       //GPIO14
constexpr uint8_t USBpin = D1;      //GPIO5
constexpr uint8_t Vmon = A0;        //ADC
constexpr uint8_t resolution = 9;
constexpr unsigned long numRegs = 12;
constexpr uint8_t slaveID = 1;
constexpr uint16_t RPMcalcPeriodMS = 400;
constexpr char mDNS_name[] = STR(MDNS_NAME);
const uint16_t port = atoi(STR(PORT));

volatile uint8_t halfRevs = 0;
uint16_t pwm = 0;
uint16_t setPoint_RPM = 0;
DeviceAddress tempAddr;

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
};

//! @link https://forum.arduino.cc/t/pid-self-balancing-robot/626526
struct PIDcontroller {
    private:
    unsigned long previousTime = 0;
    uint16_t *setPoint;
    int lastError, cumError;

    public:
    float Kp = 1.0;
    float Ki = 1.00;
    float Kd = 0.1;

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
};

IRAM_ATTR void rpmISR();
void calculateRPM_callback();
void measureTemp_callback();
void measureVoltages_callback();
void USBsense_callback();
bool setup_wifi(const wifiList* APlist, const size_t len);
void setup_OTA();

ESP8266WiFiMulti wifiMulti;
ModbusIP modbus;
Scheduler ts;
struct Kalman RPMfilter;
struct PIDcontroller pid;
Task calculateRPM(RPMcalcPeriodMS, TASK_FOREVER, &calculateRPM_callback);
Task measureTemp(5000, TASK_FOREVER, &measureTemp_callback);
Task measureVoltages(500, TASK_FOREVER, &measureVoltages_callback);
Task USBsense(500, TASK_FOREVER, &USBsense_callback);
OneWire oneWire(onewireBUS);
DallasTemperature ds(&oneWire);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_ON);
    pinMode(rpmPin, INPUT);
    pinMode(pwmPin, OUTPUT);
    pinMode(Vmon, INPUT);
    pinMode(USBpin, INPUT);
    analogWriteFreq(PWMfreq);
    analogWriteResolution(10);  /*! @link https://github.com/esp8266/Arduino/pull/7456 */
    analogWrite(pwmPin, pwm);
    attachInterrupt(digitalPinToInterrupt(rpmPin), rpmISR, CHANGE);

    Serial.begin(9600);

    setup_wifi(APlist, APlen);
    setup_OTA();
    MDNS.begin(mDNS_name);
    digitalWrite(LED_BUILTIN, LED_OFF);

    modbus.server();
    for (size_t i = 0; i < numRegs; i++)
        modbus.addHreg(i);
    modbus.Hreg(3, pwm);

    //init PID controller for fan RPM
    pid.init(&setPoint_RPM);
    modbus.Hreg(0, setPoint_RPM);

    ts.init();
    ts.addTask(calculateRPM);
    //ts.addTask(measureVoltages);
    ts.addTask(USBsense);
    ts.enableAll();

    modbus.Hreg(5, pid.Kp * 1000);
    modbus.Hreg(6, pid.Ki * 1000);
    modbus.Hreg(7, pid.Kd * 1000);
    modbus.Hreg(9, 2);

    ds.begin();
    if (ds.getDeviceCount() == 1) {
        ds.getAddress(tempAddr, 0);
        ds.setResolution(tempAddr, resolution);
        ds.requestTemperaturesByAddress(tempAddr);
        ts.addTask(measureTemp);
    }

}

void loop() {
    ts.execute();

    wifiMulti.run();
    ArduinoOTA.handle();

    modbus.task();
    setPoint_RPM = modbus.Hreg(0);
    //pwm = modbus.Hreg(3);
    analogWrite(pwmPin, pwm);
    yield();
}

IRAM_ATTR void rpmISR() {
    halfRevs++;
}

void measureTemp_callback() {
    modbus.Hreg(4, (float)(ds.getTempC(tempAddr) * 1000));
    ds.requestTemperatures();
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

    modbus.Hreg(8, pid.compute(rpm, deltaT));
    pwm = pid.compute(rpm, deltaT);
    modbus.Hreg(3, pwm);
    Serial.printf("Measured RPM: %u Filtered RPM: %u\r\n", (uint16_t)rpm, modbus.Hreg(1));
}

void measureVoltages_callback() {
    Serial.printf("Measured voltage: %f\r\n", 0.0);
    return;
}

void USBsense_callback() {
    int val = digitalRead(USBpin);
    modbus.Hreg(9, val);
    return;
}

bool setup_wifi(const wifiList* APs, const size_t len) {
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

    return true;
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