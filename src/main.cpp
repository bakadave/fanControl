/*!
    @link https://github.com/cyberponk/PWM_Signal_Analyzer_for_Arduino/blob/master/PWM_Signal_Analyzer_for_Arduino.ino
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
//#include <ModbusRTU.h>
#include <ModbusIP_ESP8266.h>
#include "secrets.h"

/*! @brief how to receive variables from platformio.ini
    @link https://community.platformio.org/t/setting-variables-in-code-using-build-flags/17167/5
*/
#define ST(A) #A
#define STR(A) ST(A)

constexpr uint32_t BAUDRATE = 9600;
constexpr uint32_t PWMfreq = 25000;
constexpr uint8_t rpmPin = D1; //D1
constexpr uint8_t pwmPin = D3;
constexpr uint8_t onewireBUS = D4;
constexpr uint8_t ledPin = 2;
constexpr uint8_t resolution = 9;
constexpr unsigned long numRegs = 12;
constexpr uint8_t slaveID = 1;
constexpr uint16_t RPMcalcPeriodMS = 400;
constexpr char mDNS_name[] = STR(MDNS_NAME);
const uint16_t port = atoi(STR(PORT));

volatile uint8_t halfRevs = 0;
uint8_t lowRPMoverflow = 0;
uint8_t pwm = 255;
DeviceAddress tempAddr;

struct Kalman {

    private:
    float Pc = 0.0;
    float G = 0.0;
    float P = 1.0;
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

struct PIDcontroller {
    private:
    unsigned long previousTime = 0;
    uint16_t *setPoint;
    int lastError, cumError;

    public:
    unsigned int Kp = 1;
    unsigned int Ki = 1;
    unsigned int Kd = 1;

    void init (uint16_t _setPoint) {
        setPoint = &_setPoint;
    }

    uint16_t compute(uint16_t input) {
        unsigned long currentTime = millis();

        //handle overlfow
        unsigned int elapsedTime = (previousTime > currentTime) ? (ULONG_MAX - previousTime) + currentTime : currentTime - previousTime;

        int error = *setPoint - input;
        cumError += error * elapsedTime;
        int rateError = (error - lastError) / elapsedTime;

        uint16_t output = Kp * error + Ki + cumError + Kd * rateError;

        lastError = error;
        previousTime = currentTime;

        return output;
    }
} pid;

IRAM_ATTR void rpmISR();
void calculateRPM_callback();
void measureTemp_callback();
void measureVoltages_callback();
bool setup_wifi(const wifiList* APlist, const size_t len);
void setup_OTA();

ESP8266WiFiMulti wifiMulti;
ModbusIP modbus;
//ModbusRTU modbus;
Scheduler ts;
Task calculateRPM(RPMcalcPeriodMS, TASK_FOREVER, &calculateRPM_callback);
Task measureTemp(750, TASK_FOREVER, &measureTemp_callback);
Task measureVoltages(500, TASK_FOREVER, &measureVoltages_callback);
OneWire oneWire(onewireBUS);
DallasTemperature ds(&oneWire);

void setup() {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

    Serial.begin(9600);

    setup_wifi(APlist, APlen);
    setup_OTA();
    MDNS.begin(mDNS_name);
    digitalWrite(ledPin, HIGH);

    modbus.server();
    for (size_t i = 0; i < numRegs; i++)
        modbus.addHreg(i);

    pinMode(rpmPin, INPUT);
    pinMode(pwmPin, OUTPUT);
    analogWriteFreq(PWMfreq);
    analogWrite(pwmPin, pwm);
    modbus.Hreg(3, pwm);

    attachInterrupt(digitalPinToInterrupt(rpmPin), rpmISR, CHANGE);

    ts.init();
    ts.addTask(calculateRPM);
    ts.addTask(measureTemp);
    //ts.addTask(measureVoltages);
    ts.enableAll();

    ds.begin();
    modbus.Hreg(6, 1000);
    if (ds.getDeviceCount() == 1) {
        ds.getAddress(tempAddr, 0);
        ds.setResolution(tempAddr, resolution);
        ds.requestTemperaturesByAddress(tempAddr);
    }

}

void loop() {
    ts.execute();

    pwm = modbus.Hreg(3);
    analogWrite(pwmPin, pwm);

    //! enable for continuous reconnect if wifi is lost
    wifiMulti.run();
    ArduinoOTA.handle();

    modbus.Hreg(0, WiFi.status());
    modbus.task();
    yield();
}

IRAM_ATTR void rpmISR() {
    halfRevs++;
}

void measureTemp_callback() {
    modbus.Hreg(6, (int)ds.getTempC(tempAddr) * 1000);
    ds.requestTemperatures();
    Serial.printf("Measured temperature: %f C\r\n", modbus.Hreg(6));
}

void calculateRPM_callback() {
    float rpm;
    uint16_t deltaT = RPMcalcPeriodMS + calculateRPM.getStartDelay();
    modbus.Hreg(5, deltaT);
    modbus.Hreg(4, halfRevs);

    rpm = (halfRevs / ((float)(deltaT /* (1 + lowRPMoverflow)*/) / 1000.0)) * 60.0 / 4;
    halfRevs = 0;

    modbus.Hreg(1, RPMfilter.addMeasurement(rpm));
    modbus.Hreg(2, rpm);
    Serial.printf("Measured RPM: %u Filtered RPM: %u\r\n", rpm, modbus.Hreg(1));
}

void measureVoltages_callback() {
    Serial.printf("Measured voltage: %f\r\n", 0.0);
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