/*!
    @link https://github.com/cyberponk/PWM_Signal_Analyzer_for_Arduino/blob/master/PWM_Signal_Analyzer_for_Arduino.ino
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TaskScheduler.h>
#include "limits.h"
#include <ModbusRTU.h>

#define _TASK_TIMECRITICAL

constexpr uint32_t BAUDRATE = 9600;
constexpr uint32_t PWMfreq = 25000;
constexpr uint8_t rpmPin = D1; //D1
constexpr uint8_t pwmPin = D3;
constexpr uint8_t onewireBUS = D4;
constexpr uint8_t resolution = 9;
constexpr unsigned long numRegs = 12;
constexpr uint8_t slaveID = 1;
constexpr uint16_t RPMcalcPeriodMS = 400;

volatile uint8_t halfRevs = 0;
uint8_t lowRPMoverflow = 0;
unsigned long lastMillis = 0;
float rpm = 0;
uint8_t pwm = 255;

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
DeviceAddress tempAddr;

ModbusRTU modbus;
Scheduler ts;
Task calculateRPM(RPMcalcPeriodMS, TASK_FOREVER, &calculateRPM_callback);
Task measureTemp(750, TASK_FOREVER, &measureTemp_callback);
OneWire oneWire(onewireBUS);
DallasTemperature ds(&oneWire);

void setup() {
    Serial.begin(9600);
    modbus.begin(&Serial);
    modbus.slave(slaveID);
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
    //ts.addTask(measureTemp);
    ts.enableAll();

    //ds.begin();
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

    modbus.task();
    yield();
}

IRAM_ATTR void rpmISR() {
    halfRevs++;
}

void measureTemp_callback() {
    modbus.Hreg(6, (int)ds.getTempC(tempAddr) * 1000);
    ds.requestTemperatures();
    //modbus.Hreg(7, measureTemp.getOverrun());
}

void calculateRPM_callback() {
    unsigned long currentTime = millis();
    uint16_t deltaT = currentTime - lastMillis; // overflow handled implicitly @link https://www.gammon.com.au/millis

    lastMillis = currentTime;
    modbus.Hreg(5, deltaT);
    modbus.Hreg(4, halfRevs);
    //modbus.Hreg(8, calculateRPM.getStartDelay());

    //cli();
    // if (halfRevs == 0) {
    //     lowRPMoverflow++;
    //     return;
    // }
    rpm = (halfRevs / ((float)(deltaT /* (1 + lowRPMoverflow)*/) / 1000.0)) * 60.0 / 4;
    halfRevs = 0;
    //lowRPMoverflow = 0;
    //sei();
    modbus.Hreg(1, RPMfilter.addMeasurement(rpm));
    modbus.Hreg(2, rpm);
}