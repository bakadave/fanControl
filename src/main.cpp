/*!
    @link https://github.com/cyberponk/PWM_Signal_Analyzer_for_Arduino/blob/master/PWM_Signal_Analyzer_for_Arduino.ino
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TaskScheduler.h>
#include <SimpleModbusSlave.h>
#include "limits.h"

constexpr long BAUDRATE = 9600;
constexpr uint8_t rpmPin = 2;
constexpr uint8_t pwmPin = 3;
constexpr uint8_t onewireBUS = 4;
constexpr uint8_t resolution = 12;
constexpr unsigned long numRegs = 12;
constexpr uint8_t slaveID = 1;
constexpr uint16_t RPMcalcPeriodMS = 400;

volatile uint8_t halfRevs = 0;
uint8_t lowRPMoverflow = 0;
unsigned long lastMillis = 0;
float rpm = 0;
uint8_t pwm = 79;

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

void pwm25kHzBegin();
void timer1_setup();
void rpmISR();
void calculateRPM_callback();
void measureTemp_callback();
DeviceAddress tempAddr;

//MODBUS setup
SimpleModbusSlave modbus(slaveID);
uint16_t regs[numRegs];

Scheduler ts;
Task calculateRPM(RPMcalcPeriodMS, TASK_FOREVER, &calculateRPM_callback);
Task measureTemp(700, TASK_FOREVER, &measureTemp_callback);
OneWire oneWire(onewireBUS);
DallasTemperature ds(&oneWire);
void pwmDuty(byte ocrb);

void setup() {
    //Serial.begin(BAUDRATE);
    modbus.setup(BAUDRATE);

    pinMode(rpmPin, INPUT);
    pinMode(pwmPin, OUTPUT);
    //digitalWrite(pwmPin, pwm);
    pwm25kHzBegin();
	pwmDuty(pwm);
    regs[3] = pwm;

    attachInterrupt(digitalPinToInterrupt(rpmPin), rpmISR, CHANGE);

    ts.init();
    ts.addTask(calculateRPM);
    ts.enableAll();

    ds.begin();
    regs[6] = -12700;
    if (ds.getDeviceCount() == 1) {
        ds.getAddress(tempAddr, 0);
        ds.setResolution(tempAddr, resolution);
        ds.requestTemperaturesByAddress(tempAddr);
    }
}

void loop() {
    ts.execute();

    pwm = regs[3];
    pwmDuty(pwm);
    modbus.loop(regs, sizeof(regs) / sizeof(regs[0]));
}

void rpmISR() {
    halfRevs++;
}

void measureTemp_callback() {
    regs[6] = ds.getTempC(tempAddr) * 100;
    ds.requestTemperaturesByAddress(tempAddr);
}

void calculateRPM_callback() {
    unsigned long currentTime = millis();
    uint16_t deltaT = currentTime - lastMillis; // overflow handled implicitly @link https://www.gammon.com.au/millis

    lastMillis = currentTime;
    regs[5] = deltaT;
    regs[4] = halfRevs;

    //cli();
    if (halfRevs == 0) {
        lowRPMoverflow++;
        return;
    }
    rpm = (halfRevs / ((float)(deltaT * (1 + lowRPMoverflow)) / 1000.0)) * 60.0 / 4;
    halfRevs = 0;
    lowRPMoverflow = 0;
    //sei();
    regs[1] = RPMfilter.addMeasurement(rpm);
    regs[2] = rpm;
}

#if defined (__AVR__)

void timer1_setup() {
    // Setup Timer1 for precise timing
	TCCR1A = 0; // No comparator, no Waveform generator
	TCCR1B = (1 << CS10); // clk/1 prescaler for 0.0625us precision
//	TCCR1B = (1 << CS11); // clk/8 prescaler for 0.5us precision
	TIMSK1 = 0; // Disable overflow interrupt
	#ifdef USE_T1_OVERFLOW_COUNT
		TIMSK1 = 1; // Enable overflow interrupt (only needed for low/high pulses > 32768 ms)
	#endif
}

//! @link https://forum.arduino.cc/t/controlling-4pin-fan-with-pwm-25khz/399618/3
//! @note 25 kHz PWM
void pwm25kHzBegin() {
    TCCR2A = 0;                               // TC2 Control Register A
    TCCR2B = 0;                               // TC2 Control Register B
    TIMSK2 = 0;                               // TC2 Interrupt Mask Register
    TIFR2 = 0;                                // TC2 Interrupt Flag Register
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
    TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
    OCR2A = 79;                               // TOP overflow value (Hz) 16Mhz / 8 / (79 + 1) = 25Khz
    OCR2B = 0;
}

//! @note range = 0-79 = 1.25-100%
void pwmDuty(byte ocrb) {
    OCR2B = ocrb;                             // PWM Width (duty)
}

#endif