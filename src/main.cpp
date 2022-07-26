#define DECODE_NEC // Includes Apple and Onkyo

#include "DFRobot_HuskyLens.h"
#include "arduino-timer.h"
#include <Arduino.h>
#include <HCSR04.h>
#include <IRremote.hpp>
#include <Servo.h>
#include <TaskScheduler.h>
#define IR_INPUT_PIN 8
#include "TinyIRReceiver.hpp"

const int StatusRedPin = 10, StatusYellowPin = 11, StatusGreenPin = 9;
// const int StatusLEDBrightness = 255;

enum class ClassifierID
{
    Empty = 1,
    Left = 2,
    Right = 3
};
enum class IRButtonCode : uint8_t
{
    Power = 0x44,
    Left = 0x07,
    Right = 0x09,
    Down = 0x19,
    OK = 0x15,
    Refresh = 0x43
};

Servo TiltServo;
UltraSonicDistanceSensor DistanceSensor(2, 3);
DFRobot_HuskyLens VisionSensor;
float Distance;
bool IsProcessing = false, IsLearning = false, IsBinFull = false;
const int TiltNeutral = 128, TiltLeft = 64, TiltRight = 96, TiltLength = 3000, TiltCooldown = 1000;
const int TiltOverallLength = TiltLength + TiltCooldown;
int CurrentTilting = 0;
auto TiltTimer = timer_create_default();
bool IsTilted, IsInCooldown;

ClassifierID LearningID = ClassifierID::Empty;
volatile struct TinyIRReceiverCallbackDataStruct IRCallbackData;

Scheduler TaskScheduler;
void IRReceive();
Task IRReceiveTask(50, TASK_FOREVER, &IRReceive, &TaskScheduler, true);
void UpdateStatusLED();
Task UpdateStatusTask(100, TASK_FOREVER, &UpdateStatusLED, &TaskScheduler, true);
void UpdateDistance();
Task UpdateDistanceTask(50, TASK_FOREVER, &UpdateDistance, &TaskScheduler, true);
void LearnProcess();
Task LearnProcessTask(100, TASK_FOREVER, &LearnProcess, &TaskScheduler, true);
void Process();
Task ProcessTask(50, TASK_FOREVER, &Process, &TaskScheduler, true);

void WriteNames();

void setup()
{
    Serial.begin(9600);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ __TIME__ "\r\nUsing library version " VERSION_IRREMOTE));

    TiltServo.attach(12);
    pinMode(StatusRedPin, OUTPUT);
    pinMode(StatusYellowPin, OUTPUT);
    pinMode(StatusGreenPin, OUTPUT);
    Serial.println("Setting up HuskyLens...");
    VisionSensor.beginI2CUntilSuccess();
    WriteNames();
    Serial.println("Succeeded");
    VisionSensor.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
    initPCIInterruptForTinyReceiver();
    Serial.println("Ready to receive NEC IR signals");
    TaskScheduler.startNow();
}
void WriteNames() {
    VisionSensor.writeName("Left", (int)ClassifierID::Left);
    VisionSensor.writeName("Right", (int)ClassifierID::Right);
    VisionSensor.writeName("Empty", (int)ClassifierID::Empty);
}
void FinishLearning() {
    IsLearning = false;
    WriteNames();
}
bool ResetCooldown(void * = nullptr)
{
    Serial.println("Cooldown reset");
    IsInCooldown = false;
    return true;
}
bool ResetTiltWithoutCooldown(void * = nullptr)
{
    TiltTimer.cancel();
    TiltServo.write(TiltNeutral);
    IsTilted = false;
    IsInCooldown = false;
    return true;
}
bool ResetTilt(void * = nullptr)
{
    TiltServo.write(TiltNeutral);
    IsTilted = false;
    IsInCooldown = true;
    TiltTimer.in(TiltCooldown, ResetCooldown);
    return true;
}
void TiltTo(int value)
{
    if (IsTilted || IsInCooldown)
        return;
    TiltServo.write(value);
    IsTilted = true;
    IsInCooldown = false;
    TiltTimer.in(TiltLength, ResetTilt);
}
void IRReceive()
{
    if (IRCallbackData.justWritten && !IRCallbackData.isRepeat)
    {
        IRCallbackData.justWritten = false;
        Serial.print("Address=0x");
        Serial.print(IRCallbackData.Address, HEX);
        Serial.print(" Command=0x");
        Serial.print(IRCallbackData.Command, HEX);
        Serial.println();

        auto buttonCode = (IRButtonCode)IRCallbackData.Command;
        if (buttonCode == IRButtonCode::Power)
        {
            IsProcessing = !IsProcessing;
            if (!IsProcessing)
                ResetTiltWithoutCooldown();
            if (IsProcessing && IsLearning)
                FinishLearning();
        }
        else if (buttonCode == IRButtonCode::Left || buttonCode == IRButtonCode::Right ||
                 buttonCode == IRButtonCode::Down)
        {
            IsLearning = true;
            IsProcessing = false;
            LearningID = buttonCode == IRButtonCode::Down   ? ClassifierID::Empty
                         : buttonCode == IRButtonCode::Left ? ClassifierID::Left
                                                            : ClassifierID::Right;
            ResetTiltWithoutCooldown();
        }
        else if (buttonCode == IRButtonCode::OK && IsLearning)
        {
            FinishLearning();
        }
        else if (buttonCode == IRButtonCode::Refresh)
        {
            VisionSensor.forgetLearn();
            ResetTiltWithoutCooldown();
        }
    }
}

void UpdateStatusLED()
{
    digitalWrite(StatusGreenPin, IsProcessing ? HIGH : LOW);
    digitalWrite(StatusYellowPin, IsLearning ? HIGH : LOW);
    digitalWrite(StatusRedPin, !IsProcessing || IsBinFull ? HIGH : LOW);
}
void UpdateDistance()
{
    if (!IsProcessing)
        return;
    Distance = DistanceSensor.measureDistanceCm();
    // Serial.println(Distance);
}
void LearnProcess()
{
    if (!IsLearning)
        return;
    if (!VisionSensor.learnOnece((int)LearningID))
    {
        Serial.print("Learning not successful on ");
        Serial.println((int)LearningID);
    }
}
void Process()
{
    if (!IsProcessing)
        return;
    if (Distance < 20)
    {
        VisionSensor.request();
        if (VisionSensor.isAppear((int)ClassifierID::Left, HUSKYLENSResultBlock))
        {
            TiltTo(TiltLeft);
        }
        else if (VisionSensor.isAppear((int)ClassifierID::Right, HUSKYLENSResultBlock))
        {
            TiltTo(TiltRight);
        }
    }
}

void loop()
{
    TaskScheduler.execute();
    TiltTimer.tick();
}

void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat)
{
    IRCallbackData.Address = aAddress;
    IRCallbackData.Command = aCommand;
    IRCallbackData.isRepeat = isRepeat;
    IRCallbackData.justWritten = true;
}