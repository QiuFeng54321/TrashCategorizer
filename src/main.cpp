#define DECODE_NEC // Includes Apple and Onkyo

#include "DFRobot_HuskyLens.h"
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
    OK = 0x15
};

Servo TiltServo;
UltraSonicDistanceSensor DistanceSensor(2, 3);
DFRobot_HuskyLens VisionSensor;
float Distance;
bool IsProcessing = false, IsLearning = false, IsBinFull = false;
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

void setup()
{
    Serial.begin(9600);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ __TIME__ "\r\nUsing library version " VERSION_IRREMOTE));

    TiltServo.attach(12);
    pinMode(StatusRedPin, OUTPUT);
    pinMode(StatusYellowPin, OUTPUT);
    pinMode(StatusGreenPin, OUTPUT);
    Serial.println("Beginning HuskyLens...");
    VisionSensor.beginI2CUntilSuccess();
    Serial.println("Succeeded");
    VisionSensor.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
    initPCIInterruptForTinyReceiver();
    Serial.println("Ready to receive NEC IR signals");
    TaskScheduler.startNow();
}

void IRReceive()
{
    if (IRReceiveTask.isFirstIteration()) {
      Serial.println("Hi");
    }
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
            if (IsProcessing && IsLearning)
                IsLearning = false;
        }
        else if (buttonCode == IRButtonCode::Left || buttonCode == IRButtonCode::Right ||
                 buttonCode == IRButtonCode::Down)
        {
            IsLearning = true;
            IsProcessing = false;
            LearningID = buttonCode == IRButtonCode::Down   ? ClassifierID::Empty
                         : buttonCode == IRButtonCode::Left ? ClassifierID::Left
                                                            : ClassifierID::Right;
        }
        else if (buttonCode == IRButtonCode::OK && IsLearning)
        {
            IsLearning = false;
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
    if (!IsProcessing) return;
    Distance = DistanceSensor.measureDistanceCm();
    Serial.println(Distance);
}
void LearnProcess()
{
    if (!IsLearning) return;
    if (!VisionSensor.learnOnece((int)LearningID))
    {
        Serial.print("Learning not successful on ");
        Serial.println((int)LearningID);
    }
}
void Process()
{
    if (!IsProcessing) return;
    if (Distance < 20)
    {
        VisionSensor.request();
        if (VisionSensor.isAppear(1, HUSKYLENSResultBlock))
        {
            TiltServo.write(90);
            delay(3000);
            TiltServo.write(0);
            delay(1000);
        }
    }
}

void loop()
{
    TaskScheduler.execute();
    // delay(100);
    // IRReceive();
    // UpdateStatusLED();
    // if (IsProcessing)
    // {
    //     UpdateDistance();
    //     Process();
    // }
    // else if (IsLearning)
    // {
    //     LearnProcess();
    // }
}

void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat)
{
    IRCallbackData.Address = aAddress;
    IRCallbackData.Command = aCommand;
    IRCallbackData.isRepeat = isRepeat;
    IRCallbackData.justWritten = true;
}