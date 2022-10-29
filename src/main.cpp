
#include "AverageClass.hpp"
#include "Common.hpp"
#include "EEPROMHelper.hpp"

Servo TiltServo;
const int TiltServoPin = 12;
UltraSonicDistanceSensor DistanceSensor(2, 3);
DFRobot_HuskyLens VisionSensor;
const float MaximumDetectionRange = 20;
float Distance;
bool IsProcessing = false, IsLearning = false, IsBinFull = false;
const int TiltNeutral = 85, TiltLeft = 40, TiltRight = 130, TiltLength = 3000,
          TiltCooldown = 1000, TiltWaitLength = 300;
const int TiltOverallLength = TiltLength + TiltCooldown;
int CurrentTilting = 0;
auto TiltTimer = timer_create_default();
bool IsTilted, IsInCooldown, IsWaiting;
int TiltValue = TiltNeutral;

ClassifierType CurrentLearningType = ClassifierType::Empty;
int CurrentLearningID;

volatile struct TinyIRReceiverCallbackDataStruct IRCallbackData;

Scheduler TaskScheduler;
void IRReceive();
Task IRReceiveTask(50, TASK_FOREVER, &IRReceive, &TaskScheduler, true);
void UpdateStatusLED();
Task UpdateStatusTask(100, TASK_FOREVER, &UpdateStatusLED, &TaskScheduler,
                      true);
void UpdateDistance();
Task UpdateDistanceTask(50, TASK_FOREVER, &UpdateDistance, &TaskScheduler,
                        true);
void LearnProcess();
Task LearnProcessTask(100, TASK_FOREVER, &LearnProcess, &TaskScheduler, true);
void Process();
Task ProcessTask(50, TASK_FOREVER, &Process, &TaskScheduler, true);

void WriteNames();
bool ResetTiltWithoutCooldown(void* = nullptr);

void setup() {
    Serial.begin(9600);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ __TIME__
                     "\r\nUsing library version " VERSION_IRREMOTE));
    InitializeEEPROM();
    TiltServo.attach(TiltServoPin);
    pinMode(StatusRedPin, OUTPUT);
    pinMode(StatusYellowPin, OUTPUT);
    pinMode(StatusGreenPin, OUTPUT);
    Serial.println("Setting up HuskyLens...");
    VisionSensor.beginI2CUntilSuccess();
    WriteNames();
    ResetTiltWithoutCooldown();
    Serial.println("Succeeded");
    VisionSensor.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
    initPCIInterruptForTinyReceiver();
    Serial.println("Ready to receive NEC IR signals");
    TaskScheduler.startNow();
}
void WriteNames() {
    for (int i = LeftIDOffset; i < Header.LeftTop; i++) {
        VisionSensor.writeName("L" + i, (int)ClassifierType::Left);
    }
    for (int i = RightIDOffset; i < Header.RightTop; i++) {
        VisionSensor.writeName("R" + i, (int)ClassifierType::Right);
    }
    VisionSensor.writeName("E", EmptyID);
}
void FinishLearning() {
    IsLearning = false;
    WriteNames();
}
bool ResetCooldown(void* = nullptr) {
    Serial.println("Cooldown reset");
    IsInCooldown = false;
    return true;
}
bool ResetTiltWithoutCooldown(void* = nullptr) {
    TiltTimer.cancel();
    TiltServo.write(TiltNeutral);
    IsTilted = false;
    IsInCooldown = false;
    IsWaiting = false;
    return true;
}
bool ResetTilt(void* = nullptr) {
    TiltServo.write(TiltNeutral);
    IsTilted = false;
    IsInCooldown = true;
    IsWaiting = false;
    ResetFeed();
    TiltTimer.in(TiltCooldown, ResetCooldown);
    return true;
}
void TiltTo(int value) {
    TiltServo.write(value);
    IsTilted = true;
    TiltTimer.in(TiltLength, ResetTilt);
}
bool ManageTilt(void* = nullptr) {
    IsWaiting = false;
    IsInCooldown = false;
    VisionSensor.request();
    if (VisionSensor.countBlocks() > 0) {
        auto id = VisionSensor.getBlockID();
        // Serial.print("ID: ");
        Serial.println((int)id, DEC); // ???? 不知道为啥这行加进去数字就好了，怪
        auto klassRead = GetClass(id);
        auto klass = FeedClassifier(klassRead);
        if (klass == ClassifierType::Left) {
            TiltTo(TiltLeft);
        } else if (klass == ClassifierType::Right) {
            TiltTo(TiltRight);
        }
    }
    return true;
}
void ProcessTilt() {
    if (IsWaiting || IsTilted || IsInCooldown) return;
    IsWaiting = true;
    TiltTimer.in(TiltWaitLength, ManageTilt);
}
void IRReceive() {
    if (IRCallbackData.justWritten && !IRCallbackData.isRepeat) {
        IRCallbackData.justWritten = false;
        Serial.print("Address=0x");
        Serial.print(IRCallbackData.Address, HEX);
        Serial.print(" Command=0x");
        Serial.print(IRCallbackData.Command, HEX);
        Serial.println();

        auto buttonCode = (IRButtonCode)IRCallbackData.Command;
        if (buttonCode == IRButtonCode::Power) {
            IsProcessing = !IsProcessing;
            ResetFeed();
            if (!IsProcessing) ResetTiltWithoutCooldown();
            if (IsProcessing && IsLearning) FinishLearning();
        } else if (buttonCode == IRButtonCode::Left ||
                   buttonCode == IRButtonCode::Right ||
                   buttonCode == IRButtonCode::Down) {
            IsLearning = true;
            IsProcessing = false;
            CurrentLearningType =
                buttonCode == IRButtonCode::Down   ? ClassifierType::Empty
                : buttonCode == IRButtonCode::Left ? ClassifierType::Left
                                                   : ClassifierType::Right;
            ResetTiltWithoutCooldown();
            CurrentLearningID = HeaderAddID(CurrentLearningType);
        } else if (buttonCode == IRButtonCode::OK && IsLearning) {
            FinishLearning();
        } else if (buttonCode == IRButtonCode::Refresh) {
            VisionSensor.forgetLearn();
            ResetHeader();
            ResetTiltWithoutCooldown();
        }
    }
}

void UpdateStatusLED() {
    digitalWrite(StatusGreenPin, IsProcessing ? HIGH : LOW);
    digitalWrite(StatusYellowPin, IsLearning ? HIGH : LOW);
    digitalWrite(StatusRedPin, !IsProcessing || IsBinFull ? HIGH : LOW);
}
void UpdateDistance() {
    if (!IsProcessing) return;
    Distance = DistanceSensor.measureDistanceCm();
    // Serial.println(Distance);
}
void LearnProcess() {
    if (!IsLearning) return;
    if (!VisionSensor.learnOnece(CurrentLearningID)) {
        Serial.print("Learning not successful on ");
        Serial.println(CurrentLearningID);
    }
}
void Process() {
    if (!IsProcessing) return;
    if (Distance < MaximumDetectionRange) {
        ProcessTilt();
    }
}

void loop() {
    TaskScheduler.execute();
    TiltTimer.tick();
}

void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand,
                              bool isRepeat) {
    IRCallbackData.Address = aAddress;
    IRCallbackData.Command = aCommand;
    IRCallbackData.isRepeat = isRepeat;
    IRCallbackData.justWritten = true;
}