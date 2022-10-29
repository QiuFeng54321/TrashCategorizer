#ifndef EEPROMHELPER_HPP
#define EEPROMHELPER_HPP
#include "Common.hpp"

struct ROMHeader {
    bool Initialized;
    short LeftTop, RightTop;
};
const short LeftIDOffset = 32, RightIDOffset = 64, EmptyID = 96;
ROMHeader Header;

inline void PrintHeaderInfo(const ROMHeader& header) {
    Serial.print("Header: ");
    Serial.print(header.Initialized ? "Initialized, " : "Uninitialized, ");
    Serial.print("LeftTop = ");
    Serial.print(header.LeftTop);
    Serial.print(", RightTop = ");
    Serial.println(header.RightTop);
}

inline void SaveHeader() {
    EEPROM.put(0, Header);
    Serial.println("Header save:");
    PrintHeaderInfo(Header);
}

inline void ResetHeader() {
    Header.Initialized = true;
    Header.LeftTop = LeftIDOffset;
    Header.RightTop = RightIDOffset;
    SaveHeader();
}
inline void InitializeEEPROM() {
    Serial.println("Initializing EEPROM Header..."); 
    EEPROM.get(0, Header);
    if (!Header.Initialized) {
        Serial.println("First time initialization.");
        ResetHeader();
    }
    Serial.println("Header initialized.");
    PrintHeaderInfo(Header);
}

inline short HeaderAddID(ClassifierType type) {
    if (type == ClassifierType::Empty) return EmptyID;
    short res =
        (type == ClassifierType::Left ? Header.LeftTop++ : Header.RightTop++);
    SaveHeader();
    return res;
}

inline ClassifierType GetClass(int id) {
    if (id == EmptyID) return ClassifierType::Empty;
    if (LeftIDOffset <= id && id <= Header.LeftTop) return ClassifierType::Left;
    if (RightIDOffset <= id && id <= Header.RightTop) return ClassifierType::Right;
    return ClassifierType::None;
}

inline bool ReadROMClass(int id) {
    auto base = id >> 3;
    auto offset = id & 0xff;
    auto read = EEPROM.read(base);
    return (read >> offset) & 1;
}
inline void WriteROMClass(int id, bool val) {
    auto base = id >> 3;
    auto offset = id & 0xff;
    auto read = EEPROM.read(base);
    read |= val << offset;
    EEPROM.write(base, read);
}

#endif