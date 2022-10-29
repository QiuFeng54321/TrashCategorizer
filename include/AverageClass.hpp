#ifndef AVERAGECLASS_HPP
#define AVERAGECLASS_HPP

#include "Common.hpp"

const short ClassifierFeedLength = 9;
ClassifierType ClassifierFeed[ClassifierFeedLength];
short ClassifierFeedCounter[4];
short ClassifierFeedTop = 0;
short ClassifierFeedTotal = 0;

inline ClassifierType GetAverageClass() {
    if (ClassifierFeedTotal != ClassifierFeedLength)
        return ClassifierType::Empty;
    if (ClassifierFeedCounter[2] + ClassifierFeedCounter[3] <=
        ClassifierFeedLength / 3 * 2 || abs(ClassifierFeedCounter[2] - ClassifierFeedCounter[3]) <= 2)
        return ClassifierType::Empty;
    if (ClassifierFeedCounter[2] > ClassifierFeedCounter[3])
        return ClassifierType::Left;
    if (ClassifierFeedCounter[2] < ClassifierFeedCounter[3])
        return ClassifierType::Right;
    return ClassifierType::Empty;
}
inline ClassifierType FeedClassifier(ClassifierType type) {
    const auto& previous = ClassifierFeed[ClassifierFeedTop];
    if (previous != ClassifierType::None) ClassifierFeedTotal--;
    ClassifierFeedCounter[(short)previous]--;
    ClassifierFeed[ClassifierFeedTop] = type;
    ClassifierFeedCounter[(short)type]++;
    ClassifierFeedTotal++;
    ClassifierFeedTop = (ClassifierFeedTop + 1) % ClassifierFeedLength;
    auto avg = GetAverageClass();
    Serial.print("Add ");
    Serial.print((uint8_t)type, 10);
    Serial.print(", Average = ");
    Serial.println((int)avg, 10);
    return avg;
}
inline void ResetFeed() {
    memset(ClassifierFeed, 0, ClassifierFeedLength * sizeof(ClassifierType));
    memset(ClassifierFeedCounter, 0, 4 * sizeof(short));
    ClassifierFeedTop = 0;
    ClassifierFeedTotal = 0;
}

#endif