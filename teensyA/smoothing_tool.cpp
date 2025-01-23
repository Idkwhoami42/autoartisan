#include "smoothing_tool.h"

#define HORIZONTAL_SERVO_PIN 0
#define VERTICAL_SERVO_PIN 1

Servo horizontalServo;
Servo verticalServo;

inline void initializeServos() {
    horizontalServo.attach(HORIZONTAL_SERVO_PIN);
    verticalServo.attach(VERTICAL_SERVO_PIN);
}

inline void retractHorizontalTool() {
    horizontalServo.write(93);
}

inline void retractVerticalTool() {
    verticalServo.write(93);
}

inline void extendHorizontalTool(int pos) {
    horizontalServo.write(93+pos);
}

inline void extendVerticalTool(int pos) {
    verticalServo.write(93+pos);
}