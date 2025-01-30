#include <Arduino.h>
#include <Servo.h>

#define BRUSH_SERVO_PIN 2
#define VERTICAL_SMOOTHING_SERVO_PIN 3
#define HORIZONTAL_SMOOTHING_SERVO_PIN 4
#define MOTOR_SPEED 15

#define BRUSH_SERVO_DEFAULT_POSITION 93

Servo brushServo;
Servo verticalSmoothingServo;
Servo horizontalSmoothingServo;

inline void setupBrushServo() { brushServo.attach(BRUSH_SERVO_PIN); }

inline void setupSmoothingServos() {
    verticalSmoothingServo.attach(VERTICAL_SMOOTHING_SERVO_PIN);
    horizontalSmoothingServo.attach(HORIZONTAL_SMOOTHING_SERVO_PIN);
}

inline void setupBrushMotor() {
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
}

inline void activateBrush() {
    brushServo.write(BRUSH_SERVO_DEFAULT_POSITION + 25);
    analogWrite(5, 240);
}

inline void deactivateBrush() {
    analogWrite(5, 0);
    brushServo.write(BRUSH_SERVO_DEFAULT_POSITION);
}

inline void activateVerticalSmoothing() { verticalSmoothingServo.write(93 + 10); }

inline void activateHorizontalSmoothing() { horizontalSmoothingServo.write(93 + 10); }

inline void deactivateVerticalSmoothing() { verticalSmoothingServo.write(93); }

inline void deactivateHorizontalSmoothing() { horizontalSmoothingServo.write(93); }


inline void setupExtruderMotor() {
    pinMode(29, OUTPUT);
    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);

    digitalWrite(29, LOW);
    digitalWrite(31, LOW);
    digitalWrite(32, LOW);
}

inline void activateExtruderMotor() {
    analogWrite(29, MOTOR_SPEED);
    digitalWrite(31, HIGH);
    digitalWrite(32, LOW);
}

inline void deactivateExtruderMotor() {
    analogWrite(29, 0);
    digitalWrite(31, LOW);
    digitalWrite(32, LOW);
}


inline void goForward(int duration) {
    deactivateExtruderMotor();
    delay(300);

    activateExtruderMotor();
    delay(duration);
    deactivateExtruderMotor();
}

inline void goBack(int duration) {
    deactivateExtruderMotor();
    delay(300);

    analogWrite(29, MOTOR_SPEED);
    digitalWrite(31, LOW);
    digitalWrite(32, HIGH);

    delay(duration);

    deactivateExtruderMotor();
}