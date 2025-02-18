#include <Arduino.h>
#include <Servo.h>
#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

#define BRUSH_SERVO_PIN 0
#define VERTICAL_SMOOTHING_SERVO_PIN 2
#define HORIZONTAL_SMOOTHING_SERVO_PIN 3
#define MOTOR_SPEED 50

#define BRUSH_SERVO_DEFAULT_POSITION 93

#define EXTRUDER_PWM_PIN 6
#define EXTRUDER_IN1_PIN 5
#define EXTRUDER_IN2_PIN 4

#define BRUSH_PWN_PIN 9
#define BRUSH_IN1_PIN 8
#define BRUSH_IN2_PIN 7

Servo brushServo;
Servo verticalSmoothingServo;
Servo horizontalSmoothingServo;
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

inline void setupBrushServo() { brushServo.attach(BRUSH_SERVO_PIN); }

inline void setupAS5047P() { 
    while (!as5047p.initSPI()) {
        Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
        delay(500);
    }
}

inline float getAngle() {
    return as5047p.readAngleDegree();
}

inline void setupSmoothingServos() {
    verticalSmoothingServo.attach(VERTICAL_SMOOTHING_SERVO_PIN);
    horizontalSmoothingServo.attach(HORIZONTAL_SMOOTHING_SERVO_PIN);
}

inline void setupBrushMotor() {
    pinMode(BRUSH_PWN_PIN, OUTPUT);
    pinMode(BRUSH_IN1_PIN, OUTPUT);
    pinMode(BRUSH_IN2_PIN, OUTPUT);

    digitalWrite(BRUSH_IN1_PIN, HIGH);
    digitalWrite(BRUSH_IN2_PIN, LOW);
}

inline void activateBrush() {
    brushServo.write(BRUSH_SERVO_DEFAULT_POSITION + 25);
    analogWrite(5, 240);
}

inline void deactivateBrush() {
    analogWrite(5, 0);
    brushServo.write(BRUSH_SERVO_DEFAULT_POSITION);
}

inline void activateVerticalSmoothing() { verticalSmoothingServo.write(93 + 75); }

inline void activateHorizontalSmoothing() { horizontalSmoothingServo.write(93 + 73); }

inline void deactivateVerticalSmoothing() { verticalSmoothingServo.write(93); }

inline void deactivateHorizontalSmoothing() { horizontalSmoothingServo.write(93); }

inline void setupExtruderMotor() {
    pinMode(EXTRUDER_PWM_PIN, OUTPUT);
    pinMode(EXTRUDER_IN1_PIN, OUTPUT);
    pinMode(EXTRUDER_IN2_PIN, OUTPUT);

    digitalWrite(EXTRUDER_PWM_PIN, LOW);
    digitalWrite(EXTRUDER_IN1_PIN, LOW);
    digitalWrite(EXTRUDER_IN2_PIN, LOW);
}

inline void activateExtruderMotor() {
    analogWrite(EXTRUDER_PWM_PIN, MOTOR_SPEED);
    digitalWrite(EXTRUDER_IN1_PIN, HIGH);
    digitalWrite(EXTRUDER_IN2_PIN, LOW);
}

inline void deactivateExtruderMotor() {
    analogWrite(EXTRUDER_PWM_PIN, 0);
    digitalWrite(EXTRUDER_IN1_PIN, LOW);
    digitalWrite(EXTRUDER_IN2_PIN, LOW);
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

    analogWrite(EXTRUDER_PWM_PIN, MOTOR_SPEED);
    digitalWrite(EXTRUDER_IN1_PIN, LOW);
    digitalWrite(EXTRUDER_IN2_PIN, HIGH);

    delay(duration);

    deactivateExtruderMotor();
}