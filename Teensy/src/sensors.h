#include <Arduino.h>
#include <Servo.h>
#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

#define BRUSH_SERVO_PIN 0
#define VERTICAL_SMOOTHING_SERVO_PIN 1
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

float percentage = std::nanf("");

inline void setupBrushServo() { brushServo.attach(BRUSH_SERVO_PIN); }

inline void setupAS5047P() { 
    as5047p.initSPI();
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
    analogWrite(BRUSH_PWN_PIN, 240);
}

inline void deactivateBrush() {
    analogWrite(BRUSH_PWN_PIN, 0);
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


// inline int get_section(float p) {
//     if (p >= 0.0f && p < 10.0f)
//         return 1;
//     else if (p >= 10.0f && p < 20.0f)
//         return 2;
//     else if (p >= 20.0f && p < 30.0f)
//         return 3;
//     else if (p >= 30.0f && p < 40.0f)
//         return 4;
//     else if (p >= 40.0f && p < 50.0f)
//         return 5;
//     else if (p >= 50.0f && p < 60.0f)
//         return 6;
//     else if (p >= 60.0f && p < 70.0f)
//         return 7;
//     else if (p >= 70.0f && p < 80.0f)
//         return 8;
//     else if (p >= 80.0f && p < 90.0f)
//         return 9;
//     else
//         return 10;
// }

// inline int update_capacity(float angle) {
//     float d =
//         (angle / static_cast<float>(360)) * (static_cast<float>(60) / static_cast<float>(34))
//         * 2.0;
//     float prev_percentage = std::isnan(percentage) ? std::nanf("") : percentage;
//     percentage = 100 * (17.75 - d) / 17.75;


//     if (std::isnan(prev_percentage) || (get_section(prev_percentage) != get_section(percentage)))
//     {
//         return get_section(percentage);
//     }
// }