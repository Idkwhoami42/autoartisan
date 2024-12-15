#include <Wire.h>
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// #include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/int32.h>
#include "ICM_20948.h"
#include "Adafruit_VL6180X.h"

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define SERIAL_PORT Serial

#define SPI_PORT SPI
#define CS_PIN 2

#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;
Adafruit_VL6180X vl = Adafruit_VL6180X();
float init_vol = 0.0;
int i = 10;
IntervalTimer interrupt_timer;
volatile int capacity = 0;

rcl_publisher_t publisher_tof;
// geometry_msgs__msg__Quaternion msg;
std_msgs__msg__Int32 msg_tof;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

float rounded_rect_area(float a, float b, float radius) {
    return ((a*b) + 2*radius*(a+b) + 3.141593*radius*radius);
}

float frustrum_volume(float area1, float area2, float height) {
    return (height / 3) * (area1 + area2 + sqrtf(area1*area2));
}

void get_capacity() {
    uint8_t status = vl.readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
        float h = (float) vl.readRange();

        float lsf = (101.0 - h) / 101.0;

        if (lsf > 0.0) {
            capacity = (int) (100.0 * powf(lsf, 3.0));
        }
    }
}

void setup() {
  SERIAL_PORT.begin(115200);

  while(!Serial){
    delay(100);
  }

  set_microros_serial_transports(SERIAL_PORT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 
  
  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 86);
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);)
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_tof,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "percentage_capacity"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Calculate initial volume
  float approx_area = (25.0*rounded_rect_area(39.87, 103.34, 8) + (39.87*116.84)) / 26.0;
  init_vol = frustrum_volume(rounded_rect_area(187.94, 179.56, 8.0), approx_area, 101.0);

  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  SERIAL_PORT.println(F("Device connected!"));

  bool success = true;

  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; 
  }

  interrupt_timer.begin(get_capacity, 150000);
}

void loop() {
    int capacity_cpy;

    noInterrupts();
    capacity_cpy = capacity;
    interrupts();

    int cap_index = (int) floor(((float) (((float) capacity_cpy) / 10.0)));

    // This is set up so that messages are only published when the capacity crosses a threshold
    // The thresholds: 0%, 10%, 20%, 30%, 40%, 50%, 60%, 70%, 80%, 90%, 100%
    if (cap_index != i) {
      // Serial.printf("Capacity: %d\n", capacity_cpy);
      msg_tof.data = capacity_cpy;
      RCSOFTCHECK(rcl_publish(&publisher_tof, &msg_tof, NULL));
      i = cap_index;
    }
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) { // If the agent is not reachable, reset the board
      digitalWrite(LED_PIN, LOW);
      asm volatile("bx %0" :: "r" (0x00000000));
    }
}
