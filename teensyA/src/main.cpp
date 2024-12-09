#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/int32.h>
#include "ICM_20948.h"
#include "Adafruit_VL6180X.h"

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_tof;
geometry_msgs__msg__Quaternion msg;
std_msgs__msg__Int32 msg_tof;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

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

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
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
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "imu"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_tof,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "tof"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

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
}

void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      msg.x = qx;
      msg.y = qy;
      msg.z = qz;
      msg.w = qw;

      // double t0 = +2.0 * (qw * qx + qy * qz);
      // double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      // double roll = atan2(t0, t1) * 180.0 / PI;

      // double t2 = +2.0 * (qw * qy - qx * qz);
      // t2 = t2 > 1.0 ? 1.0 : t2;
      // t2 = t2 < -1.0 ? -1.0 : t2;
      // double pitch = asin(t2) * 180.0 / PI;

      // double t3 = +2.0 * (qw * qz + qx * qy);
      // double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      // double yaw = atan2(t3, t4) * 180.0 / PI;

      // msg.data = roll;

      uint8_t range = vl.readRange();
      uint8_t status = vl.readRangeStatus();
      msg_tof.data = range;

      RCSOFTCHECK(rcl_publish(&publisher_imu, &msg, NULL));
      RCSOFTCHECK(rcl_publish(&publisher_tof, &msg_tof, NULL));

      // SERIAL_PORT.print(F("Roll:"));
      // SERIAL_PORT.print(roll, 1);
      // SERIAL_PORT.print(F(" Pitch:"));
      // SERIAL_PORT.print(pitch, 1);
      // SERIAL_PORT.print(F(" Yaw:"));
      // SERIAL_PORT.println(yaw, 1);
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)
  {
    delay(10);
  }

  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) { // If the agent is not reachable, reset the board
    digitalWrite(LED_PIN, LOW);
    asm volatile("bx %0" :: "r" (0x00000000));
  }
}
