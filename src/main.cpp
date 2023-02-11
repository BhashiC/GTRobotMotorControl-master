#include <Arduino.h>
#include <motorController.h>
#include <allInterrupts.h>
#include <serialHandler.h>
#include <robotKinematics.h>
#include <ps2Handler.h>
#include <math.h>
#include <SAMDUETimerInterrupt.h> //https://github.com/khoih-prog/SAMDUE_TimerInterrupt
#include <SAMDUE_ISR_Timer.h>     //https://github.com/khoih-prog/SAMDUE_TimerInterrupt

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <stdio.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor_sub_twist;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Timer parameters
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
int led_rosCom = 31;
int led_motPid = 33;
int led_error = 35;

int pid_timer_interval_ms = 50;

unsigned int left_motor_pwm = 6; // pwm motor left connect to AN1 at MDS10
unsigned int left_motor_dir = 4; // direction motor left connect to DIG1 at MDS10
// Actually cnannel B is 19, but to ensure possitive encoder count when robot moving forward we changed
unsigned int left_motor_encoderA = 19; // encoder pinA motor left
// Actually cnannel A is 18, but to ensure possitive encoder count when robot moving forward we changed
unsigned int left_motor_encoderB = 18; // encoder pinB motor left

unsigned int right_motor_pwm = 5;       // pwm motor right connect to AN2 at MDS10
unsigned int right_motor_dir = 7;       // direction motor right connect to DIG2 at MDS10
unsigned int right_motor_encoderA = 20; // encoder pinB motor right
unsigned int right_motor_encoderB = 21; // encoder pinA motor right

double wheelRadius = 0.100;      // meters
double wheelBase = 0.55;         // meters
double robotOmegaXMax = 1;       // ms-1
double robotOmegaXMin = -1;      // ms-1
double robotOmegaZMax = PI / 2;  // rads-1
double robotOmegaZMin = -PI / 2; // rads-1

robotKinematics *robotKinObjPtr = new robotKinematics(wheelRadius, wheelBase, robotOmegaXMax, robotOmegaXMin, robotOmegaZMax, robotOmegaZMin);
// robotKinematics robotKinObj = *robotKinObjPtr;

motorController *allInterrupts::leftMotorPtr = new motorController(motorName::left, left_motor_pwm, left_motor_dir, left_motor_encoderA, left_motor_encoderB);
motorController *allInterrupts::rightMotorPtr = new motorController(motorName::right, right_motor_pwm, right_motor_dir, right_motor_encoderA, right_motor_encoderB);

motorController &leftMotor = *allInterrupts::leftMotorPtr;
motorController &rightMotor = *allInterrupts::rightMotorPtr;

serialHandler *serialObjPtr = new serialHandler(*allInterrupts::leftMotorPtr, *allInterrupts::rightMotorPtr, *robotKinObjPtr);
// serialHandler serialObj = *serialObjPtr;

ps2Handler *ps2ObjPtr = new ps2Handler(*allInterrupts::leftMotorPtr, *allInterrupts::rightMotorPtr, *robotKinObjPtr);
// ps2Handler ps2Obj = *ps2ObjPtr;

/// @brief Left motor job loop
void leftMotorJob()
{
  leftMotor.pidObj->setCurrentPositionCount(leftMotor.current_position_motor);
  leftMotor.motorCalculateShaftSpeed(pid_timer_interval_ms);
  if (!robotKinObjPtr->getIsRobotStop())
  {
    leftMotor.motorCalculatePid();
  }
}

/// @brief Right motor job loop
void rightMotorJob()
{
  rightMotor.pidObj->setCurrentPositionCount(rightMotor.current_position_motor);
  rightMotor.motorCalculateShaftSpeed(pid_timer_interval_ms);
  if (!robotKinObjPtr->getIsRobotStop())
  {
    rightMotor.motorCalculatePid();
  }
}

void serailJob()
{
  serialObjPtr->readSerialData();
  // serialObj.printEncoderCounts();
  serialObjPtr->printSpeeds(leftMotor.pidObj->getMappedControlVariable(), rightMotor.pidObj->getMappedControlVariable());
}

void ps2Job()
{
  ps2ObjPtr->readPs2Data();
  // robotKinObjPtr->printKinematicsData();
  //  serialObj.printEncoderCounts();
  serialObjPtr->printSpeeds(leftMotor.pidObj->getMappedControlVariable(), rightMotor.pidObj->getMappedControlVariable());
}

uint16_t attachDueInterrupt(double microseconds, timerCallback callback, const char *TimerName)
{
  DueTimerInterrupt dueTimerInterrupt = DueTimer.getAvailable();
  dueTimerInterrupt.attachInterruptInterval(microseconds, callback);
  uint16_t timerNumber = dueTimerInterrupt.getTimerNumber();
  //Serial.print(TimerName);
  //Serial.print(" attached to Timer(");
  //Serial.print(timerNumber);
  //Serial.println(")");
  return timerNumber;
}

void pidTimerHandler(void)
{
  // Set current motor positions
  leftMotor.pidObj->setCurrentPositionCount(leftMotor.current_position_motor);
  rightMotor.pidObj->setCurrentPositionCount(rightMotor.current_position_motor);
  
  // Calculate shaft speeds
  leftMotor.motorCalculateShaftSpeed(pid_timer_interval_ms);
  rightMotor.motorCalculateShaftSpeed(pid_timer_interval_ms);
  
  if (!robotKinObjPtr->getIsRobotStop())
  {
    digitalWrite(led_motPid, !digitalRead(led_motPid));
    leftMotor.motorCalculatePid();
    rightMotor.motorCalculatePid();
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(led_error, !digitalRead(led_error));
    delay(100);
  }
}

double angularZ = 0;
double linearX = 0;
/// @brief Set motor directions and pass target Rmps to Pid loop
void achieveRpms()
{
    digitalWrite(led_rosCom, !digitalRead(led_rosCom));

    if (robotKinObjPtr->getLeftWheelOmega() >= 0)
    {
        robotKinObjPtr->setIsRobotStop(false);
        leftMotor.motorForward();
        leftMotor.pidObj->setTargetSpeed(robotKinObjPtr->getLeftWheelOmega());
    }
    else
    {
        robotKinObjPtr->setIsRobotStop(false);
        leftMotor.motorBackward();
        leftMotor.pidObj->setTargetSpeed(-robotKinObjPtr->getLeftWheelOmega());
    }
    if (robotKinObjPtr->getRightWheelOmega() >= 0)
    {
        robotKinObjPtr->setIsRobotStop(false);
        rightMotor.motorForward();
        rightMotor.pidObj->setTargetSpeed(robotKinObjPtr->getRightWheelOmega());
    }
    else
    {
        robotKinObjPtr->setIsRobotStop(false);
        rightMotor.motorBackward();
        rightMotor.pidObj->setTargetSpeed(-robotKinObjPtr->getRightWheelOmega());
    }
}

void subscription_callback_twist(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // msg->anguar part's z is the Robot body's rotation
  angularZ = msg->angular.z;
  // correction for Z rotation
  angularZ = -angularZ;
  // msg->linear part's x is the Robot movement in forward direction
  linearX = msg->linear.x;

  if(angularZ == 0 && linearX == 0)
  {
      leftMotor.pidObj->setIntegral(0);
      rightMotor.pidObj->setIntegral(0);
      robotKinObjPtr->setRobotXdot(0);
      robotKinObjPtr->setRobotZdot(0);
      leftMotor.pidObj->setTargetSpeed(0);
      rightMotor.pidObj->setTargetSpeed(0);
      leftMotor.motorStop();
      rightMotor.motorStop();
      robotKinObjPtr->setIsRobotStop(true);
      return;
  }
  
  robotKinObjPtr->setRobotZdot(angularZ);
  robotKinObjPtr->setRobotXdot(linearX);
  
  if (robotKinObjPtr->getRobotZdot() > robotKinObjPtr->getRobotZdotMax())
  {
      robotKinObjPtr->setRobotZdot(robotKinObjPtr->getRobotZdotMax());
  }
  if (robotKinObjPtr->getRobotZdot() < robotKinObjPtr->getRobotZdotMin())
  {
      robotKinObjPtr->setRobotZdot(robotKinObjPtr->getRobotZdotMin());
  }    
  if (robotKinObjPtr->getRobotXdot() > robotKinObjPtr->getRobotXdotMax())
  {
      robotKinObjPtr->setRobotXdot(robotKinObjPtr->getRobotXdotMax());
  }
  if (robotKinObjPtr->getRobotXdot() < robotKinObjPtr->getRobotXdotMin())
  {
      robotKinObjPtr->setRobotXdot(robotKinObjPtr->getRobotXdotMin());
  }
  achieveRpms();
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  digitalWrite(led_rosCom, !digitalRead(led_rosCom));
  int pwm = msg->data;
  pwm = pwm < 0 ? -pwm : pwm;
  if(msg->data > 0)
  {
    leftMotor.motorForward();
    rightMotor.motorForward();
  }
  else if(msg->data < 0)
  {
    leftMotor.motorBackward();
    rightMotor.motorBackward();
  }
  else
  {
    leftMotor.motorStop();
    rightMotor.motorStop();
  }  
  leftMotor.motorSetPwm(pwm);
  rightMotor.motorSetPwm(pwm);
}

void setup()
{
  Serial.begin(115200);
  //Serial2.begin(9600);
  //Serial.println("Setup Start!");
  leftMotor.motorStop();
  rightMotor.motorStop();
  allInterrupts::attachInterruptFuncs();

  pinMode(led_rosCom, OUTPUT);
  pinMode(led_motPid, OUTPUT);
  pinMode(led_error, OUTPUT);

  digitalWrite(led_rosCom, HIGH);

  // Interval in microsecs
  attachDueInterrupt(pid_timer_interval_ms * 1000, pidTimerHandler, "ITimer3");

  set_microros_serial_transports(Serial);

  //delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create subscriber
  // RCCHECK(rclc_subscription_init_default(
  //   &subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_arduino_subscriber"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // create executor
  //RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub_twist, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_twist, &twist_subscriber, &twist_msg, &subscription_callback_twist, ON_NEW_DATA));

  //Serial.println("Setup Done!");
}

void loop()
{
  // leftMotorJob();
  // rightMotorJob();
  // serailJob();
  // pidTimerHandler();
  // ps2Job();

  // put your main code here, to run repeatedly:
  //delay(100);
  //RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub_twist, RCL_MS_TO_NS(100)));
}
