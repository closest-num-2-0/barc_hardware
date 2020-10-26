#include "Servo.h"

int TESTING_STATE = true; //test between receiving Nvidia Jetson Signal and Writing Constant PWM

//wheel variables
const float pi = 3.14159265359;
const float R = 0.0325; //wheel radius

//receiver variables
char receivedData[100]; //creates variable to store data from jetson (100 is byte size)
char handshake = '&';
float throttle_read = 1500;
float steering_read = 1500;
float throttle_write = 1500;
float steering_write = 1500;
float receiverStateVar = LOW;
float str_prev_2 = 1500;
float str_prev_1 = 1500;
float thr_prev_2 = 1500;
float thr_prev_1 = 1500;

//servo encoder variables
int encoder_servo_state_A = 0;
int encoder_servo_state_B = 0;
int encoder_servo_state_C = 0;
int direction_servo = 0;
int servo_enc_count = 0;
float servo_angle = 0;
float steering_angle = 0;

//wheel encoder variables
int wheel_enc_count_FL = 0;
int wheel_enc_count_FR = 0;
int wheel_enc_count_BL = 0;
int wheel_enc_count_BR = 0;
float time_initial = 0;
volatile uint32_t FL_time_prev = 0;
volatile uint32_t FR_time_prev = 0;
volatile uint32_t BL_time_prev = 0;
volatile uint32_t BR_time_prev = 0;
volatile uint32_t FL_time_pres = 0;
volatile uint32_t FR_time_pres = 0;
volatile uint32_t BL_time_pres = 0;
volatile uint32_t BR_time_pres = 0;

#define CHANNEL1_IN_PIN 2
#define CHANNEL2_IN_PIN 3
#define CHANNEL1_OUT_PIN 4
#define CHANNEL2_OUT_PIN 5
#define FL_IN_PIN 22
#define FR_IN_PIN 24
#define BL_IN_PIN 26
#define BR_IN_PIN 28
#define RECEIVER_STATE_PIN 30
#define PHASE_A_servo 32
#define PHASE_B_servo 34
#define PHASE_C_servo 36

Servo servoChannel1;
Servo servoChannel2;

#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define ENCODER_FL_FLAG 3
#define ENCODER_FR_FLAG 4
#define ENCODER_BL_FLAG 5
#define ENCODER_BR_FLAG 6

volatile uint32_t bUpdateFlagsShared;

volatile uint32_t CHANNEL_1_IN_PWM;
volatile uint32_t CHANNEL_2_IN_PWM;
volatile uint32_t FL_DT;
volatile uint32_t FR_DT;
volatile uint32_t BL_DT;
volatile uint32_t BR_DT;

void setup() {

  Serial.begin(115200);

  servoChannel1.attach(CHANNEL1_OUT_PIN);
  servoChannel2.attach(CHANNEL2_OUT_PIN);

  pinMode(RECEIVER_STATE_PIN, INPUT);
//  pinMode(FL_IN_PIN, INPUT_PULLUP);
//  pinMode(FR_IN_PIN, INPUT_PULLUP);
//  pinMode(BL_IN_PIN, INPUT_PULLUP);
//  pinMode(BR_IN_PIN, INPUT_PULLUP);
  pinMode(PHASE_A_servo, INPUT_PULLUP);
  pinMode(PHASE_B_servo, INPUT_PULLUP);
  pinMode(PHASE_C_servo, INPUT_PULLUP);

  attachInterrupt(CHANNEL1_IN_PIN, MEASURE_CHANNEL_1, CHANGE);
  attachInterrupt(CHANNEL2_IN_PIN, MEASURE_CHANNEL_2, CHANGE);
//  attachInterrupt(FL_IN_PIN, MEASURE_FL, CHANGE);
//  attachInterrupt(FR_IN_PIN, MEASURE_FR, CHANGE);
//  attachInterrupt(BL_IN_PIN, MEASURE_BL, CHANGE);
//  attachInterrupt(BR_IN_PIN, MEASURE_BR, CHANGE);
  attachInterrupt(PHASE_A_servo, calc_A_servo, CHANGE);
  attachInterrupt(PHASE_B_servo, calc_B_servo, CHANGE);
  attachInterrupt(PHASE_C_servo, calc_C_servo, CHANGE);

  time_initial = micros();
}

void loop() {
  receiverStateVar = digitalRead(RECEIVER_STATE_PIN);

  static uint32_t unChannel1In;
  static uint32_t unChannel2In;

  static uint32_t bUpdateFlags;

  if (bUpdateFlagsShared) {
    //noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;

    if (bUpdateFlags & CHANNEL1_FLAG) {
      //      if (unChannel1In > 2025) {
      //        unChannel1In = str_prev_1 - str_prev_2 + str_prev_1;
      //      }
      //      str_prev_2 = str_prev_1;
      //      str_prev_1 = unChannel1In;
      unChannel1In = CHANNEL_1_IN_PWM;
    }

    if (bUpdateFlags & CHANNEL2_FLAG) {
      //      if (unChannel2In > 2025) {
      //        unChannel2In = thr_prev_1 - thr_prev_2 + thr_prev_1;
      //      }
      //      str_prev_2 = str_prev_1;
      //      str_prev_1 = unChannel1In;
      unChannel2In = CHANNEL_2_IN_PWM;
    }

//    if (bUpdateFlags & ENCODER_FL_FLAG) {
//      FL_DT = FL_time_pres - FL_time_prev;
//    }
//
//    if (bUpdateFlags & ENCODER_FR_FLAG) {
//      FR_DT = FL_time_pres - FR_time_prev;
//    }
//
//    if (bUpdateFlags & ENCODER_BL_FLAG) {
//      BL_DT = BL_time_pres - BL_time_prev;
//    }
//
//    if (bUpdateFlags & ENCODER_BR_FLAG) {
//      BR_DT = BR_time_pres - BR_time_prev;
//    }

    bUpdateFlagsShared = 0;

    //interrupts();
  }

  servo_angle = servo_enc_count * 0.3515625; //change to calculate function that calculates all angles and velocities
  steering_angle = servo_angle * 0.65;

  Serial.println();
  Serial.print(CHANNEL_1_IN_PWM);
  Serial.print(" , ");
  Serial.print(CHANNEL_2_IN_PWM);
  Serial.print(" , ");
  Serial.print(steering_angle);
//  Serial.print(" , ");
//  Serial.print(wheel_enc_count_FL);
//  Serial.print(" , ");
//  Serial.print(wheel_enc_count_FR);
//  Serial.print(" , ");
//  Serial.print(wheel_enc_count_BL);
//  Serial.print(" , ");
//  Serial.print(wheel_enc_count_BR);

  if (TESTING_STATE = true) {
    Serial.print("ang");
    Serial.print(servo_angle);
    Serial.print(" , ss_enc_cnt: ");
    Serial.print(servo_enc_count);
    Serial.print(" ,enc_A ");
    Serial.print(encoder_servo_state_A);
    Serial.print(" ,enc_B");
    Serial.print(encoder_servo_state_B);
    Serial.print(" ,enc_C");
    Serial.print(encoder_servo_state_C);
  }

  steering_read = CHANNEL_1_IN_PWM;
  throttle_read = CHANNEL_2_IN_PWM;

  if (receiverStateVar == LOW) {
    throttle_write = throttle_read;
    steering_write = steering_read;

    servoChannel2.writeMicroseconds(throttle_write);
    servoChannel1.writeMicroseconds(steering_write);
  } else if (receiverStateVar == HIGH) {

    if (TESTING_STATE = true) {
      throttle_write = 1500;
      steering_write = 1500;
    } else {

      // NEW REQUIRED FORMAT: "HANDSHAKE" "PWMTHROTTLE" "STEERINGIN"
      // (neutral formatting): & 1500 1500

      byte size = Serial.readBytes(receivedData, 100); //reads serial data into buffer and times out after 100ms
      receivedData[size] = 0; //end of the string can be specified with a 0.
      char *s = strtok(receivedData, " "); //allows string to be broken into tokens by " ".
      if (s[0] == handshake) {
        s = strtok(NULL, " ");
        if (s != NULL) throttle_read = atoi(s); //sets variable to received data and converts ASCII to integer if message is not empty
        s = strtok(NULL, " ");
        if (s != NULL) steering_read = atoi(s); //sets variable to received data and converts ASCII to integer if message is not empty
        steering_read = steering_write;
        throttle_read = throttle_write;
      } else {

        throttle_write = 1500; //neutral PWM
        steering_write = 1500; //neutral PWM

      }
    }

    servoChannel2.writeMicroseconds(throttle_write);
    servoChannel1.writeMicroseconds(steering_write);

  }

  bUpdateFlags = 0;
}

void MEASURE_CHANNEL_1()
{
  static uint32_t ulStart;

  if (digitalRead(CHANNEL1_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    CHANNEL_1_IN_PWM = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL1_FLAG;
  }
}

void MEASURE_CHANNEL_2()
{
  static uint32_t ulStart;

  if (digitalRead(CHANNEL2_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    CHANNEL_2_IN_PWM = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL2_FLAG;
  }
}

void readServoEncoder() {
  encoder_servo_state_A = digitalRead(PHASE_A_servo);
  encoder_servo_state_B = digitalRead(PHASE_B_servo);
  encoder_servo_state_C = digitalRead(PHASE_C_servo);
}

void calc_A_servo() {
  readServoEncoder();
  if (encoder_servo_state_A + encoder_servo_state_B + encoder_servo_state_C == 2) {
    if (encoder_servo_state_B == HIGH) {
      direction_servo = -1;
    } else {
      direction_servo = 1;
    }
  } else {
    if (encoder_servo_state_B == HIGH) {
      direction_servo = 1;
    } else {
      direction_servo = -1;
    }
  }
  servo_enc_count += direction_servo;
}

void calc_B_servo() {
  readServoEncoder();
  if (encoder_servo_state_A + encoder_servo_state_B + encoder_servo_state_C == 2) {
    if (encoder_servo_state_A == HIGH) {
      direction_servo = 1;
    } else {
      direction_servo = -1;
    }
  } else {
    if (encoder_servo_state_A == HIGH) {
      direction_servo = -1;
    } else {
      direction_servo = 1;
    }
  }
  servo_enc_count += direction_servo;
}

void calc_C_servo() {
  readServoEncoder();
  if (encoder_servo_state_A + encoder_servo_state_B + encoder_servo_state_C == 2) {
    if (encoder_servo_state_A == HIGH) {
      direction_servo = -1;
    } else {
      direction_servo = 1;
    }
  } else {
    if (encoder_servo_state_A == HIGH) {
      direction_servo = 1;
    } else {
      direction_servo = -1;
    }
  }
  servo_enc_count += direction_servo;
}

//void MEASURE_FL() {
//  wheel_enc_count_FL++;
//  FL_time_prev = FL_time_pres;
//  FL_time_pres = micros();
//  bUpdateFlagsShared |= ENCODER_FL_FLAG;
//}
//
//void MEASURE_FR() {
//  wheel_enc_count_FR++;
//  FR_time_prev = FR_time_pres;
//  FR_time_pres = micros();
//  bUpdateFlagsShared |= ENCODER_FR_FLAG;
//}
//
//void MEASURE_BL() {
//  wheel_enc_count_BL++;
//  BL_time_prev = BL_time_pres;
//  BL_time_pres = micros();
//  bUpdateFlagsShared |= ENCODER_BL_FLAG;
//}
//
//void MEASURE_BR() {
//  wheel_enc_count_BR++;
//  BR_time_prev = BR_time_pres;
//  BR_time_pres = micros();
//  bUpdateFlagsShared |= ENCODER_BR_FLAG;
//}
