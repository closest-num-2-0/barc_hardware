#define PHASE_A_servo 32
#define PHASE_B_servo 34
#define PHASE_C_servo 36

int encoder_servo_state_A = 0;
int encoder_servo_state_B = 0;
int encoder_servo_state_C = 0;

int direction_servo = 0;
float servo_angle = 0;
int enc_count = 0;

void setup() {

  Serial.begin(115200);

  pinMode(PHASE_A_servo, INPUT_PULLUP);
  pinMode(PHASE_B_servo, INPUT_PULLUP);
  pinMode(PHASE_C_servo, INPUT_PULLUP);

  attachInterrupt(PHASE_A_servo, calc_A_servo, CHANGE);
  attachInterrupt(PHASE_B_servo, calc_B_servo, CHANGE);
  attachInterrupt(PHASE_C_servo, calc_C_servo, CHANGE);

}

void loop() {

  servo_angle = enc_count*0.3515625;
  Serial.println();
  Serial.print(servo_angle);
  Serial.print(" , ");
  Serial.print(enc_count);
  Serial.print(" , ");
  Serial.print(encoder_servo_state_A);
  Serial.print(" , ");
  Serial.print(encoder_servo_state_B);
  Serial.print(" , ");
  Serial.print(encoder_servo_state_C);

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
  enc_count += direction_servo;
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
  enc_count += direction_servo;
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
  enc_count += direction_servo;
}
