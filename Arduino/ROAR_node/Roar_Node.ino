#include <string.h>
#include <Servo.h>

int throttleIn = 1500; //global variable for throttle
int steeringIn = 1500; //global variable for steering
int receiverStateVar = LOW;

class Car {
  public:
    void initArduino();
    void readReceiver();
    void writeToActuators();
    void receiverState();
    void idling();
    float saturateMotor(float x);
    float saturateServo(float x);

  private:

    const int MOTOR_PIN = 3;
    const int SERVO_PIN = 2;

    const int ch1 = 4;
    const int ch2 = 5;
    const int receiver_state_pin = 30;

    Servo throttle;
    Servo steering;

    const int MOTOR_MAX = 1750;
    const int MOTOR_MIN = 800;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 3000;
    const int THETA_MIN = 0;

    uint16_t microseconds2PWM(uint16_t microseconds);
};

Car car;


void setup() {
  Serial.begin(115200);
  car.initArduino();
}

//Main Program
void loop() {
  car.receiverState();
  Serial.print(receiverStateVar);
  if (receiverStateVar == LOW) {
    car.readReceiver(); //reads reciever data.
    car.writeToActuators();
    Serial.print(car.saturateMotor(throttleIn));
    Serial.print(" ");
    Serial.print(car.saturateServo(steeringIn));
    Serial.println();
  }
  else if (receiverStateVar == HIGH) {
    char receivedData[100]; //creates variable to store data from jetson
    byte size = Serial.readBytes(receivedData, 100); //reads serial data into buffer and times out after 100ms
    receivedData[size] = 0;
    char *s = strtok(receivedData, "@& "); //allows string to be broken into tokens by " ".
    if (s != NULL) throttleIn = atoi(s); //sets variable to received data and converts ASCII to integer if message is not empty
    s = strtok(NULL, " ");
    if (s != NULL) steeringIn = atoi(s); //sets variable to received data and converts ASCII to integer if message is not empty
    car.writeToActuators(); //writes actuation variables to servo and ESC.
  }
}

void Car::initArduino() {
  throttle.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(receiver_state_pin, INPUT);
}

void Car::receiverState() {
  receiverStateVar = digitalRead(receiver_state_pin);
}

void Car::readReceiver() {
  throttleIn = pulseIn (ch2, HIGH); //replace with hardware timers.
  steeringIn = pulseIn (ch1, HIGH);
}

float Car::saturateMotor(float x) {
  if (x > MOTOR_MAX) {
    x = MOTOR_MAX;
  }
  if (x < MOTOR_MIN) {
    x = MOTOR_MIN;
  }
  return x;
}

float Car::saturateServo(float x) {
  if (x > THETA_MAX) {
    x = THETA_MAX;
  }
  if (x < THETA_MIN) {
    x = THETA_MIN;
  }
  return x;
}

void Car::writeToActuators() {
  throttle.writeMicroseconds( (uint16_t) saturateMotor( throttleIn ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( steeringIn ) );
}

void Car::idling() {
  throttle.writeMicroseconds( (uint16_t) MOTOR_NEUTRAL );
  steering.writeMicroseconds( (uint16_t) THETA_CENTER  );
}
