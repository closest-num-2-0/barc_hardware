#include <string.h>
#include <Servo.h>

int throttle_read = 1500; //global variable for throttle
int steering_read = 1500; //global variable for steering
int throttle_write = 1500; //global variable for throttle
int steering_write = 1500; //global variable for steering
int receiverStateVar = LOW;
char receivedData[100]; //creates variable to store data from jetson (100 is byte size)
char handshake = '&';

class Car {
  public:
    void initArduino();
    void readReceiver();
    void writeToActuators();
    void receiverState();
    void idling();
    void Master_Multiplexer();
    void Slave_Multiplexer();
    float saturateMotor(float x);
    float saturateServo(float x);

  private:
    
    const int SERVO_PIN = 2;
    const int MOTOR_PIN = 3;
    const int ch1 = 4;
    const int ch2 = 5;
    const int MULTIPLEXER_PIN = 10;
    const int receiver_state_pin = 30;

    Servo throttle;
    Servo steering;
    Servo enable_Multiplexer;

    const int switchingValueMultiplexer = 1700;
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
  // Serial.print(receiverStateVar); Debugging purposes.
  if (receiverStateVar == LOW) {
    car.Master_Multiplexer(); //switches to MASTER on POLULU multiplexer
  } else if (receiverStateVar == HIGH) {
    car.Slave_Multiplexer(); //switches to SLAVE on POLULU multiplexer
  }

  if (receiverStateVar == LOW) {
    car.readReceiver(); //reads reciever data.
    Serial.print(car.saturateMotor(throttle_read));
    Serial.print(" ");
    Serial.print(car.saturateServo(steering_read));
    // Serial.print(" ");
    // Serial.print(receiverStateVar);
    Serial.println();
  } else if (receiverStateVar == HIGH) {
    throttle_write = 1550;
    steering_write = 1900;
    car.writeToActuators();

    // NEW REQUIRED FORMAT: "HANDSHAKE" "PWMTHROTTLE" "STEERINGIN"
    // (neutral formatting): & 1500 1500

    //    byte size = Serial.readBytes(receivedData, 100); //reads serial data into buffer and times out after 100ms
    //    receivedData[size] = 0; //end of the string can be specified with a 0.
    //    char *s = strtok(receivedData, " "); //allows string to be broken into tokens by " ".
    //    if (s[0] == handshake) {
    //      s = strtok(NULL, " ");
    //      if (s != NULL) throttle_read = atoi(s); //sets variable to received data and converts ASCII to integer if message is not empty
    //      s = strtok(NULL, " ");
    //      if (s != NULL) steering_read = atoi(s); //sets variable to received data and converts ASCII to integer if message is not empty
    //      steering_read = steering_write;
    //      throttle_read = throttle_write;
    //      car.writeToActuators(); //writes actuation variables to servo and ESC.
    //    } else {
    //      car.idling();
    //    }
  }
}

void Car::initArduino() {
  steering.attach(SERVO_PIN);
  throttle.attach(MOTOR_PIN);
  enable_Multiplexer.attach(MULTIPLEXER_PIN);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(receiver_state_pin, INPUT);
}

void Car::receiverState() {
  receiverStateVar = digitalRead(receiver_state_pin);
}

void Car::readReceiver() {
  steering_read = pulseIn (ch1, HIGH);
  throttle_read = pulseIn (ch2, HIGH);
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
  throttle.writeMicroseconds( (uint16_t) saturateMotor( throttle_write ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( steering_write ) );
}

void Car::idling() {
  throttle.writeMicroseconds( (uint16_t) MOTOR_NEUTRAL );
  steering.writeMicroseconds( (uint16_t) THETA_CENTER  );
}

void Car::Slave_Multiplexer() {
  enable_Multiplexer.writeMicroseconds((switchingValueMultiplexer + 200));
}

void Car::Master_Multiplexer() {
  enable_Multiplexer.writeMicroseconds((switchingValueMultiplexer - 200));
}
