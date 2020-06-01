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
    void Master_Multiplexer();
    void Slave_Multiplexer();

  private:

    const int SERVO_PIN = 2;
    const int MOTOR_PIN = 3;
    const int ch1 = 36;
    const int ch2 = 38;
    const int MULTIPLEXER_PIN = 10;
    const int receiver_state_pin = 30;

    Servo throttle;
    Servo steering;
    Servo enable_Multiplexer;

    const int switchingValueMultiplexer = 1700;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;

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
    car.readReceiver();
    Serial.print(throttleIn);
    Serial.print(" ");
    Serial.print(steeringIn);
    Serial.println();
  }

  else if (receiverStateVar == HIGH) {
    car.Slave_Multiplexer(); //switches to SLAVE on POLULU multiplexer
    throttleIn = 1600;
    steeringIn = 1200;
    car.writeToActuators();
  }
}

void Car::initArduino() {
  throttle.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
  enable_Multiplexer.attach(MULTIPLEXER_PIN);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(receiver_state_pin, INPUT);
}

void Car::receiverState() {
  receiverStateVar = digitalRead(receiver_state_pin);
}

void Car::readReceiver() {
  steeringIn = pulseIn (ch1, HIGH);
  throttleIn = pulseIn (ch2, HIGH);
}

void Car::writeToActuators() {
  throttle.writeMicroseconds( (uint16_t) throttleIn );
  steering.writeMicroseconds( (uint16_t) steeringIn );
}

void Car::idling() {
  throttle.writeMicroseconds( (uint16_t) MOTOR_NEUTRAL );
  steering.writeMicroseconds( (uint16_t) THETA_CENTER  );
}

void Car::Slave_Multiplexer() {
  enable_Multiplexer.writeMicroseconds((uint16_t) (switchingValueMultiplexer + 200));
}

void Car::Master_Multiplexer() {
  enable_Multiplexer.writeMicroseconds((uint16_t) (switchingValueMultiplexer - 200));
}
