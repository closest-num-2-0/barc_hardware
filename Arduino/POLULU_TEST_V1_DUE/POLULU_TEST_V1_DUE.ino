// MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading and writing eight RC Channels using Arduino Due interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//


//global variable
int throttle_read = 1500;
int steering_read = 1500;
int throttle_write = 1500;
int steering_write = 1500;
int receiverStateVar = LOW;

#include "Servo.h"

// Assign your channel in pins
#define CHANNEL1_IN_PIN 2
#define CHANNEL2_IN_PIN 3
#define RECEIVER_STATE_PIN 30

// Assign your channel out pins
#define CHANNEL1_OUT_PIN 10
#define CHANNEL2_OUT_PIN 11

Servo servoChannel1;
Servo servoChannel2;


#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2

volatile uint32_t bUpdateFlagsShared;

volatile uint32_t unChannel1InShared;
volatile uint32_t unChannel2InShared;

void setup()
{
  Serial.begin(115200);

  servoChannel1.attach(CHANNEL1_OUT_PIN);
  servoChannel2.attach(CHANNEL2_OUT_PIN);

  pinMode(RECEIVER_STATE_PIN, INPUT);

  attachInterrupt(CHANNEL1_IN_PIN, calcChannel1,CHANGE);
  attachInterrupt(CHANNEL2_IN_PIN, calcChannel2,CHANGE);

}

void loop()
{
  receiverStateVar = digitalRead(RECEIVER_STATE_PIN);
  
  static uint32_t unChannel1In;
  static uint32_t unChannel2In;
 
  static uint32_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts();
    
    bUpdateFlags = bUpdateFlagsShared;
  
    if(bUpdateFlags & CHANNEL1_FLAG)
    {
      unChannel1In = unChannel1InShared;
    }
  
    if(bUpdateFlags & CHANNEL2_FLAG)
    {
      unChannel2In = unChannel2InShared;
    }
    bUpdateFlagsShared = 0;
  
    interrupts();
  }
 
  if(bUpdateFlags & CHANNEL1_FLAG)
  {
      Serial.println();
      Serial.print(unChannel1In);
      Serial.print(",");
  }

  if(bUpdateFlags & CHANNEL2_FLAG)
  {
      Serial.print(unChannel2In);
  }

  
  throttle_read = unChannel2In;
  steering_read = unChannel1In;

  
  if (receiverStateVar == LOW) {
    throttle_write = throttle_read;
    steering_write = steering_read;
    
    servoChannel2.writeMicroseconds(throttle_write);
    servoChannel1.writeMicroseconds(steering_write);
  } else if (receiverStateVar == HIGH) {
    throttle_write = 1550;
    steering_write = 1900;

    servoChannel2.writeMicroseconds(throttle_write);
    servoChannel1.writeMicroseconds(steering_write);

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
  
  bUpdateFlags = 0;
}

void calcChannel1()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL1_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel1InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL1_FLAG;
  }
}

void calcChannel2()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL2_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel2InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL2_FLAG;
  }
}
