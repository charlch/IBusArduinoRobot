#include <HBridge.h>
#include <FlySkyIBus.h>

const float DEAD_BAND_FRC = 0.1;
const int ledPin = LED_BUILTIN;
const int THROTTLE_CHANNEL = 1;
const int STEERING_CHANNEL = 0;
const int ARM_CHANNEL = 4;
const int REVERSE_CHANNEL = 5; //Set to -1 to not have reverse
const int ARMED = 1;
const int DISARMED = 0;
const int FORWARD_GEAR = 0;
const int REVERSE_GEAR = 1;
const long RADIO_TIMEOUT = 500;

HBridge left_motor(11, 10, DEAD_BAND_FRC);
HBridge right_motor(5, 6, DEAD_BAND_FRC);

int ledState = LOW;
unsigned long previousMillis = 0;
long interval = 1000; 
int armedState = DISARMED;
int gearDirection = FORWARD_GEAR;

 
void setup() {
  Serial.begin(115200);
  IBus.begin(Serial);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  IBus.loop();
  Serial.println(IBus.millisSinceUpdate());
  if (IBus.readChannel(ARM_CHANNEL) == 0 or IBus.millisSinceUpdate()>RADIO_TIMEOUT)
  {
    armedState=DISARMED;
    interval = 100;
  }
  else if (IBus.readChannel(ARM_CHANNEL) < 1200)
  {
    armedState = DISARMED;
    interval = 200;
  }
  else
  {
    armedState=ARMED;
    interval = 1000;
  }
  if (REVERSE_CHANNEL > -1){
    if (IBus.readChannel(REVERSE_CHANNEL) <1200){
      gearDirection = FORWARD_GEAR;
    }
    else
    {
      gearDirection = REVERSE_GEAR;
    }
  }
  
  updateLed();
  
  int steering = IBus.readChannel(STEERING_CHANNEL);
  int throttle = IBus.readChannel(THROTTLE_CHANNEL);

  if (armedState == DISARMED)
  {
    throttle = 1500;
    steering = 1500;
  }
  
  setControls(throttle, steering);
}

void updateLed() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}

void setControls(int throttle, int steering) {
  /*
   * (0,0) -> (0,0)
   * (1,0) -> (1,1)
   * (1,1) -> (1,0)
   * (1,-1)-> (0,1)
   * (0,1) ->(1,-1)
   * (0,-1) ->(-1,0)
   * 
   * Option 1:
   * t+s then bound -1<a<1
   * t-s then bound -1<b<1
   * 
   * Option 2:
   * Scale option 1 in rather than bounding it
   */
   float _throttle = normalise(throttle);
   float _steering = normalise(steering);

   float left_signal = bound(_throttle+_steering);
   float right_signal = bound(_throttle-_steering);

   if (gearDirection == REVERSE_GEAR) {
     float t = left_signal;
     left_signal = right_signal * -1.0;
     right_signal = t * -1.0;
   }

   Serial.print(left_signal);
   Serial.print(" ");
   Serial.println(right_signal);
   left_motor.set_signal(left_signal);
   right_motor.set_signal(right_signal);
}

float bound(float signal) {
  return min(max(signal, -1.0), 1.0);
}

float normalise(float signal) {
  return (signal-1500.0)/500.0;  
}
