#include <HBridge.h>
#include <FlySkyIBus.h>

// Channel selection
const int THROTTLE_CHANNEL = 1;
const int STEERING_CHANNEL = 0;
const int ARM_CHANNEL = 4;
const int REVERSE_CHANNEL = 5; //Set to -1 to not have reverse
const int MAX_SPEED_CHANNEL = 6; //Set to -1 to disable 

// Tuning params
const float STEERING_EXPO = 2.0;

// Radio settings
const float DEAD_BAND_FRC = 0.1;
const long RADIO_TIMEOUT = 500;

 // Enum style constants
const int ARMED = 1;
const int DISARMED = 0;
const int FORWARD_GEAR = 0;
const int REVERSE_GEAR = 1;
const int SLOW_LED = 1000;
const int MED_LED = 2000;
const int FAST_LED = 100;

// Pin setup
const int ledPin = LED_BUILTIN;
HBridge left_motor(11, 10);
HBridge right_motor(5, 6);

// Initial state
int armedState = DISARMED;
int gearDirection = FORWARD_GEAR;
int ledState = LOW;
unsigned long previousMillis = 0;
long led_flash_interval = SLOW_LED;


void setup() {
    Serial.begin(115200);
    IBus.begin(Serial);
    pinMode(ledPin, OUTPUT);
    left_motor.set_dead_band_fct(DEAD_BAND_FRC);
    right_motor.set_dead_band_fct(DEAD_BAND_FRC);
}

void loop() {
    IBus.loop();

    if (IBus.readChannel(ARM_CHANNEL) == 0 or IBus.millisSinceUpdate() > RADIO_TIMEOUT) {
        armedState = DISARMED;
        led_flash_interval = FAST_LED;
    } else if (IBus.readChannel(ARM_CHANNEL) < 1200) {
        armedState = DISARMED;
        led_flash_interval = MED_LED;
    } else {
        armedState = ARMED;
        led_flash_interval = SLOW_LED;
    }
    if (REVERSE_CHANNEL > -1) {
        if (IBus.readChannel(REVERSE_CHANNEL) < 1200) {
            gearDirection = FORWARD_GEAR;
        } else {
            gearDirection = REVERSE_GEAR;
        }
    }

    updateLed();

    int steering = IBus.readChannel(STEERING_CHANNEL);
    int throttle = IBus.readChannel(THROTTLE_CHANNEL);

    if (armedState == DISARMED) {
        throttle = 1500;
        steering = 1500;
    }

    setControls(throttle, steering);
}

void updateLed() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= led_flash_interval) {
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
     * t+s then bound -1<a<1
     * t-s then bound -1<b<1 
     */
    float _throttle = normalise(throttle);
    float _steering = normalise(steering);

    // Expo
    _steering = sign(_steering) * pow(abs(_steering), STEERING_EXPO);

    float left_signal = bound(_throttle + _steering);
    float right_signal = bound(_throttle - _steering);

    if (gearDirection == REVERSE_GEAR) {
        float t = left_signal;
        left_signal = right_signal * -1.0;
        right_signal = t * -1.0;
    }

    if (MAX_SPEED_CHANNEL > -1) {
        float max_speed = mapfloat(IBus.readChannel(MAX_SPEED_CHANNEL), 1000.0, 2000.0, 1.0, 0.0);
        left_signal *= max_speed;
        right_signal *= max_speed;
    }

    left_motor.set_signal(left_signal);
    right_motor.set_signal(right_signal);
}

float sign(float x) {
    return (x > 0.0) - (x < 0.0);
}

float bound(float signal) {
    return constrain(signal, -1.0, 1.0);
}

float normalise(float signal) {
    return mapfloat(signal, 1000.0, 2000.0, -1.0, 1.0);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
