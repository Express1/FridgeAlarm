// Low power fridge door open alarm with battery low indicator.
// Arduino pro mini, 3.3V, 8Mhz clock. Remove the low voltage regulator and power LED from the circuit.
// power source 3XAA batteries - connect to GND and VCC

// LDR 5539 photoresistor connected between "INTERRUPT_PIN" and GND , 4.7 GOhm resitor to Vcc

// Piezo buzzer connected between "MY_pin1" and "MY_pin2"

// standby current at 3.3v is about 8-9uA at 3.5v

// install Arduino DeepSleepScheduler Library
// https://github.com/PRosenb/DeepSleepScheduler

// (c) May 2018 Radu C

/*
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/ 


#define INTERRUPT_PIN 2  // pin where the sensor is mounted to the ground
#define LED_PIN 13 // internal LED
#define MY_pin1 12 // buzzer pin 1
#define MY_pin2 11 // buzzer pin 2

// debug, turn off for production
#define debug // Serial port debug 115200baud
// show when the cpu is on on pin 13 where we have a led to gnd for debugging
#define AWAKE_INDICATION_PIN LED_PIN


#include <DeepSleepScheduler.h>
#define SLEEP_MODE SLEEP_MODE_PWR_DOWN
// delay deep sleep by 100 milli seconds, if is shorter the sound will be distorted.
#define DEEP_SLEEP_DELAY 100 

#define MYT1 30 //30 wait time
#define MYT2 60 //60 alarm time
#define MYT3 10 //10 low voltage alarm time
#define MYVCC 2500  // 2500 = 2.5V low voltage alarm treshold @8MHz clock

unsigned short t1;  // initial wait interval 30 sec
unsigned short t2;  // alarm time 60 sec
unsigned short t3;  // low battery alarm time 10 sec
signed long vcc;


////////////////////////////////////////////////////////////////////

void playBatt()
{
  playTone(1000,150);
  playTone(500,150);
}

void playAlarm()
{
  if (t2 > MYT2/2 ) {
    playTone(500,300);
  }
  else
  {
    playTone(600,150);
    playTone(500,150);
  }
}

void playTone(int tone1, int duration) { // tone freq is f=1/2*tone, duration in ms
    
  //tone(MY_pin1, tone1, duration);
   
  for (long i = 0; i < duration *1000L; i += tone1 * 2) {
    //noInterrupts();
    digitalWrite(MY_pin1, HIGH);
    digitalWrite(MY_pin2, LOW);
    delayMicroseconds(tone1);
    digitalWrite(MY_pin1, LOW);
    digitalWrite(MY_pin2, HIGH);
    delayMicroseconds(tone1);
    //interrupts(); 
  }
  

}

void alarmBattlow() {
  t3--;
#if defined(debug)
  Serial.print(F("AlarmBattLow:"));
  Serial.println(t3);
  delay(10);
#endif
  scheduler.schedule(playBatt);
  scheduler.schedule(playBatt);
  if (digitalRead(INTERRUPT_PIN) == 0 && t3 > 0) {
    
    scheduler.scheduleDelayed(alarmBattlow, 1000);
  }
  else
  {
    inton(); // we are done with the alarm, time is out or light is off, back to wait for interrupt
  }
}
void alarm() {
  t2--;
#if defined(debug)
  Serial.print(F("Alarm:"));
  Serial.println(t2);
  delay(10);
#endif
    scheduler.schedule(playAlarm);
  if (digitalRead(INTERRUPT_PIN) == 0 && t2 > 0) {

    scheduler.scheduleDelayed(alarm, 1000);
  }
  else
  {
    inton(); // we are done with the alarm, time is out or light is off, back to wait for interrupt
  }
}

void justwait() {
  t3 = 0;
  if (t1 == MYT1) {     // check Battery voltage
    vcc = readVcc();
    if (vcc < MYVCC) {
      // undervoltage
#if defined(debug)
      Serial.print(F("Vcc is low:"));
      Serial.println(vcc);
      delay(10);
#endif
      // sound low battery alarm
      t3 = MYT3;  // 10 sec
      scheduler.scheduleDelayed(alarmBattlow, 1000);
    }
  }
  // we continue only if t3 is zero, otherwise is a batt low alarm
  if (t3 == 0) {
    t1--; // batt is ok, we wait for t1 sec and then alarm
#if defined(debug)
    Serial.print(F("Just wait:"));
    Serial.println(t1);
    delay(10);
#endif
    if (digitalRead(INTERRUPT_PIN) == 0 && t1 > 0) {
      scheduler.scheduleDelayed(justwait, 1000);
    }
    else {
      if (digitalRead(INTERRUPT_PIN) == 1) {
        inton(); // the light is off, back to initial state, wait for interrupt
      }
      else {  // we continue with the alarm now
        t2 = MYT2;
        scheduler.scheduleDelayed(alarm, 1000);
      }
    }
  }
}

void isrInterruptPin() {
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  t1 = MYT1;  //reset time interval
  scheduler.schedule(justwait);
}

void inton() {
  EIFR |= 0x01; // Avoid spurious interrupts. I had to clear the interrupt using the EIFR |= 0x01 immediately after I attach the interrupt or else there were false triggers. I was using FALLING condition.
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isrInterruptPin, FALLING);
  EIFR |= 0x01;
  digitalWrite(MY_pin1, LOW); // buxxer pin low, both of them.
  digitalWrite(MY_pin2, LOW);
}

// See: http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
signed long readVcc() {
  signed long resultVcc;
  //float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10);                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                 // Convert
  while (bit_is_set(ADCSRA, ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH << 8;
  resultVcc = 1126400L / resultVcc;    // Back-calculate AVcc in mV
  //resultVccFloat = (float) resultVcc / 1000.0; // Convert to Float
  return resultVcc;
}

void setup() {
  //pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(INTERRUPT_PIN, INPUT);  // internal pullup resitor value is too low
  pinMode(MY_pin1, OUTPUT);
  pinMode(MY_pin2, OUTPUT);
  Serial.begin(115200);
  inton();
#if defined(debug)
  Serial.print(F("Start:"));
  Serial.println();
  delay(10);
#endif
  // waiting for interrupt
}

void loop() {
  scheduler.execute();
}

