// Enable debug prints
//#define MY_DEBUG
#define MY_NODE_ID 3


// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#include <MySensors.h>
#include <SPI.h>
#include <NewPing.h>

#define ID 19
#define OPEN 1
#define CLOSE 0
#define SOUND_PIN 5
#define CHILD_ID 10
#define BUTTON_PIN  3
#define SAMPLING_PERIOD 5000

int oldValue=-1;
unsigned long period_start = millis();
int state = -1;
int sensorPin = A0;
int sensorValue = 0;
int lastValue = 0;

MyMessage msgring(ID, V_TRIPPED);
MyMessage msg(CHILD_ID,V_TRIPPED);

void setup()
{
  //som
  pinMode(SOUND_PIN,INPUT) ; // input from the Sound Detection Module
  //som

  //vibra
  // Setup the button
  pinMode(BUTTON_PIN,INPUT);  
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN,HIGH);
  //vibra
}


int prev_state = -1;
int prev_state_length = 0;
void report_state(int state)
{
    Serial.print("reporting state: ");
    Serial.println(state); 
    send(msg.set(state));
  
}


void presentation()
{
    sendSketchInfo("Alerta Paciente", "2.0");
    present(ID, S_MOTION);
    present(CHILD_ID, S_MOTION);
}



void loop()
{
   //som
  sensorValue = analogRead(sensorPin);
  if (sensorValue > 100) {
  Serial.println(sensorValue);
  Serial.println("LOUD SOUND DETECTED! WARNING!");
  lastValue = 1;
  }
  send(msgring.set(lastValue));
  lastValue = 0;
  //som

  //VIBRA
  int value = digitalRead(BUTTON_PIN);

  if (value != 0 && state != 1) { // vibration detected for the first time during the period
    state = 1; 
  }

  if (millis() - period_start > SAMPLING_PERIOD) {
    period_start = millis();
    Serial.println(state);
    if (state != prev_state) {
      if (state == 1 && prev_state_length > 4) { //a vibration period after long period of quiet means a cycle started
          report_state(state);
        prev_state_length = 1;
      
      } 
    } else if (state == 0 && prev_state_length > 2) { // a long period of quiet means cycle ended
      if (prev_state_length == 3)
        report_state(state); // report only once 
    }
    
    if (state != prev_state)
      prev_state_length = 1;
    else
      prev_state_length++;
  prev_state = state;
    state = 0;
    
  }
  
 
  delay(1000);
  //VIBRA
  
}
