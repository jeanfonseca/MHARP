//dist e movimento

// Enable debug prints

//#define MY_NODE_ID 2
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#include <SPI.h>
#include <MySensors.h>
#include <NewPing.h>

#define DIGITAL_INPUT_SENSOR 3
#define PRES_ID 11 
#define CHILD_ID 12
#define TRIGGER_PIN  6
#define ECHO_PIN     5
#define MAX_DISTANCE 300

MyMessage msg11(PRES_ID, V_TRIPPED);
MyMessage msgd(CHILD_ID, V_DISTANCE);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
int lastDist;
bool metric = true;
uint32_t SLEEP_TIME = 2000;




void setup()
{
    pinMode(DIGITAL_INPUT_SENSOR, INPUT);
    metric = getControllerConfig().isMetric;
}

void presentation()
{
    sendSketchInfo("Presenca e Distancia", "1.0");
    present(PRES_ID, S_MOTION);
    present(CHILD_ID, S_DISTANCE);
}

void loop()
{
    // presença
    bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;

    Serial.println(tripped);
    send(msg11.set(tripped?"1":"0"));

    //presença

    //dist
    int dist = metric?sonar.ping_cm():sonar.ping_in();
    Serial.print("Ping: ");
    Serial.print(dist);
    Serial.println(metric?" cm":" in");

    if (dist != lastDist) {
      send(msgd.set(dist));
      lastDist = dist;
  }  
    //dist 
    sleep(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), CHANGE, SLEEP_TIME);
    
}
