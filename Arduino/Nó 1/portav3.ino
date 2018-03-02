
/* DESCRIPTION
*
*Componentes c/ child ids diferentes, mudei pino do porta
*/

// Enable debug prints to serial monitor
#define MY_NODE_ID 0
#define MY_DEBUG


// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_GATEWAY_SERIAL

// Define a lower baud rate for Arduino's running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Inverses the behavior of leds
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
//#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

#include <Bounce2.h>
#include <MyConfig.h>
#include <MySensors.h>
#include <NewPing.h>

#define DOOR_ID 1
#define BUTTON_PIN  3
#define CHILD_ID 2
#define TRIGGER_PIN  6
#define ECHO_PIN     5
#define MAX_DISTANCE 300
unsigned long SLEEP_TIME = 2000;


MyMessage msg1(DOOR_ID,V_TRIPPED);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
MyMessage msg2(CHILD_ID, V_DISTANCE);
int lastDist;
bool metric = true;
Bounce debouncer = Bounce(); 
int oldValue=-1;


void setup()  
{  
  // porta
  pinMode(BUTTON_PIN,INPUT);
  digitalWrite(BUTTON_PIN,HIGH);
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);
  //porta

  //distancia
  metric = getControllerConfig().isMetric;
  //distancia
}

void presentation() {
  sendSketchInfo("Porta, Distancia e Gateway", "3.0");
  
  present(DOOR_ID, S_DOOR);
  present(CHILD_ID, S_DISTANCE);
}




void loop() 
{
  //porta
  debouncer.update();
  int value = debouncer.read();

  if (value != oldValue) {
     send(msg1.set(value==HIGH ? 1 : 0));
     oldValue = value;
     if (value == 1) {
      Serial.println("Porta foi aberta");
      }
      if (value == 0) {
      Serial.println("Porta foi fechada");
      } 
  }
  //porta

  //distancia
  int distance = metric?sonar.ping_cm():sonar.ping_in();
  Serial.print("Ping: ");
  Serial.print(distance);
  Serial.println(metric?" cm":" in");

  if (distance != lastDist) {
      send(msg2.set(distance));
      lastDist = distance;
  }

  sleep(SLEEP_TIME);
  //distancia
}
