// Enable debug prints
#define MY_DEBUG
#define MY_NODE_ID 2

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

#define DHT_DATA_PIN 3
#define SENSOR_TEMP_OFFSET 0

// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 2000;
unsigned long SLEEP_TIME = 100;


#include <MyConfig.h>
#include <MySensors.h>
#include <SPI.h>
#include <DHT.h>
#include <BH1750.h>
#include <Wire.h>



#define CHILD_ID_HUM 5
#define CHILD_ID_TEMP 6
#define CHILD_ID_LIGHT 7
#define CHILD_ID_MQ 8

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgl(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msg(CHILD_ID_MQ, V_LEVEL);


BH1750 lightSensor;
DHT dht;
uint16_t lastlux;
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;


//MQ2

/************************Hardware Related Macros************************************/
#define     MQ_SENSOR_ANALOG_PIN         (0)  //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in
//normal operation
/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

float Ro = 10000.0;    // this has to be tuned 10K Ohm
int val = 0;           // variable to store the value coming from the sensor
float valMQ =0.0;
float lastMQ =0.0;
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float           SmokeCurve[3] = {2.3,0.53,-0.44};   //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2:(lg10000,-0.22)


//MQ2

void presentation()  
{ 
  sendSketchInfo("Temperatura e Ambiente", "3");

  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_MQ, S_AIR_QUALITY);

  metric = getControllerConfig().isMetric;
}


void setup()
{
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }

  sleep(dht.getMinimumSamplingPeriod());

  lightSensor.begin();

  Ro = MQCalibration(
    MQ_SENSOR_ANALOG_PIN);         //Calibrating the sensor. Please make sure the sensor is in clean air
  
}


void loop()      
{ 
	//temp
  dht.readSensor(true);


  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } 
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    nNoUpdatesTemp = 0;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));

    #ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temperature);
    #endif


  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } 
    lastHum = humidity;
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));

    #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
    #endif

	//temp
     
	//lux
  uint16_t lux = lightSensor.readLightLevel();
  Serial.print("LUZ:");
  Serial.println(lux);
  if (lux != lastlux) {
      send(msgl.set(lux));
      lastlux = lux;
  }
	//lux

    uint16_t valMQ = MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_CO);
    Serial.println(val);

    Serial.print("LPG:");
    Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_LPG) );
    Serial.print( "ppm" );
    Serial.print("    ");
    Serial.print("CO:");
    Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_CO) );
    Serial.print( "ppm" );
    Serial.print("    ");
    Serial.print("SMOKE:");
    Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_SMOKE) );
    Serial.print( "ppm" );
    Serial.print("\n");

    
        send(msg.set((int16_t)ceil(valMQ)));
        lastMQ = ceil(valMQ);
    
   

  sleep(UPDATE_INTERVAL); 
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
    return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
    int i;
    float val=0;

    for (i=0; i<CALIBARAION_SAMPLE_TIMES; i++) {          //take multiple samples
        val += MQResistanceCalculation(analogRead(mq_pin));
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

    val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro
    //according to the chart in the datasheet

    return val;
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
    int i;
    float rs=0;

    for (i=0; i<READ_SAMPLE_TIMES; i++) {
        rs += MQResistanceCalculation(analogRead(mq_pin));
        delay(READ_SAMPLE_INTERVAL);
    }

    rs = rs/READ_SAMPLE_TIMES;

    return rs;
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
    if ( gas_id == GAS_LPG ) {
        return MQGetPercentage(rs_ro_ratio,LPGCurve);
    } else if ( gas_id == GAS_CO ) {
        return MQGetPercentage(rs_ro_ratio,COCurve);
    } else if ( gas_id == GAS_SMOKE ) {
        return MQGetPercentage(rs_ro_ratio,SmokeCurve);
    }

    return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
    return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


