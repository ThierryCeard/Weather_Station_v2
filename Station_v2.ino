// Code simplification attempts

// Request JSONified
// Timer set up 
// Implement WPS

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <ArduinoJson.h>
#include "AS5600.h"
#include <DHTesp.h>
#include <BMP180I2C.h>

#define I2C_ADDRESS 0x77

//create an BMP180 object using the I2C interface
BMP180I2C bmp(I2C_ADDRESS);

// Cablage sur UART
// GROUND Marron /  TX BLEU /  RX ORANGE

// Pour Debug decommenter les lignes suivantes
#define DEBUG_TRACE 
#define WIFI_CONNECTION 
#define DEBUG_HTTP 
#define DEBUG_HTTP2
//#define TRACE_DHT 
//#define DEBUG_WIND_DIR 
//#define DEBUG_WATER
//#define TIMER
//#define WIFI_TRACE
//#define BMP180  --> Bug on Serial print


// Parametres
#define SAMPLE_RETRY  1e3               // Temps d'attente avant renvoi sur requete invalide
#define COUNT_RETRY   2                 // Nombre de tentative avant de passer a la suite
#define SLEEPING_TIME 300               // in s
#define WATER_BOUNCING_TIMEMS 500          // Time in ms to debounced 2 water drop
#define SCREEN_LIGHT_TIME 10           // in s time the display is on
#define SSID_NETWORK  "ARRIS-CEARD"
#define PASS_WIFI     "49681154"        // "49681154"          
#define SERIAL_RATE   115200
#define DHT_RETRY     5
#define URL           "http://192.168.1.19/Meteo/upload_data_JSON.php" // Web Service URL to manage data
//#define URL           "http://192.168.1.19/Meteo/hello.php" // Web Service URL to manage data
 
#define HOSTNAME      "OUTER-STATION-456098"

// Les pins de branchement 
#define DHT_PIN 2                      //GPIO2
#define I2C_SDA 14                    //I2C DATA
#define I2C_SCL 13                    //I2C CLOCK
#define PIN_WIND_SPEED 0
#define WATER_PIN 5 // 4 For debug should be 5

// Initialization 

//Adafruit_BMP085 bmp;

AS5600 as5600;   
DHTesp dht;

String        MAC;
int           Retry=0;
int           Counter=0;
int           Water=0;
int           FallTime=0;
bool          goodToSend=true;
bool          isWPSConnected=true;
int secondElapsed=0; // Time till the last request is sent
bool isWaterDebounced=false;
long lastWaterInterrupt=0;
const String compile_date = __DATE__ " " __TIME__;
const String compile_version = "2.1-git";
const String sketch_name= __FILE__;

//JSON Object
DynamicJsonDocument object(2048);

void setup()
{
// Interruption setup for Wind and Water detection
// ------------------------------------------------
// Wind Detection 
//pinMode(SCREEN_INPUT, INPUT);
pinMode(WATER_PIN, INPUT); 
//pinMode(WIND_DIRECTION_RESET, INPUT);

//attachInterrupt(digitalPinToInterrupt(SCREEN_INPUT), screenInterrupt, FALLING);
// attachInterrupt(digitalPinToInterrupt(WIND_DIRECTION_PIN2), handleInterruptPin2, CHANGE);
attachInterrupt(digitalPinToInterrupt(WATER_PIN), waterInterrupt,CHANGE);

 // Wind Meter
pinMode(PIN_WIND_SPEED,INPUT);

// Timer set up
// ------------
timer1_isr_init();
timer1_attachInterrupt(drumBeat);
timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP);
timer1_write(312500); // Timer on DIV256 = Frequence : 312.5KHz (second drum beat)

// Pour eviter les pb d'initialisation suite a l'appel de courant au demarrage
delay(500);
 
// Ouverture de la connexion serie pour tracking
#ifdef DEBUG_TRACE 
Serial.begin(SERIAL_RATE);
Serial.println();
Serial.println("-------------------------------------------------------------------------------------------------------------------");
Serial.print(" Sketch name  : ");Serial.println(sketch_name); 
Serial.print(" Compile date : ");Serial.print(compile_date);
Serial.print("\t Version : ");Serial.println(compile_version);
Serial.print("AS5600_LIB_VERSION: ");  Serial.println(AS5600_LIB_VERSION);
Serial.println("-------------------------------------------------------------------------------------------------------------------");
#endif

#ifdef TIMER
Serial.print("Sleeping Time : ");
Serial.println(SLEEPING_TIME);
#endif

// On cree le DTH connecte au pin 2
dht.setup(DHT_PIN, DHTesp::DHT11); // Connect DHT sensor to GPIO 2

// Initiate connexion on pin SDA  and SCL 
Wire.begin(I2C_SDA,I2C_SCL);
delay(500);

//Serial.println("BMP Connection :");

if (!bmp.begin()) {
  #ifdef BMP180
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
		while (1);  
  #endif
}

// Delai au boot pour debug et reprise de main
#ifdef DEBUG_TRACE
Serial.println("Debut delai attente 2s");
delay(2000);
#endif

  as5600.begin(4);  //  set direction pin.
  delay(2000);
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect AS5600 : ");
  Serial.println(b);
}

void loop() {  
if (goodToSend==true) {
  // object clean up to avoid memory leak
  object.clear();
  
  #ifdef DEBUG_TRACE 
  Serial.println("Entering collecting data process");
  #endif
    
  // Get Temp and humidity populated in JSON
  getDHTMeasure();

  // Get Temp and pressure populated in JSON
  getBMP180Measure();

  // Get Wind speed and direction and Rain water populated in JSON
  getWindAndWater();

  // Connect to WIFI via PSW or WPS
  // WiFi.printDiag(Serial);
  bool isWifiConnected = connectWifi();
  if (isWifiConnected) {
    #ifdef WIFI_CONNECTION
    Serial.print("Wifi Connection Status : ");
    Serial.println(isWifiConnected);
    delay(100);
    #endif
    // Send HTTP Request to the server 
    goodToSend=!sendHTTPRequest();
    
  }

  
      //  Sleep mode entering :
    #ifdef WIFI_CONNECTION
    Serial.println("Entering Sleep Mode");
    #endif
    
    WiFi.mode( WIFI_OFF );
    WiFi.forceSleepBegin();
    delay(10);
    
    
   /*Serial.println("Deep Sleep Begin");
   WiFi.disconnect( true );
   delay( 1 );

    // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
    ESP.deepSleep( SLEEPING_TIME/2*1000000, RF_DEFAULT );*/
  }
}

  ICACHE_RAM_ATTR void waterInterrupt() { // Possible issue to fix during millis rollover
  if (((millis()-lastWaterInterrupt)> WATER_BOUNCING_TIMEMS) || (millis()-lastWaterInterrupt)<0) {
    Water++;
    lastWaterInterrupt=millis();  
  }
      #ifdef DEBUG_WATER
       Serial.print("Water : ");Serial.println(Water);
       #endif
 }

  ICACHE_RAM_ATTR void drumBeat() {
    // Cadence every seconds
    
  if (++secondElapsed>=SLEEPING_TIME) {
    goodToSend=true;
    secondElapsed=secondElapsed-SLEEPING_TIME;
    }

#ifdef TIMER
    Serial.print("Time : "); Serial.println(secondElapsed);
#endif
}
  
void getDHTMeasure() {
  // DTH Sensor
  // Get Humidty - Temp in Celsius and Heat Index and populate the JSON Object with values 
  int dhtRetry=0;
  delay(dht.getMinimumSamplingPeriod()); //Wait for DHT to initialize
  while ((dht.getStatusString() !="OK") && (dhtRetry<DHT_RETRY)) {
    delay(100);
    #ifdef TRACE_DHT
    Serial.print("DHT Retry : ");Serial.println(dhtRetry);
    #endif 
    dhtRetry++;
    }
  object["Humidity"] = dht.getHumidity();
  object["DHT_Temp"] = dht.getTemperature();
  object["HeatIndex"]= dht.computeHeatIndex(dht.getTemperature(), dht.getHumidity(), false);
  
  #ifdef TRACE_DHT
  // Trace out the DHT Values
  Serial.println("DHT Traces :");
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(dht.getHumidity(), 1);
  Serial.print("\t\t");
  Serial.print(dht.getTemperature(), 1);
  Serial.print("\t\t");
  Serial.println(dht.computeHeatIndex(dht.getTemperature(), dht.getHumidity(), false), 1);
  #endif
 }

 void getBMP180Measure() {
  #ifdef BMP180
  Serial.print("BMP Traces :");
  #endif
  // BMP180  Sensor
  object["BMP_Temperature"]=bmp.getTemperature();
  delay(100) ;
  object["BMP_Pressure"]=bmp.getPressure();
 
  delay(100);
  #ifdef BMP180
  Serial.println("BMP Sensor ");
  Serial.print("BMP Pressure :");
  Serial.print(object["BMP_Pressure"]);
  Serial.print("\t");
  Serial.print("BMP Temperature :");
  Serial.println(object["BMP_Temperature"]);
  #endif
  if (!bmp.measureTemperature())
	{
		Serial.println("could not start temperature measurement, is a measurement already running?");
		return;
	}

	//wait for the measurement to finish. proceed as soon as hasValue() returned true. 
	do
	{
		delay(100);
	} while (!bmp.hasValue());
}

void getWindAndWater() {
  // Wind sensors
  int rawWind=analogRead(PIN_WIND_SPEED);
  
  int winDir =as5600.rawAngle()* AS5600_RAW_TO_DEGREES;
  #ifdef DEBUG_WIND_DIR  
  Serial.print("Wind Direction  : ");   Serial.print(Counter);
  Serial.print("\t WindSpeed : "); Serial.print(rawWind);
  Serial.print("\t Water :"); Serial.println(Water);
  Serial.print("\t WinDirection AS5600 :"); Serial.println(winDir);
  #endif
  object["Wind_Dir"]=Counter;
  object["Wind_Speed"]=rawWind;
  object["Water"]=Water;
  object["Wind_Dir2"]=winDir;
}



bool sendHTTPRequest() {
 
  // send the JSON formatted data off all the sensor to the HTTP Web service
  // Return true if http 200 is recieved 
  WiFiClient wificlient;
  HTTPClient http;  
 
  http.begin(wificlient,URL);      
  http.addHeader("Content-Type", "application/json"); 
  int httpCode=0;  
  String json;
  
  serializeJson(object,json);
  
  
  while ((httpCode!=200)&&(Retry<=COUNT_RETRY)) {
   httpCode = http.POST(json);   //Send the request
   

   #ifdef DEBUG_HTTP2
   String payload = http.getString();  //Get the response payload 
   Serial.print("POST : ");  Serial.println(json);
   Serial.print("Http code : "); Serial.println(httpCode);   //Print HTTP return code
   Serial.print("Payload   : "); Serial.println(payload);    //Print request response payload
   Serial.print("Retry :");Serial.println(Retry);
   #endif
   #ifdef DEBUG_TRACE
   Serial.print("Http code : "); Serial.println(httpCode);   //Print HTTP return code
   #endif
   delay(SAMPLE_RETRY);
   Retry++;
   }
  Retry=0;
  http.end();  //Close connection
  wificlient.stop();
  
  
  delay(500);
  if (httpCode!=200) {
    Water=0; // Water reset to 0 between each request
    #ifdef DEBUG_HTTP
    Serial.println("Exiting with false");  
    #endif
    
    return false;  
    }

    #ifdef DEBUG_HTTP
    Serial.println("Exiting with true");
    #endif  
    
    return true;
    
}

bool connectWifi() {
  #ifdef WIFI_CONNECTION
  Serial.println("Entering connectWifi");
  delay(100);
  #endif

  //WiFi.forceSleepWake();
  //delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  delay(100);
  
   #ifdef WIFI_CONNECTION2
    WiFi.printDiag(Serial);
    delay(100);
    #endif
  int i=0;
  for (i=1;i<=3; i++) {
    switch(i) {
    case 1:
    // Previous connection parameter attempt
    WiFi.begin(SSID_NETWORK,PASS_WIFI);
    delay(500);
    break;
    // PWD and SSID connection parameter attempt
    case 2:
    WiFi.begin(SSID_NETWORK,PASS_WIFI);
    delay(500);
    break;
    // WPS Connection
    case 3:
    WiFi.disconnect();
    #ifdef WIFI_CONNECTION
    Serial.println(") Please press WPS button on your router.\n Press GPIO5 to continue...");
    #endif
    while(digitalRead(5)) {yield(); }
    bool wpsSuccess = WiFi.beginWPSConfig();
    delay(500);
    if(!wpsSuccess) {
        return false;}
    break;
      }
    WiFi.hostname(HOSTNAME);
    object["MAC"]=WiFi.macAddress();
    object["HOSTNAME"]=WiFi.hostname();
    
    int j=0;
    while (j<30) {
      delay(500);
      j++;
      if (WiFi.status() == WL_CONNECTED) {     
        #ifdef WIFI_CONNECTION
        Serial.print("Connected with option :");Serial.println(i);
        #endif
        return true;
		
        }  
      }
  }
  return false;
}
