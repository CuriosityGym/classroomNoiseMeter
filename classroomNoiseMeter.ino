#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Servo.h>
#include <WiFiManager.h> 
#include <EEPROM.h>
#include <WiFiUDP.h>
IPAddress ipBroadCast(255, 255, 255, 255);
unsigned int udpRemotePort = 2392;      // local port to listen on
unsigned int UDPPortLocal = 4210;      // local port to listen on
const int UDP_PACKET_SIZE = 48;
char udpBuffer[ UDP_PACKET_SIZE];
int a =0;
WiFiUDP Udp;
//char packetBuffer[255]; //buffer to hold incoming packet
//char  replyBuffer[] = "acknowledged";       // a string to send back

// pins
#define MicPin A0 // used with analogRead mode only

// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024*2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.

// modes
//#define Use3.3 // use 3.3 voltage. the 5v voltage from usb is not regulated. this is much more stable.

#define VolumeGainFactorBits 0
double maxm = 0.0;
double minm = 100.00;
int soundLevel = 0;
int sensor = A0;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned long currentMillis = 0;
unsigned long currentMillis1 = 0;
bool noiseLevel_1 = false;
bool noiseLevel_2 = false;
bool sendAlert = false;
bool measure = true;
int count = 0;
Servo myservo;
int threshold = 70;
int pos;
int sound;
float dB;

int greenLed = 5;
int redLed = 2;

#define WLAN_SSID       "DT_LAB"
#define WLAN_PASS       "fthu@050318"


#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "siddheshCG"
#define AIO_KEY         "5e0217bf2f53446983193bc050996163"

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish classroom = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/classroom");

Adafruit_MQTT_Subscribe threshold_6A = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/threshold_6A");

void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(10);
  pinMode(greenLed , OUTPUT);
  pinMode(redLed , OUTPUT);
  digitalWrite(redLed, LOW);
  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
 /* Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }*/
  WiFiManager wifiManager;
    // wifiManager.resetSettings();
    wifiManager.autoConnect("Classroom 6A");
    Serial.println("connected to");
    Serial.println( WiFi.SSID().c_str());
    Serial.println(WiFi.psk().c_str());
    wifiManager.setConfigPortalTimeout(180);

  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  Udp.begin(UDPPortLocal);
   mqtt.subscribe(&threshold_6A);
   //myservo.attach(0);  // attaches the servo on GIO2 to the servo object
   //myservo.write(0);
  
}



void loop() 
{

   if(! mqtt.ping(3)) {
      // reconnect to adafruit io
      if(! mqtt.connected())
        mqtt.connect();
    } 
   
 // MQTT_connect();
  // what do we want to do?
  //MeasureAnalog();
  //if(measuredB == true){
  if(measure == true){
     soundLevel = MeasureVolume();
     a++;
  } 
  
  if ((millis() - currentMillis) > 60000){
     Serial.print(F("\nSending classroom val "));
     
     if(! classroom.publish(soundLevel))
       {
         Serial.println(F("Failed"));
       } 
     else 
       {
         Serial.println(F("OK!"));
       }
    currentMillis = millis();   
  }
  //delay(2000);
  /*
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) 
    {
      if (subscription == &threshold_6A) 
      {
        Serial.print(F("Got: "));
        Serial.println((char *)threshold_6A.lastread);
        String recvdThreshold = (char *)threshold_6A.lastread;
        threshold = recvdThreshold.toInt();   
      }
    }
*/
  if(soundLevel > threshold)
    {
      Serial.println("Noise level 1");   
      noiseLevel_1 = true;
      //myservo.write(180);              // tell servo to go to position in variable 'pos'
      //delay(15);                       // waits 15ms for the servo to reach the position
      currentMillis1 = millis();
    }
  else
    {    
      //myservo.write(0);              // tell servo to go to position in variable 'pos'
      //delay(15);                       // waits 15ms for the servo to reach the position
    }
 while(noiseLevel_1 == true && (millis() - currentMillis1) < 10000){
        if(! mqtt.ping(3)) {
      // reconnect to adafruit io
      if(! mqtt.connected())
        mqtt.connect();
    }
      soundLevel = MeasureVolume();
      Serial.println("level1");
      if(soundLevel > threshold){
        count++;
        
      }
      if(count >= 5){
       Serial.println("Noise level 2"); 
       noiseLevel_2 = true;
       noiseLevel_1 = false; 
       count = 0;
       currentMillis1 = millis();  
      }
      
  }
 while(noiseLevel_2 == true && (millis()-currentMillis1) < 10000){
         if(! mqtt.ping(3)) {
      // reconnect to adafruit io
      if(! mqtt.connected())
        mqtt.connect();
    }
      soundLevel = MeasureVolume();
      Serial.println("level2");
      if(soundLevel > threshold){
        count++;
        
      }
      if(count >= 4){
        Serial.println("Noise level 3");
       noiseLevel_2 = false;
       noiseLevel_1 = false;
       sendAlert = true; 
       count = 0;  
      }
 }
 if(sendAlert == true){
  Serial.println("Send message to display"); 
  String value = "1";
  value.toCharArray(udpBuffer,UDP_PACKET_SIZE);
  //Serial.println(udpBuffer); 
  Udp.beginPacket(ipBroadCast, udpRemotePort);
  Udp.write(udpBuffer, sizeof(udpBuffer));
  Udp.endPacket();
  noiseLevel_2 = false;
  noiseLevel_1 = false;
  sendAlert = false;
  count = 0;
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
 }
 else{
  if(a==8){
    String value1 = "0";
    value1.toCharArray(udpBuffer,UDP_PACKET_SIZE);
    //Serial.println(udpBuffer); 
    Udp.beginPacket(ipBroadCast, udpRemotePort);
    Udp.write(udpBuffer, sizeof(udpBuffer));
    Udp.endPacket();
    a =0;
    Serial.println("Messege Sent");
    digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  }
 }
}


void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(1000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}


// measure basic properties of the input signal
// determine if analog or digital, determine range and average.
void MeasureAnalog()
{
  long signalAvg = 0, signalMax = 0, signalMin = 1024, t0 = millis();
  for (int i = 0; i < MicSamples; i++)
  {

    int k = analogRead(MicPin);

    //signalMin = min(signalMin, k);
    //signalMax = max(signalMax, k);
    signalAvg += k;
  }
  signalAvg /= MicSamples;
  //sei();

  // print
  Serial.print("Time: " + String(millis() - t0));
  Serial.print(" Min: " + String(signalMin));
  Serial.print(" Max: " + String(signalMax));
  Serial.print(" Avg: " + String(signalAvg));
  Serial.print(" Span: " + String(signalMax - signalMin));
  Serial.print(", " + String(signalMax - signalAvg));
  Serial.print(", " + String(signalAvg - signalMin));
  Serial.println("");

  //lcd.clear();  //Clears the LCD screen and positions the cursor in the upper-left corner.   
  //lcd.setCursor(6, 0);                    // set the cursor to column 0, line 0
  //sprintf(lcdLineBuf, "%3d%% %3ddB", (int)soundVolRMS, (int)dB);
  //lcd.print(lcdLineBuf);
}

// calculate volume level of the signal and print to serial and LCD
int MeasureVolume()
{
  long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();
  //cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0; i < MicSamples; i++)
  {
//#ifdef ADCFlow
  //  while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
   // sbi(ADCSRA, ADIF); // restart adc
   // byte m = ADCL; // fetch adc data
   // byte j = ADCH;
   // int k = ((int)j << 8) | m; // form into an int
//#else
    int k = analogRead(MicPin);
//#endif
    int amp = abs(k - AmpMax);
    amp <<= VolumeGainFactorBits;
    //soundVolMax = max(soundVolMax, amp);
    if(soundVolMax > amp){
      soundVolMax = amp;
    }
    soundVolAvg += amp;
    soundVolRMS += ((long)amp*amp);
  }
  soundVolAvg /= MicSamples;
  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);
  //sei();

  float dB = 20.0*log10(soundVolRMSflt/AmpMax);
  double db = dB + 94 - 44 - 16;
  if(db > maxm){
    maxm = db;
  }
  if(db<minm){
    minm=db;
  }
  // convert from 0 to 100
  soundVolAvg = 100 * soundVolAvg / AmpMax; 
  soundVolMax = 100 * soundVolMax / AmpMax; 
  soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
  soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)
  int soundValues = abs(db);
  soundValues = map(soundValues, 0,36,0,120);
  
  // print
  Serial.print("Time: " + String(millis() - t0));
  Serial.print(" Amp: Max: " + String(soundVolMax));
  Serial.print("% Avg: " + String(soundVolAvg));
  Serial.print("% RMS: " + String(soundVolRMS));
  Serial.print(" Max: " + String(maxm));
  Serial.print(" Min: " + String(minm));
  Serial.print(" Value: " + String(soundValues));
  Serial.println("% dB: " + String(db,3));
  return soundValues;

}  
