#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiManager.h> 
#include <EEPROM.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    2

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 144

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned int UDPPort = 2392;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back
WiFiUDP Udp;

//const char* ssid = "No Honking";
//const char* password = "NoHonking";
String msg="";

void setup() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(40); // Set BRIGHTNESS to about 1/5 (max = 255)
    Serial.begin(115200);
  /*WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Wait for WiFi");
WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Wait for WiFi");

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());*/
    WiFiManager wifiManager;
    // wifiManager.resetSettings();
    wifiManager.autoConnect("Classroom 6A Led");
    Serial.println("connected to");
    Serial.println( WiFi.SSID().c_str());
    Serial.println(WiFi.psk().c_str());
    wifiManager.setConfigPortalTimeout(180);

  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.println();
  Udp.begin(UDPPort);
  Serial.println();
    Serial.println("Started ap. Local ip: " + WiFi.localIP().toString());

 
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
   /* Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());*/

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    msg = packetBuffer;
    Serial.println("Contents:");
    Serial.println(msg);
    // send a reply, to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
   // Udp.endPacket();
 
   if (msg == "1") {
       for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
         strip.setPixelColor(i, 255,0,0);         //  Set pixel's color (in RAM)
         strip.show();                          //  Update strip to match
         delay(10);                           //  Pause for a moment   
      }
      delay(10000); 
    }
   if (msg == "0") {
       for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
          strip.setPixelColor(i, 0,255,0);         //  Set pixel's color (in RAM)
          strip.show();                          //  Update strip to match
          delay(10);                           //  Pause for a moment
         }
         
      }
  }

}
