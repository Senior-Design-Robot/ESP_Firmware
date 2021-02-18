#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

#define SERVO_PIN D7

const char* ssid = "KATY-CORSAIR 0494";        // Enter SSID here
const char* password = "0Y46u%02";  //Enter Password here

WiFiUDP udp;
const unsigned int UDP_PORT = 2462;
char pkt_buf[255];

Servo serv;

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    Serial.println("Connecting to ");
    Serial.println(ssid);
    
    //connect to your local wi-fi network
    WiFi.begin(ssid, password);
    
    //check wi-fi is connected to wi-fi network
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    udp.begin(UDP_PORT);

    serv.attach(SERVO_PIN, 700, 2300); // fs5106b limits
}

void loop() 
{
    int pkt_size = udp.parsePacket();
    if( pkt_size > 0 )
    {
        // received a packet
        Serial.printf("Received %d bytes from %s:%d\n", pkt_size, udp.remoteIP().toString().c_str(), udp.remotePort());
        int n_read = udp.read(pkt_buf, 255);
        
        if( n_read <= 0 ) return; // no data found ?_?
        
        // was actually able to read data
        pkt_buf[n_read] = '\0';
        Serial.printf("Packet contents: %s\n", pkt_buf);

        // parse as single integer
        int pos = atoi(pkt_buf);
        serv.write(pos);
    }
}
