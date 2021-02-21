#include <ESP8266WiFi.h>
#include <Servo.h>

#include "util.h"
#include "kinematics.h"
#include "pathiterator.h"

#define SHOULDER_PWM D7
#define ELBOW_PWM D6

const char* const ssid = "KATY-CORSAIR 0494";        // Enter SSID here
const char* const password = "0Y46u%02";  //Enter Password here

const unsigned int LISTEN_PORT = 2462;
WiFiServer server(LISTEN_PORT);

const unsigned int PKT_BUF_SIZE = 255;
char pktBuf[PKT_BUF_SIZE];

PathQueueIterator path;

Servo shoulder;
Servo elbow;

struct arm_angles dockAngles;

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    Serial.print("Connecting to ");
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
    server.begin();

    shoulder.attach(SHOULDER_PWM, 700, 2300); // fs5106b limits
    elbow.attach(ELBOW_PWM, 700, 2300);

    dockAngles.shoulder = deg_to_rad(180);
    dockAngles.elbow = deg_to_rad(90);
}

void handlePacket( int pktLength )
{
    // smallest possible is 2 digits separated by comma
    if( pktLength < 3 ) return;

    // find comma delimiter
    int delimIdx = -1;
    for( int i = 1; i < (pktLength - 1); i++ )
    {
        if( pktBuf[i] == ',' )
        {
            delimIdx = i;
            break;
        }
    }

    // no comma found
    if( delimIdx < 0 ) return;

    float x, y;
    if( !parseFloatStrict(pktBuf, delimIdx, x) ) return;
    if( !parseFloatStrict(pktBuf + delimIdx + 1, pktLength - delimIdx - 1, y) ) return;

    // successfully parsed
    Serial.printf("Good data: x = %f, y = %f\n", x, y);
    path.addMove(x, y);
}

void setAngles( const struct arm_angles& ang )
{
    float shoulderDeg = rad_to_deg(ang.shoulder);
    float elbowDeg = rad_to_deg(ang.elbow);

    shoulder.write(shoulderDeg);
    elbow.write(elbowDeg);
}

void loop() 
{
    WiFiClient remote = server.available();
    
    if( remote )
    {
        // received an incoming connection
        int nRead = 0;
        while( remote.connected() )
        {
            if( remote.available() )
            {
                nRead = remote.readBytesUntil('\n', pktBuf, PKT_BUF_SIZE - 1);
                pktBuf[nRead] = '\0';
                
                // we only care about the first line
                while( remote.available() ) remote.read(); // flush incoming buffer
                remote.stop();
                break;
            }
        }

        Serial.printf("Received %d bytes: ", nRead);
        Serial.println(pktBuf);

        handlePacket(nRead);
    }

    PathElement nextMove = path.moveNext();
    struct arm_angles ang;
    
    switch( nextMove.type )
    {
    case PATH_MOVE:
        ang = calculate_angles(nextMove.x, nextMove.y);
        setAngles(ang);
        break;

    default:
        //setAngles(dockAngles);
        break;
    }

    delay(5);
}
