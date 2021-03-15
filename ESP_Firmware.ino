#include <ESP8266WiFi.h>

#include "util.h"
#include "kinematics.h"
#include "pathiterator.h"
#include "dynamixel.h"

#define SHOULDER_PWM D7
#define ELBOW_PWM D6

const char* const ssid = "KATY-CORSAIR 0494";        // Enter SSID here
const char* const password = "0Y46u%02";  //Enter Password here

//const char* const ssid = "SM-G892UAD7";        // Enter SSID here
//const char* const password = "4128498770";  //Enter Password here

const unsigned int LISTEN_PORT = 2462;
WiFiServer server(LISTEN_PORT);

const unsigned int PKT_BUF_SIZE = 255;
char pktBuf[PKT_BUF_SIZE];

PathQueueIterator path;
CirclePathIterator path2(0,25,10);

struct arm_angles dockAngles;

void setup() {
    Serial.begin(9600);
    init_dyn_serial();
    delay(1000);
    
    Serial.print("\nConnecting to ");
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
    path.addMove(x, y, true);
}

void setAngles( const struct arm_angles& ang )
{
    uint16_t s = angle_to_goal_pos(ang.shoulder);
    uint16_t e = angle_to_goal_pos(ang.elbow);

    write_synch_goal(s, e);

    Serial.printf("Servos set to: s = %d, e = %d\n", s, e);
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

    // check for responses from dynamixels
    int responseId = dynamixel_rcv();

    if( responseId )
    {
        Serial.printf("Received response from %d", responseId);
    }

    if( !(shoulder_status.last_pkt_acked && elbow_status.last_pkt_acked) ) return;

    PathElement nextMove = path2.moveNext();
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

    //delay(5);
}
