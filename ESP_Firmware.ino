#include <ESP8266WiFi.h>
#include <Servo.h>

#include "util.h"
#include "kinematics.h"
#include "pathiterator.h"
#include "dynamixel.h"
#include "piwifi.h"

#define PIN_LED D4
#define PIN_5V_GOOD D5
#define PIN_7V_GOOD D6
#define PIN_PEN_SRV D8

enum EspMode
{
    MODE_IDLE = 0,
    MODE_DRAW = 1,
    MODE_PAUSE = 2
};

const int ESP_ID = 1;
EspMode currentMode = MODE_IDLE;

// Wifi Interface
const char* const ssid = "KATY-CORSAIR 0494";        // Enter SSID here
const char* const password = "0Y46u%02";  //Enter Password here

// transmission -> Pi
IPAddress PI_SERVER(192,168,137,1);
const unsigned int PI_PORT = 1896;
WiFiClient client;
char sendBuf[32];

// reception <- Pi
const unsigned int LISTEN_PORT = 1897;
WiFiServer server(LISTEN_PORT);
WiFiClient piRemote;

const unsigned int WIFI_RCV_SIZE = 255;
char wifiRcvBuf[WIFI_RCV_SIZE];
char *wifiReadLoc = wifiRcvBuf;
int totalWifiRead = 0;
uint8_t *wifiDataStart = (uint8_t *)wifiRcvBuf;

// Drawing buffers
PathQueueIterator path;
CirclePathIterator path2(0,25,10);

struct arm_angles dockAngles;

// digital interfaces
Servo penActuator;
bool powerOk;

// Utility Functions
//-------------------------------------------------------------------

void setPenDown( bool extended )
{
    int setPoint = extended ? 180 : 0;
    penActuator.write(setPoint);
}

void setAngles( const struct arm_angles& ang )
{
    uint16_t s = angle_to_goal_pos(ang.shoulder);
    uint16_t e = angle_to_goal_pos(-ang.elbow);

    write_synch_goal(s, e);

    Serial.printf("Servos set to: s = %d, e = %d\n", s, e);
}

void gotoIdle()
{
    path.clear();
    setPenDown(false);
    setAngles(dockAngles);
    currentMode = MODE_IDLE;

    Serial.println("Start Idle Mode");
}

// WiFi Functions
//-------------------------------------------------------------------

void sendStatus()
{
    int nWrite = sprintf(sendBuf, "1,%d,%d,%d,%d,%d,%d\n",
        ESP_ID,
        powerOk,
        shoulder_status.last_status,
        elbow_status.last_status,
        0,
        path.remaining());
    
    if( client.connect(PI_SERVER, PI_PORT) )
    {
        client.write(sendBuf, nWrite);
        client.stop();
    }
    else
    {
        Serial.println("Couldn't connect to pi socket");
    }
}

void handleWifiPacket()
{
    // smallest possible is 2 byte mode packet
    if( totalWifiRead < 2 ) return;

    

    WPacketType type = (WPacketType)wifiRcvBuf[0];
    EspMode newMode;

    switch( type )
    {
        case WPKT_CHANGE_MODE:
            newMode = (EspMode)wifiRcvBuf[1];
            if( (newMode == MODE_IDLE) && (currentMode != MODE_IDLE) )
            {
                gotoIdle();
            }
            break;

        case WPKT_POINTS:
            int nPts = wifiRcvBuf[1];

            char *ptPtr = wifiRcvBuf + 2;
            for( int i = 0; i < nPts; i++ )
            {
                if( (ptPtr + WIFI_PT_LEN) > (wifiRcvBuf + pktLength) )
                {
                    Serial.println("Wifi packet #pts exceeds received length");
                    break;
                }

                // process a point/command
                float x = getWifiPtX(ptPtr);
                float y = getWifiPtY(ptPtr);

                Serial.printf("Pkt: type=%d: x=%f, y=%f\n", (int)ptPtr[0], x, y);

                switch( (PathElementType)ptPtr[0] )
                {
                    case PATH_PEN_UP:
                        // pen up in current position
                        path.addPenMove(PEN_UP);
                        break;

                    case PATH_PEN_DOWN:
                        // make sure pen goes down in correct position
                        path.addMove(x, y);
                        path.addPenMove(PEN_DOWN);
                        break;

                    case PATH_MOVE:
                        path.addMove(x, y);
                        break;

                    default:
                        break;
                }

                // move to next point struct
                ptPtr += WIFI_PT_LEN;
            }

            break;
    }
}

// Arduino Functions
//-------------------------------------------------------------------

void setup()
{
    // set status LED to off
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // PWR_GOOD inputs
    pinMode(PIN_5V_GOOD, INPUT_PULLUP);
    pinMode(PIN_7V_GOOD, INPUT_PULLUP);

    // Start serial comms
    Serial.begin(57600, SERIAL_8N1);
    init_dyn_serial();
    delay(1000);

    Serial.println("\nChecking voltage...");
    while( !powerOk )
    {
        powerOk = digitalRead(PIN_5V_GOOD) && digitalRead(PIN_7V_GOOD);
    }
    Serial.println("Voltage good!");

    // setup linear actuator
    penActuator.attach(PIN_PEN_SRV, 1500, 2100); // microsec limits
    setPenDown(false);
    
    Serial.print("\nConnecting to ");
    Serial.println(ssid);
    
    //connect to your local wi-fi network
    WiFi.begin(ssid, password);
    
    //check wi-fi is connected to wi-fi network
    uint8_t ledBlink = LOW;

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        digitalWrite(PIN_LED, ledBlink);
        ledBlink = !ledBlink;
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    
    Serial.print("IP: ");
    Serial.print(WiFi.localIP());
    Serial.print("  Gateway: ");
    Serial.println(WiFi.gatewayIP());

    server.begin();

    digitalWrite(PIN_LED, LOW); // LED on

    sendStatus();

    dockAngles.shoulder = deg_to_rad(180);
    dockAngles.elbow = deg_to_rad(90);
}

void loop() 
{
    if( !piRemote || !piRemote.connected() )
    {
        WiFiClient piRemote = server.available();
        wifiReadLoc = wifiRcvBuf;
        totalWifiRead = 0;
    }
    
    if( piRemote && piRemote.connected() )
    {
        // received an incoming connection
        int toRead;

        if( toRead = piRemote.available() )
        {
            if( toRead > (RCV_BUF_LEN - totalWifiRead) )
            {
                toRead = RCV_BUF_LEN - totalWifiRead;
            }

            int nRead = piRemote.readBytes(wifiReadLoc, toRead);
            wifiReadLoc += nRead;
            totalWifiRead += nRead;

            Serial.printf("Received %d bytes\n", nRead);
        }

        handleWifiPacket();
    }
    
    powerOk = digitalRead(PIN_5V_GOOD) && digitalRead(PIN_7V_GOOD);

    // check for responses from dynamixels
    int responseId = dynamixel_rcv();

    if( responseId )
    {
        Serial.printf("Received response from %d\n", responseId);
    }

    //if( !(shoulder_status.last_pkt_acked && elbow_status.last_pkt_acked) ) return;

    if( currentMode == MODE_IDLE || currentMode == MODE_PAUSE )
    {
        return;
    }

    PathElement nextMove = path.moveNext();
    struct arm_angles ang;
    
    switch( nextMove.type )
    {
    case PATH_MOVE:
        ang = calculate_angles(nextMove.x, nextMove.y);
        setAngles(ang);
        break;

    case PATH_PEN_DOWN:
        setPenDown(true);
        break;

    case PATH_PEN_UP:
    default:
        setPenDown(false);
        break;
    }

    //delay(5);
}
