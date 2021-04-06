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

enum EspSetting
{
    SETTING_MODE = 1,
    SETTING_SPEED = 2
};

enum EspMode
{
    MODE_IDLE = 0,
    MODE_DRAW = 1,
    MODE_PAUSE = 2
};

enum EspDrawSpeed
{
    DSPEED_MAX = 0,
    DSPEED_25 = 1,
    DSPEED_50 = 2,
    DSPEED_75 = 3,
    DSPEED_100 = 4
};

uint16_t drawSpeedLUT[]
{
    0, 256, 512, 768, 1023
};

const int ESP_ID = 1;
EspMode currentMode = MODE_IDLE;
EspDrawSpeed currentSpeed = DSPEED_MAX;

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

WPacketBuffer wifiBuffer;

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

    Serial.printf("\nPen down: %d\n", extended);
}

void setAngles( const struct arm_angles& ang )
{
    uint16_t s = angle_to_goal_pos(ang.shoulder);
    uint16_t e = angle_to_goal_pos(-ang.elbow);

    write_synch_goal(s, e);

    //Serial.printf("Servos set to: s = %d, e = %d\n", s, e);
}

void gotoIdle()
{
    path.clear();
    setPenDown(false);
    setAngles(dockAngles);
    currentMode = MODE_IDLE;

    Serial.println("Start Idle Mode");
}

void changeSetting( EspSetting setting, uint8_t value )
{
    if( setting == SETTING_MODE )
    {
        EspMode newMode = (EspMode)value;
        Serial.printf("Set mode to %d\n", value);

        if( newMode != currentMode )
        {
            switch( newMode )
            {
                case MODE_IDLE:
                    gotoIdle();
                    break;

                default:
                    currentMode = newMode;
                    break;
            }
        }
    }
    else if( setting == SETTING_SPEED )
    {
        EspDrawSpeed newSpeed = (EspDrawSpeed)value;
        Serial.printf("Set speed to %d/4\n", value);

        if( (newSpeed != currentSpeed) && (newSpeed <= DSPEED_100) )
        {
            uint16_t speedRegVal = drawSpeedLUT[value];
            synch_write_value(MOVE_SPEED, 2, speedRegVal, speedRegVal);
        }
    }
}

// WiFi Functions
//-------------------------------------------------------------------

void sendStatus()
{
    int nWrite = sprintf(sendBuf, "1,%d,%d,%d,%d,%d,%d,%d\n",
        ESP_ID,
        powerOk,
        currentMode,
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

void handleWifiPacket( WPacketType type )
{
    switch( type )
    {
        case WPKT_SETTING:
            changeSetting((EspSetting)wifiBuffer.packetByte(WFIELD_SETTING_ID), (uint8_t)wifiBuffer.packetByte(WFIELD_SETTING_VAL));
            wifiBuffer.munch(WPKT_SETTING_LEN);
            break;

        case WPKT_POINTS:
            int nPts = wifiBuffer.packetByte(WFIELD_N_PTS);

            for( int i = 0; i < nPts; i++ )
            {
                // process a point/command
                PathElement nextPoint = wifiBuffer.packetPoint(WFIELD_POINTS + (WPOINT_LEN * i));

                Serial.printf("Rcv Point: type=%d: x=%f, y=%f\n", nextPoint.type, nextPoint.x, nextPoint.y);

                path.addElement(nextPoint);
            }

            wifiBuffer.munch(WPKT_POINTS_LEN + (WPOINT_LEN * nPts));

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
    penActuator.attach(PIN_PEN_SRV, 900, 2100); // microsec limits
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
    if( piRemote && !piRemote.connected() )
    {
        piRemote.stop();
    }

    if( !piRemote || !piRemote.connected() )
    {
        piRemote = server.available();
        wifiBuffer.reset();
    }
    
    if( piRemote && piRemote.connected() )
    {
        // received an incoming connection
        WPacketType pktType = wifiBuffer.tryReceiveData(piRemote);

        if( pktType != WPKT_NULL )
        {
            handleWifiPacket(pktType);
        }
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
        setPenDown(false);
        break;

    case PATH_END:
        gotoIdle();
        break;

    case PATH_NONE:
    default:
        currentMode = MODE_PAUSE;
        break;
    }

    //delay(5);
}
