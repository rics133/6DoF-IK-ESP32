#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <Adafruit_PWMServoDriver.h>

// Robot Arm servo firmware.
//
// Listens on UDP 4210 for six joint angles in radians, payload
// "q1,q2,q3,q4,q5,q6\n". Reachable on the LAN as braccio-robot.local.
// Servos hang off a PCA9685 over I2C; each joint has +/- pi/2 of travel.
// The host side that produces this packet is main.py / ik_dls_rpy.py.

// Network
const char* ssid = "wifi_ssid";
const char* password = "wifi_pw";
const unsigned int localPort = 4210;

// PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Per-servo trim, adjust for the servos actually fitted
#define SERVOMIN  150 
#define SERVOMAX  600 

WiFiUDP Udp;
char packetBuffer[255];

void setup() {
  Serial.begin(115200);
  
  // Init the PCA9685 over I2C
  pwm.begin();
  pwm.setPWMFreq(50); // standard servo frequency

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");

  // mDNS so the board answers to braccio-robot.local
  if (!MDNS.begin("braccio-robot")) {
    Serial.println("mDNS failed to start");
  } else {
    Serial.println("mDNS started, answering to: braccio-robot.local");
  }

  Udp.begin(localPort);

  Serial.printf("Listening on UDP port %u...\n", localPort);
  delay(20);
}

// Radians to PWM pulse. 0 rad is servo centre, clamped to the joint's
// +/- pi/2 of travel.
int radToPWM(float rad) {
  if(rad < -1.57) rad = -1.57;
  if(rad > 1.57) rad = 1.57;

  float pwmVal = (rad - (-1.57)) * (SERVOMAX - SERVOMIN) / (1.57 - (-1.57)) + SERVOMIN;
  return (int)pwmVal;
}

void loop() {
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    // Null terminate, strtok needs it below
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    // payload is "q1,q2,q3,q4,q5,q6"
    float q[6];
    char *token = strtok(packetBuffer, ",");
    int i = 0;

    while (token != NULL && i < 6) {
      q[i] = atof(token);
      token = strtok(NULL, ",");
      i++;
    }

    // drop anything that didn't carry all six joints
    if (i == 6) {
      for (int j = 0; j < 6; j++) {
        int pulse = radToPWM(q[j]);
        pwm.setPWM(j, 0, pulse);
      }
    }
  }
}