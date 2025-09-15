#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <U8g2lib.h>
// ------------------ OLED Config ------------------
// U8g2 for SSD1306 128x64 I2C
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ------------------ Hardware Config ------------------
#define IN1 2        // L298N IN1
#define IN2 3        // L298N IN2
#define IN3 4        // L298N IN3
#define IN4 5        // L298N IN4
#define SENSOR_PIN 7 // logical sensor (marker at 0°)

// Stepper setup (4-wire)
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// ------------------ Ethernet Config ------------------
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
unsigned int localPort = 8888;

EthernetUDP Udp;
IPAddress remoteIp;
unsigned int remotePort;

// ------------------ Variables ------------------
long stepsPerRevolution = 0;
bool calibrated = false;
bool streaming = false;
bool running = false;
char lastCmd[20] = "";

// ------------------ Functions ------------------
void calibrateStepper()
{
  stepper.setSpeed(200);
  while (digitalRead(SENSOR_PIN) == HIGH)
  {
    stepper.runSpeed();
  }
  stepper.setCurrentPosition(0);

  stepsPerRevolution = 0;
  stepper.setSpeed(200);
  do
  {
    stepper.runSpeed();
    stepsPerRevolution++;
  } while (digitalRead(SENSOR_PIN) == HIGH);

  calibrated = true;
  Serial.print("Steps/rev: ");
  Serial.println(stepsPerRevolution);
}

void sendPosition()
{
  if (!calibrated)
    return;

  long stepPos = stepper.currentPosition() % stepsPerRevolution;
  if (stepPos < 0)
    stepPos += stepsPerRevolution;

  float degrees = (360.0 * stepPos) / stepsPerRevolution;

  char reply[50];
  snprintf(reply, sizeof(reply), "POS: %.2f deg (%ld steps)", degrees, stepPos);

  Udp.beginPacket(remoteIp, remotePort);
  Udp.write(reply);
  Udp.endPacket();
}

void sendText(const char *msg)
{
  Udp.beginPacket(remoteIp, remotePort);
  Udp.write(msg);
  Udp.endPacket();
}

void updateOLED(const char *msg)
{
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.setCursor(0, 8);
    u8g2.print("Stepper Controller");
    u8g2.setCursor(0, 18);
    u8g2.print("IP:");
    u8g2.print(Ethernet.localIP());
    u8g2.setCursor(0, 28);
    u8g2.print("State:");
    u8g2.print(running ? "Run" : "Stop");
    u8g2.setCursor(0, 38);
    u8g2.print("Stream:");
    u8g2.print(streaming ? "ON" : "OFF");
    u8g2.setCursor(0, 48);
    u8g2.print("Speed:");
    if (calibrated)
    {
      int degPerSec = (stepper.speed() * 360L) / stepsPerRevolution;
      u8g2.print(degPerSec);
      u8g2.print(" d/s");
    }
    else
    {
      u8g2.print("N/A");
    }
    u8g2.setCursor(0, 58);
    u8g2.print("Cmd:");
    u8g2.print(lastCmd);
    if (msg && strlen(msg) > 0)
    {
      u8g2.setCursor(0, 64);
      u8g2.print(msg);
    }
  } while (u8g2.nextPage());
}

void handleUDP()
{
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    remoteIp = Udp.remoteIP();
    remotePort = Udp.remotePort();

    char incoming[packetSize + 1];
    Udp.read(incoming, packetSize);
    incoming[packetSize] = 0;

    Serial.print("UDP cmd: ");
    Serial.println(incoming);
    strcpy(lastCmd, incoming);

    if (strcmp(incoming, "START") == 0)
    {
      float speed = (stepsPerRevolution / 2.0); // 1 rev / 2s
      stepper.setSpeed(speed);
      running = true;
      streaming = true;
    }
    else if (strcmp(incoming, "STOP") == 0)
    {
      running = false;
    }
    else if (strncmp(incoming, "SPEED:", 6) == 0)
    {
      float degPerSec = atof(incoming + 6);
      float stepsPerSec = (degPerSec / 360.0) * stepsPerRevolution;
      stepper.setSpeed(stepsPerSec);
      running = true;
      streaming = true;
    }
    else if (strcmp(incoming, "POS?") == 0)
    {
      sendPosition();
    }
    else if (strcmp(incoming, "STREAM:OFF") == 0)
    {
      streaming = false;
    }
    else if (strcmp(incoming, "STREAM:ON") == 0)
    {
      streaming = true;
    }
    else if (strcmp(incoming, "RESET") == 0)
    {
      running = false;
      streaming = false;
      calibrateStepper();
      sendText("RESET OK");
    }
  }
}

// ------------------ Setup ------------------
void setup()
{
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  // OLED Init

  if (!u8g2.begin())
  {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }

  // Ethernet DHCP
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("DHCP failed, fallback IP");
    IPAddress ip(192, 168, 1, 177);
    Ethernet.begin(mac, ip);
  }
  delay(1000);

  Serial.print("My IP: ");
  Serial.println(Ethernet.localIP());

  Udp.begin(localPort);

  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(500);

  calibrateStepper();
  running = false;
  updateOLED("Setup complete");
}

// ------------------ Loop ------------------
void loop()
{
  handleUDP();

  if (running)
  {
    stepper.runSpeed();
  }

  if (streaming)
  {
    sendPosition();
    delay(200);
  }

  updateOLED(""); // refresh screen
}
