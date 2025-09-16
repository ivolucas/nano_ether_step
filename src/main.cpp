#include <Arduino.h>
#include <SPI.h>
#include <UIPEthernet.h> // For ENC28J60
#include <UIPUdp.h>
#include <AccelStepper.h>

// ------------------ Hardware Config ------------------
#define SENSOR_PIN 3

#define IN1 4 // L298N IN1
#define IN2 5 // L298N IN2
#define IN3 6 // L298N IN3
#define IN4 7 // L298N IN4

#define ENA 5
#define ENB 6

// logical sensor (marker at 0°)

// Stepper setup (4-wire)
AccelStepper stepper(AccelStepper::FULL2WIRE, IN1, IN3, IN2, IN4);

// ------------------ Ethernet Config ------------------
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
unsigned int localPort = 8888;

EthernetUDP Udp;
IPAddress remoteIp;
unsigned int remotePort;

// ------------------ Variables ------------------
volatile bool zeroMarked = false;
volatile uint16_t stepsPerRevolution = 0;
volatile bool calibrated = false;
bool streaming = false;
bool running = false;
int powerLevel = 200; // 0-255
char lastCmd[10] = "";

// ------------------ Functions ------------------
void setMotorPower(uint8_t level)
{
  // level: 0–255 (PWM duty cycle)
  analogWrite(ENA, level);
  analogWrite(ENB, level);
}

void enableMotor()
{
  setMotorPower(powerLevel);
}

void disableMotor()
{
  setMotorPower(0);
}

void calibrateStepper()
{
  enableMotor();
  Serial.print("Calibration start");
  stepper.setCurrentPosition(0);
  stepper.stop();
  calibrated = false;
  zeroMarked = false;
  stepper.setSpeed(200);

  while (!zeroMarked)
  {
    stepper.runSpeed();
  }
  Serial.println("Zero Found");
  stepper.setSpeed(200);
  while (!calibrated)
    stepper.runSpeed();

  Serial.println("Steps/rev: ");
  Serial.println(stepsPerRevolution);
  disableMotor();
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

    strncpy(lastCmd, incoming, sizeof(lastCmd) - 1);
    lastCmd[sizeof(lastCmd) - 1] = 0;

    char cmd = incoming[0]; // first letter = command

    switch (cmd)
    {
    case 'S':                                     // Start
      stepper.setSpeed(stepsPerRevolution / 2.0); // 1 rev / 2 sec
      enableMotor();
      running = true;
      streaming = true;
      break;

    case 'X': // Stop
      disableMotor();
      running = false;
      break;

    case 'V':
    { // Speed in deg/s
      int degPerSec = atoi(incoming + 1);
      float stepsPerSec = (degPerSec / 360.0) * stepsPerRevolution;
      stepper.setSpeed(stepsPerSec);
      enableMotor();
      running = true;
      streaming = true;
      break;
    }

    case 'P': // Position query
      sendPosition();
      break;

    case 'O': // Stream ON
      streaming = true;
      break;

    case 'F': // Stream OFF
      streaming = false;
      break;

    case 'R': // Reset
      enableMotor();
      running = false;
      streaming = false;
      calibrateStepper();
      sendText("R_OK");
      break;
    }
  }
}

// ------------------ ISR ------------------
void isrMarker()
{
  Serial.println("sensor");
  if (!calibrated)
  {
    if (zeroMarked && stepper.currentPosition() > 20)
    {
      calibrated = true;
      stepsPerRevolution = stepper.currentPosition();
    }
    else
    {
      zeroMarked = true;
      stepper.setCurrentPosition(0);
    }
  }
}

// ------------------ Setup ------------------
void setup()
{

  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz on D5 and D6
  setMotorPower(0); // motor off
  pinMode(SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), isrMarker, RISING);

  Serial.begin(9600);

  Serial.println("DHCP begin");
  // ENC28J60 with DHCP
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("DHCP failed, fallback IP");
    IPAddress ip(192, 168, 1, 177);
    Ethernet.begin(mac, ip);
  }

  switch (Ethernet.linkStatus())
  {
  case Unknown:
    Serial.println("Ethernet Link Status: Unknown");
    break;
  case LinkON:
    Serial.println("Ethernet Link Status: LinkON");
    break;
  case LinkOFF:
    Serial.println("Ethernet Link Status: LinkOFF");
    break;
  default:
    break;
  }

  switch (Ethernet.hardwareStatus())
  {
  case EthernetW5100:
    Serial.println("Ethernet Hardware Status: EthernetW5100");
    break;
  case EthernetW5200:
    Serial.println("Ethernet Hardware Status: EthernetW5200");
    break;
  case EthernetW5500:
    Serial.println("Ethernet Hardware Status: EthernetW5500");
    break;
  case EthernetENC28J60:
    Serial.println("Ethernet Hardware Status: EthernetENC28J60");
    break;
  default:
    Serial.println("Ethernet Hardware Status: EthernetNoHardware");
    break;
  }

  delay(1000);
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());
  Udp.begin(localPort);

  Udp.begin(localPort);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(500);

  calibrateStepper();

  Serial.println("Setup complete");
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
  }
}
