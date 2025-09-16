#include <Arduino.h>
#include <SPI.h>
#include <UIPEthernet.h> // ENC28J60 support
#include <UIPUdp.h>

// ------------------ Hardware ------------------
#define SENSOR_PIN 3 // attach sensor to D2 (INT0) !!

// ------------------ Ethernet (ENC28J60) ------------------
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
unsigned int LOCAL_PORT = 8888;
EthernetUDP Udp;
IPAddress remoteIp;
unsigned int remotePort;

// ------------------ Measurement variables (shared with ISR) ------------------
volatile unsigned long lastPulseMicros = 0;   // timestamp of last pulse (micros)
volatile unsigned long prevPulseMicros = 0;   // previous pulse timestamp
volatile unsigned long revDurationMicros = 0; // measured revolution duration in micros (0 = unknown)
volatile unsigned int pulseCount = 0;         // count pulses (for diagnostics)

// ------------------ Control flags ------------------
bool streaming = false;
unsigned long lastStreamSent = 0;
volatile unsigned long streamInterval = 20;

// small buffer for last command (store ASCII)
char lastCmd[12] = "";

// ------------------ ISR: on rising edge of sensor ------------------
void isrMarker()
{
  unsigned long t = micros();

  Serial.println("isrMarker");
  // shift timestamps

  if (lastPulseMicros != 0)
  {
    // compute duration between consecutive pulses (revolution time)
    unsigned long duration = t - lastPulseMicros;
    Serial.print("Rev dur: ");
    Serial.println(duration);
    // basic sanity check: ignore too-small/too-large spikes
    if (duration > 100000UL && duration < 60UL * 1000000UL)
    { // >5ms and <60s
      pulseCount++;
      revDurationMicros = duration;
      prevPulseMicros = lastPulseMicros;
      lastPulseMicros = t;
      //streamInterval = revDurationMicros / 360; 
    }
    // else keep previous revDurationMicros
  }
  else
  {
    // first pulse
    lastPulseMicros = t;
  }
}

// ------------------ Helpers ------------------
void sendText(const char *msg)
{
  if (remotePort == 0)
    return; // no known client
  Udp.beginPacket(remoteIp, remotePort);
  Udp.write(msg);
  Udp.endPacket();
}

void sendPositionOnce()
{
  // compute position in integer degrees
  unsigned long revDur;
  unsigned long lastPulse;
  // copy volatile to local atomically
  noInterrupts();
  revDur = revDurationMicros;
  lastPulse = lastPulseMicros;
  interrupts();

  if (revDur == 0 || lastPulse == 0)
  {
    sendText("P:NA"); // no measurement yet
    return;
  }

  unsigned long now = micros();
  unsigned long elapsed;
  if (now >= lastPulse)
    elapsed = now - lastPulse;
  else

  {
    sendText("P:NA"); // no measurement yet

    return;
  }

  // wrap elapsed inside one revolution
  if (revDur != 0)
    elapsed = elapsed % revDur;

  // compute integer degrees: (elapsed * 360) / revDur
  // use 32-bit math; intermediate multiplication may overflow if values big -> do 64-bit
  unsigned long deg = (unsigned long)(((unsigned long long)elapsed * 360ULL) / (unsigned long long)revDur);

  char buf[32];
  // also send rev duration in ms and pulse count (optional)
  unsigned long revMs = revDur / 1000UL;
  snprintf(buf, sizeof(buf), "P:%lu D:%lums C:%u", deg, revMs, pulseCount);
  sendText(buf);
}

// ------------------ UDP handling ------------------
void handleUDP()
{
  int packetSize = Udp.parsePacket();
  if (packetSize > 0)
  {
    char incoming[packetSize + 1];
    int len = Udp.read(incoming, packetSize);
    if (len < 0)
      return;
    incoming[len] = 0;

    // store remote for later replies / streaming
    remoteIp = Udp.remoteIP();
    remotePort = Udp.remotePort();
    Serial.print("UDP from ");
    Serial.print(remoteIp);
    Serial.print(":");
    Serial.print(remotePort);
    Serial.print(" - ");
    // store lastCmd safely
    strncpy(lastCmd, incoming, sizeof(lastCmd) - 1);
    lastCmd[sizeof(lastCmd) - 1] = '\0';
    Serial.println(lastCmd);
    char cmd = incoming[0];
    switch (cmd)
    {
    case 'P': // position query
      sendPositionOnce();
      break;

    case 'O': // stream ON
      streaming = true;
      // acknowledge
      sendText("O_OK");
      break;

    case 'F': // stream OFF
      streaming = false;
      sendText("F_OK");
      break;

    case 'R': // reset measured duration and counters
      noInterrupts();
      revDurationMicros = 0;
      lastPulseMicros = 0;
      prevPulseMicros = 0;
      pulseCount = 0;
      interrupts();
      sendText("R_OK");
      break;

    default:
      // if command starts with 'V' we could set an assumed speed? not relevant here
      sendText("UNK");
      break;
    }
  }
}

void streamIfNeeded()
{
  if (!streaming)
    return;
  unsigned long nowMs = millis();
  if (nowMs - lastStreamSent >= streamInterval)
  {
    sendPositionOnce();
    lastStreamSent = nowMs;
  }
}

// ------------------ Setup & loop ------------------
void setup()
{
  // sensor pin must be on INT0 (D3) for attachInterrupt(INT0 ...)
  pinMode(SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), isrMarker, RISING);

  Serial.begin(115200);
  Serial.println(F("Index-only position tracker starting..."));

  // ENC28J60 DHCP
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println(F("DHCP failed - using fallback IP 192.168.1.177"));
    IPAddress ip(192, 168, 1, 177);
    Ethernet.begin(mac, ip);
  }
  delay(500);
  Serial.print(F("IP: "));
  Serial.println(Ethernet.localIP());

  Udp.begin(LOCAL_PORT);
  Serial.print(F("UDP listening on port "));
  Serial.println(LOCAL_PORT);
}

void loop()
{
  handleUDP();
  streamIfNeeded();
  // nothing else to do; keep loop small and responsive
}
