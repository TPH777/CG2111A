#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>

volatile TDirection dir;
#define PI 3.141592654
#define ALEX_LENGTH 25.7
#define ALEX_BREADTH 15
#define WHEEL_CIRC 6.6 * PI 
#define COUNTS_PER_REV 3.3 // Ticks per revolution from the encoder
float alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
float alexCirc = alexDiagonal * PI;


/*
  Alex’s Movement Configuration and Function Definition
*/
// Encoder Ticks
volatile unsigned long leftForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// To track changes
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

// Initialize Alex's internal states
void initializeState() 
{
  leftForwardTicks = 0;
  leftReverseTicks = 0;
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Ticks Computation for turning
unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  ccw(ang, speed);
}

void right(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftForwardTicksTurns + deltaTicks;
  cw(ang, speed);
}

/*
  Encoder Configuration and ISR Definition
*/
void enablePullups() // Enable pull up resistors on pin 18 (Left Encoder)
{
  DDRD = 0;
  PORTD = 0b00001000; // Set Pin 18 to input
}

void setupEINT() // For Pin 18 
{
  EICRA = 0b10000000; // Set to falling edge
  EIMSK = 0b00001000;
}

ISR(INT3_vect) { // Pin 18
  leftISR();
}

void leftISR() // Functions to be called by INT3 ISRs.
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}


/*
  Color Sensor Configuration and Function Definition
*/
#define colorOut 20
volatile unsigned long color;
volatile unsigned long colorId[3];

void setColor() {
  DDRA = 0b00001111;
  DDRD = 0;
  PORTA = 0b00000001;
}

unsigned long checkColor() {
  strip.setBrightness(0);
  strip.show();
  PORTA = 0b00000001;
  colorId[0] = pulseIn(colorOut, LOW);
  delay(100);
  PORTA = 0b00001101;
  colorId[1] = pulseIn(colorOut, LOW);
  delay(100);
  PORTA = 0b00001001;
  colorId[2] = pulseIn(colorOut, LOW);
  delay(100);
  strip.setBrightness(8);
  strip.show();
  for (int i = 0; i < 3; i++) {
    if (colorId[i] > 2000) {
      if (colorId[0] < colorId[1] && colorId[0] < colorId[2]) {
        colorWipe(strip.Color(255, 0, 0), 50);
        return 0; // Red
      } else {
        colorWipe(strip.Color(0, 255, 0), 50);
        return 1; // Green
      }
    }
  }
  colorWipe(strip.Color(255, 255, 255), 50);
  return 2; // White
}


/*
  Ultrasonic Configuration and Function Definition
*/
void setUltrasonic() {
  DDRA |= 0b00010000;
}

unsigned long checkUltrasonic() {
  PORTA &= ~(1<<4);
  delayMicroseconds(2);
  PORTA |= (1<<4);
  delayMicroseconds(10);
  PORTA &= ~(1<<4);
  long duration = pulseIn(27, HIGH);
  return duration * 0.034 / 2;
}


/*
  Ring Light Configuration and Function Definition
*/
#define RING 31 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, RING, NEO_GRB + NEO_KHZ800);

void setRing() { // Setup
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(8);
}

void colorWipe(uint32_t c, uint8_t wait) { // To display a fixed color
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbowCycle() { // To display all colors on ring light
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
  }
}


/*
  Setup and start codes for serial communications
*/
void setupSerial() // Set up the serial connection
{
  Serial.begin(9600); // To replace later with bare-metal
  // UBRR3L = 103;
}

void startSerial() // Start the serial connection
{
  // Using Arduino Wiring
  // To be replaced with bare-metal code
}

int readSerial(char *buffer) // Read the serial port
{
  int count = 0;
  while (Serial.available()) // This will be replaced later with bare-metal code.
    buffer[count++] = Serial.read();
  return count;
}

void writeSerial(const char *buffer, int len) // Write to the serial port
{
  Serial.write(buffer, len); // Replaced later with bare-metal code
}


/*
  Alex Communication Routines from R-Pi
*/
void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      forward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      left((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      right((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
      backward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
      stop();
      break;
    case COMMAND_GET_STATS:
      color = checkColor();
      sendStatus();
    default:
      sendBadCommand();
  }
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void waitForHello()
{
  int exit = 0;
  while (!exit)
  {
    TPacket hello;
    TResult result;
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}


/*
  Alex Communication Routines to R-Pi
*/
TResult readPacket(TPacket *packet)
{
  char buffer[PACKET_SIZE];
  int len = readSerial(buffer); // Reads in data from the serial port
  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet); // deserializes to packet
}

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = leftReverseTicks;
  statusPacket.params[2] = leftForwardTicksTurns;
  statusPacket.params[3] = leftReverseTicksTurns;
  statusPacket.params[4] = forwardDist;
  statusPacket.params[5] = reverseDist;
  statusPacket.params[6] = colorId[0];
  statusPacket.params[7] = colorId[1];
  statusPacket.params[8] = colorId[2];
  statusPacket.params[9] = color;
  statusPacket.params[10] = checkUltrasonic();
  sendResponse(&statusPacket);
}

// For debugging (To PC), not used in mission
void sendMessage(const char *message)
{
  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

// For debugging (In Arduino), not used in mission
void dbprintf(char* format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() // Wrong magic number
{
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum() // Wrong Checksum
{
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() // Invalid Command
{
  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket)); // Serialise Packet
  writeSerial(buffer, len); // Send to R-Pi
}

/*
  Main Function
*/
void setup() {
  cli();
  setupEINT();
  setColor();
  setUltrasonic();
  setupSerial();
  startSerial();
  setRing();
  enablePullups();
  initializeState();
  rainbowCycle();
  sei();
}

void loop() {

  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  }
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }

  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      unsigned long distance = checkUltrasonic();
      delay(100);
      if ((distance <= 10 && distance != 0) || forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
        sendStatus();
      }     
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
        sendStatus();
      }
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
      sendStatus();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        sendStatus();
      }
    }
    else {
      if (dir == RIGHT) {
        if (leftForwardTicksTurns >= targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
          sendStatus();
        }
      }
      else {
        if (dir == STOP) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
          sendStatus();
        }
      }
    }
  }
}
