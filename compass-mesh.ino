/*
 * COMPASS WITH RF24 NETWORK
 * 
 * Broadcast radio number to all nodes by writing to each pipe.
 * Read from just one.
 * 
 * Or write to just one and read from each other pipe.
 * 
 */

#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include "SeekerParticleEmitter.h"

/****************** User Config ***************************/
#define PING_INTERVAL_MILLIS 1000
#define TOTAL_NODES 4 //TOTAL NUMBER OF NODES IN MESH

//THESE MUST BE SET! CHANGE ADDRESS AS YOU UPLOAD TO EACH NODE!
 uint8_t radioNumber = 3;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 */
 RF24 radio(9,10);
/**********************************************************/

#define PRESENCE_TIMEOUT_MILLIS 10000
#define MILLIS_PER_FRAME 10  // 83   // 1/12

#define LED_PIN 6
#define NUM_PIXELS 12
#define NUM_PARTICLES 8
#define POWER 256
#define COMPASS_READ_INTERVAL 66

uint32_t friendPixelColors[12];

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
SeekerParticleEmitter emitter = SeekerParticleEmitter(NUM_PIXELS, NUM_PARTICLES);

LSM303 compass;
unsigned long lastReadAt = 0;
uint8_t pixelHour = 0;
float alpha = 0.9; // 0.1; //0.9 // factor to smooth compass reading. 
float heading = 0.0;

//const uint64_t writePipe = 0xF0F0F0F00LL;
//const uint64_t readPipe = 0xF0F0F0F00LL;

byte addresses[][6] = { "1Node", "2Node", "3Node", "4Node" };

long presence[TOTAL_NODES];
unsigned long lastPingSentAt = 0;
unsigned long lastStripUpdateAt = 0;
unsigned long compassLastReadAt = 0;

uint32_t wheel(uint8_t WheelPos);

uint32_t colorForFriend(int nodeAddress) {
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  
  switch (nodeAddress) {
    case 0:
      red = 255;
      break;
    case 1:
      green = 255;
      break;
    case 2:
      blue = 255;
      break;
    case 3:
      blue = 255;
      red = 255;
      break;
  }
  return strip.Color(red, green, blue);
  
  //return wheel(nodeAddress / (TOTAL_NODES-1) * 255);
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("rf24-mesh-ping"));

  randomSeed(analogRead(0));

  // Setup and configure radio

  radio.begin();
  Serial.println(F("radio initialized"));

  // Or try intermediate settings first: 
  // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_MAX);
  // [ ]  Put a 10uF cap across + and - on the chip

  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range
  radio.setChannel(108); // 2.508 Ghz - Above most Wifi Channels

  radio.enableAckPayload();                     // Allow optional ack payloads

  // is this necessary?
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  radio.openReadingPipe(1, addresses[radioNumber]);

  Serial.println(F("Starting to listen..."));
  radio.startListening();                       // Start listening  
  Serial.println(F("Listening."));
  
  radio.writeAckPayload(1, &radioNumber, 1);          // Pre-load an ack-paylod into the FIFO buffer for pipe 1
//  radio.printDetails();

  Wire.begin();
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>){ -2929,  -3157,  -3112};
  compass.m_max = (LSM303::vector<int16_t>){ +4217,  +3779,  +4325};
  
  strip.begin();
  strip.setBrightness(192);
  strip.show();
  emitter.begin();

  lastReadAt = millis();

  for (int i=0; i<TOTAL_NODES; i++) {
    presence[i] = -1;
  }

  emitter.particleColorScale = 0.33;
}

void displayFriendColors() {
 int nearbyFriendsCount = 0;
 int colors[TOTAL_NODES];

  for (int i=0; i<TOTAL_NODES; i++) {
    colors[i] = 0;
 
    if (i == radioNumber || presence[i] == -1) continue;

    unsigned long timeSincePing = millis() - presence[i];

//    Serial.print("radio ");
//    Serial.print(i);
//    Serial.print(" last pinged ");
//    Serial.print(timeSincePing);
//    Serial.println(" millis ago");
    
    if (timeSincePing < PRESENCE_TIMEOUT_MILLIS) {
      colors[nearbyFriendsCount++] = i;
    }
  }

//Serial.print(radioNumber);
//Serial.print(": ");
//Serial.println(nearbyFriendsCount);

  if (nearbyFriendsCount > 0) {
    for (int i=0; i<nearbyFriendsCount ; i++) {
      int pixelsPerBlock = NUM_PIXELS / nearbyFriendsCount;

      int c = 0;
      for (int p=0; p<NUM_PIXELS; p++) {
//        friendPixelColors[p] = colorForFriend(colors[c]);
        strip.setPixelColor(p, colorForFriend(colors[c]));
//        Serial.print("pixel ");
//        Serial.print(p);
//        Serial.print(" color ");
//        Serial.println(c);

        if (p != 0 && (p+1) % pixelsPerBlock == 0) {
          // change to next person's color
          c++;
        }
      }
    }    
  }
  else {
//    Serial.println("No nearby friends");
//    uint8_t minColor = 50;
    for (int p=0; p<NUM_PIXELS; p++) {
      strip.setPixelColor(p, Adafruit_NeoPixel::Color(15, 15, 0));
//      friendPixelColors[p] = Adafruit_NeoPixel::Color(50, 0, 0);
//      strip.setPixelColor(p, Adafruit_NeoPixel::Color(minColor, minColor, minColor));
    }
//    for (int p=0; p<nearbyFriendsCount+1; p++) {
//      strip.setPixelColor(p, Adafruit_NeoPixel::Color(10, 50, 0));
//    }
//    for (int p=11; p>11-radioNumber+1; p--) {
//      strip.setPixelColor(p, Adafruit_NeoPixel::Color(0, 0, 50));
//    }
  }
  
}

//
// TODO: 
// [ ]  Calibrate
// [ ]  Print roll
// [ ]  Print pitch
// 

void loop()
{
  unsigned long now = millis();

  if (now - compassLastReadAt > COMPASS_READ_INTERVAL) {
    compass.read();
    now = millis();
    compassLastReadAt = now;
  
    //  int pitch = compass.pitch();
    //  int roll = compass.roll();
    //
    //  Serial.print("pitch: ");
    //  Serial.println(pitch);
    //  Serial.print("roll: ");
    //  Serial.println(roll);
      
  }

  if (now - lastStripUpdateAt > MILLIS_PER_FRAME) {
    blacken();
    displayFriendColors();   
//    displayCompassRose();
    displayNorth();

    strip.show();
    now = millis();
    lastStripUpdateAt = now;
  }

  sendPing();
  listenForPing();
}

/////////// radio vvv
void sendPing() {
  if (millis() - lastPingSentAt > PING_INTERVAL_MILLIS) {
    byte gotByte;
    radio.stopListening();

    for (int i = 0; i < TOTAL_NODES; i++) {
      if (i == radioNumber) continue;

      radio.openWritingPipe(addresses[i]);
  
//      unsigned long time = micros();

      delay(10);
//      Serial.print(radioNumber);
//      Serial.print(F(": Sending to radio "));
//      Serial.println(i);

      if ( radio.write(&radioNumber, 1) ) {
//        Serial.println("SENT");
        if (!radio.available()) {
          // If nothing in the buffer, we got an ack but it is blank
        }
        
        else {
//          Serial.println("RECEIVED ACK");
          while (radio.available()) {                      // If an ack with payload was received
            radio.read( &gotByte, 1 );                  // Read it, and display the response time
//            unsigned long timer = micros();
            
//            Serial.print(F("Got response from radio "));
//            Serial.print(gotByte);
//            Serial.print(F(" round-trip delay: "));
//            Serial.print(timer-time);
//            Serial.println(F(" microseconds"));
  
            int pingerIndex = (int)gotByte;
            presence[pingerIndex] = millis();
          }
        }
        
      }
      else {
      // If no ack response, sending failed
//        Serial.println(F("Sending failed."));
      }
    }

    radio.openReadingPipe(1, addresses[radioNumber]);
    radio.startListening();
  }  
}

void listenForPing() {
  byte pipeNo;
  byte gotByte = -1; 
  while( gotByte != radioNumber && radio.available(&pipeNo)) {              // Read all available payloads
    radio.read( &gotByte, 1 );                   

//    Serial.print(F("I am radio "));
//    Serial.print(radioNumber);  
//    Serial.print(F(" and was pinged by radio "));
//    Serial.println(gotByte);

    int pingerIndex = (int)gotByte;
    presence[pingerIndex] = millis();
    
                                                 // Since this is a call-response. Respond directly with an ack payload.
                                                 // Ack payloads are much more efficient than switching to transmit mode to respond to a call
    radio.writeAckPayload(pipeNo, &radioNumber, 1);  // This can be commented out to send empty payloads.
  }  
}
/////////// radio ^^^

void displayNorth() {
  int measurement = trunc(compass.heading());
  heading = alpha * measurement + (1-alpha) * heading;
   
  pixelHour = min(11, (360 - heading) / 30);
    
  emitter.attractorPosition = float(pixelHour) / 11.0f;
  emitter.update(strip);
}

void displayCompassRose() {
  int opp = pixelHour - (NUM_PIXELS / 2);
  if (opp < 0) {
    opp += NUM_PIXELS - 1;
  }
//  Serial.println("opp");
//  Serial.println(opp);
//  Serial.println("ph");
//  Serial.println(pixelHour);
  strip.setPixelColor(opp, 512); //strip.Color(r, g, b));
  return;

  for (uint8_t i=0; i<NUM_PIXELS; i++) {
    
    int diff = abs(i - pixelHour);
    int remainder = diff % (NUM_PIXELS/4);

    
  
    if (remainder == 0 && i == (NUM_PIXELS / 2) - 1) {
//      uint8_t r, g, b;
//      r = 0;
//      g = 51;
//      b = 102;
  
//      float s = 0.10;
//      strip.setPixelColor((i == 0 ? NUM_PIXELS-1 : i-1), strip.Color(r*s, g*s, b*s));
      strip.setPixelColor(i, 0); //strip.Color(r, g, b));
//      strip.setPixelColor((i == NUM_PIXELS-1 ? 0 : i+1), strip.Color(r*s, g*s, b*s));
    }
  }
}

void blacken() {
    for (uint8_t i=0; i<NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
//   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
   return strip.Color(0, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, 255 - WheelPos * 3, 0);
//   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}


