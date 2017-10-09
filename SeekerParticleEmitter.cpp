// SeekerParticleEmitter.cpp


#include <arduino.h>
#include "SeekerParticleEmitter.h"

#define FPS 60.0
#define MILLIS_PER_FRAME (1000 / FPS)

const float damping = 0.85;  // 0.925 is good
const float springForce = 2.0; // 1.25 is ok;

Color wheel(uint8_t index);
float squared(float x);

SeekerParticleEmitter::SeekerParticleEmitter(uint16_t _numPixels, uint8_t _numParticles) {
  numPixels = _numPixels;
  numParticles = _numParticles;
  attractorPosition = 0.5;
  frameLastUpdatedAt = 0;
  particleColorScale = 1.0;
 
  particles = Vector<Particle>();
}

void SeekerParticleEmitter::begin(void) {
  for (uint8_t i=0; i < numParticles; i++) {
    Particle p;
//    p.pos = random(2) ? 0.0 : 1.0;
    p.pos = float(random(numPixels+1)) / float(numPixels);
    p.acc = 0;
    p.vel = 0;
    p.springForceScale = float(random(50.0, 150.0)) / 100.0;
    p.color = wheel((float(i+1) / float(numParticles)) * 255);
    particles.push(p);
  }
}

void SeekerParticleEmitter::update(Adafruit_NeoPixel& strip) {

  unsigned long now = millis();
  
  if (frameLastUpdatedAt == 0) {
    frameLastUpdatedAt = now;
  }
  
//  if ((now - frameLastUpdatedAt) < MILLIS_PER_FRAME) {
//    return;
//  }

  Color colorBuffer[numPixels];
  Color curColor;
//  Color blackColor;
//  blackColor.rgb[0] = 0;
//  blackColor.rgb[1] = 0;
//  blackColor.rgb[2] = 0;

  for (uint16_t j=0; j < numPixels; j++) {
//    colorBuffer[j] = blackColor;

    uint32_t c = strip.getPixelColor(j);
    curColor.rgb[0] = (uint8_t)(c >> 16),
    curColor.rgb[1] = (uint8_t)(c >>  8),
    curColor.rgb[2] = (uint8_t)c;

    colorBuffer[j] = curColor;

  }
  
  float millisSinceLastUpdate = float(now - frameLastUpdatedAt);
  
   for (uint8_t i=0; i < numParticles; i++) {
    Particle& p = particles[i];

    // Units must agree
    // acc unit is mm per second per second
    // vel unit is mm per second
    // pos unit is mm
    // then translate meters to pixel position
    
    p.acc = (attractorPosition - p.pos) * springForce * p.springForceScale;
    p.vel = (p.vel + p.acc) * damping;
    p.pos += (p.vel * millisSinceLastUpdate) / 1000;

    float pos = p.pos;
    if (pos > 1.0) {
      pos = (int(pos*100) % 1) / 100.0;
    }
    
    if (pos < 0.0) {
      pos = (int(-pos*100) % 1) / 100.0;
    }

    // set color
    uint8_t pixelPosition = (pos * numPixels);
    float absVel = fabs(p.vel);
   
    for (uint8_t ic=0; ic < 3; ic++) {
//      uint8_t c = (float(p.color.rgb[ic]) * max(0.75, min(1.5, absVel)) / numParticles);

      uint8_t scaledColor = float(p.color.rgb[ic]) / numParticles * particleColorScale;
      uint8_t c = max(15, min(255, (scaledColor * (absVel*100.0))));
      uint8_t c2 = c * 0.33;
//      uint8_t c3 = c2 * 0.33;

      colorBuffer[pixelPosition].rgb[ic] += c;
//      colorBuffer[pixelPosition].rgb[ic] += scaledColor;

      if (p.vel > 0 && pixelPosition > 0) {
        colorBuffer[pixelPosition-1].rgb[ic] += c2;
//        if (pixelPosition > 1) {
//          colorBuffer[pixelPosition-2].rgb[ic] += c3;
//        }
      }
     
      if (p.vel < 0 && pixelPosition < numPixels-2) {
        colorBuffer[pixelPosition+1].rgb[ic] += c2;
//        if (pixelPosition < numPixels-3) {
//          colorBuffer[pixelPosition+2].rgb[ic] += c3;
//        }
      }
    }

  }

  for (uint16_t j=0; j < numPixels; j++) {
    strip.setPixelColor(j, strip.Color(
      colorBuffer[j].rgb[0], colorBuffer[j].rgb[1], colorBuffer[j].rgb[2]));
  }

//  strip.show();
  frameLastUpdatedAt = millis();
}

float squared(float x) { return x*x; }

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
Color wheel(uint8_t index) {
  index = 255 - index;
  Color c;
  if (index < 85) {
    c.rgb[0] = 255 - index * 3;
    c.rgb[1] = 0;
    c.rgb[2] = index * 3;
  }
  else if (index < 170) {
    index -= 85;
    c.rgb[0] = 0;
    c.rgb[1] = index * 3;
    c.rgb[2] = 255 - index * 3;
  }
  else {
    index -= 170;
    c.rgb[0] = index * 3;
    c.rgb[1] = 255 - index * 3;
    c.rgb[2] = 0;
  }
  return c;
}


