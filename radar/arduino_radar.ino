// Experiment 6 : Ultrasonic radar with simple sensor fusion (Arduino Uno + HC-SR04 + MG996R / SG90)
// Configured for 20 cm Close-Range Detection
#include <Servo.h>
 
#define TRIG      7
#define ECHO      8
#define SERVO_PIN 6
 
const int   MIN_ANGLE = 15;
const int   MAX_ANGLE = 165;
const int   STEP      = 5;
const int   SAMPLES   = 5;         // echoes per angle
const float ALPHA     = 0.6;       // smoothing factor (0-1)
const float MAX_RANGE = 20.0;      // cm (Close-range 20cm)
 
Servo radar;
float filtered[181];               // stored (smoothed) range for every degree
int   angle = MIN_ANGLE;
int   dir   = STEP;
float nearestDist  = 999;
int   nearestAngle = 0;
 
float readOnce() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // 15000 us timeout ensures quick response for 20cm range (~58 us per cm)
  long t = pulseIn(ECHO, HIGH, 15000);
  if (t == 0) return MAX_RANGE;                  // no echo
  float d = t * 0.0343 / 2.0;
  return (d > MAX_RANGE) ? MAX_RANGE : d;
}
 
float medianDistance() {                         // fusion step 1: median filter
  float v[SAMPLES];
  for (int i = 0; i < SAMPLES; i++) { v[i] = readOnce(); delay(10); }
  for (int i = 1; i < SAMPLES; i++) {            // insertion sort
    float key = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  return v[SAMPLES / 2];
}
 
void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  radar.attach(SERVO_PIN);
  for (int i = 0; i <= 180; i++) filtered[i] = -1;   // -1 = no data yet
  radar.write(angle);
  delay(1000);
  Serial.println("angle,distance_cm,x_cm,y_cm");
}
 
void loop() {
  radar.write(angle);
  delay(80);                                     // let the servo settle
 
  float d = medianDistance();
  if (filtered[angle] < 0) filtered[angle] = d;  // fusion step 2: smoothing across sweeps
  else filtered[angle] = ALPHA * d + (1 - ALPHA) * filtered[angle];
 
  float r   = filtered[angle];
  float rad = angle * PI / 180.0;
  float x   = r * cos(rad);                      // fusion step 3: polar -> Cartesian
  float y   = r * sin(rad);
 
  Serial.print(angle); Serial.print(',');
  Serial.print(r, 1);  Serial.print(',');
  Serial.print(x, 1);  Serial.print(',');
  Serial.println(y, 1);
 
  if (r < nearestDist) { nearestDist = r; nearestAngle = angle; }
 
  angle += dir;
  if (angle >= MAX_ANGLE || angle <= MIN_ANGLE) {   // end of sweep
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
    dir = -dir;
    Serial.print("NEAREST,");
    Serial.print(nearestAngle); Serial.print(',');
    Serial.println(nearestDist, 1);
    nearestDist = 999;
  }
}
