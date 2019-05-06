#include <Arduino.h>
#include <math.h>
#include "const.h"
#include "config.h"
#include "Multiservo.h"

// Knee joint phase offset
float phaseOffset[N_LEGS];
// Knee joint phase offset derivative
float dPhaseOffset[N_LEGS];
// Target knee joint phase offset derivative
float tPhaseOffset[N_LEGS];
// Shoulder joint amp
float amp[N_LEGS];
// Shoulder joint amp derivative
float dAmp[N_LEGS];
// Target shoulder joint amp
float tAmp[N_LEGS];

float shoulderJoints[N_LEGS];
float kneeJoints[N_LEGS];

double phase[N_LEGS];
double PHASE_OFFSET[] = {
    0, PI, 0, PI, 0, PI, 0
};

float theta[N_OSC_PLUS_ONE];
float phaseBias[N_OSC][N_OSC];

void updateGaitPhaseBias() {
  // 7 elements representing links:
  // (1,2) (1, 3) (2, 4) (3, 4) (3, 5) (4, 6) (5, 6)
  switch (gait) {
    case GAIT_METACHRONAL: {
      theta[0] = 1.0f;
      theta[1] = 1.0f / 3.0f;
      theta[2] = 1.0f / 3.0f;
      theta[3] = 1.0f;
      theta[4] = 1.0f / 3.0f;
      theta[5] = 1.0f / 3.0f;
      theta[6] = 1.0f;
    }

      break;
    case GAIT_RIPPLE: {
      theta[0] = -1.0f;
      theta[1] = -3.0f / 2.0f;
      theta[2] = 1.0f / 2.0f;
      theta[3] = 1.0f;
      theta[4] = 1.0f / 2.0f;
      theta[5] = 1.0f / 2.0f;
      theta[6] = 1.0f;
    };
      break;
    case GAIT_TRIPOD: {
      theta[0] = 1.0f;
      theta[1] = 1.0f;
      theta[2] = -1.0f;
      theta[3] = -1.0f;
      theta[4] = -1.0f;
      theta[5] = 1.0f;
      theta[6] = 1.0f;
    };
      break;
    default: {
      theta[0] = 0;
      theta[1] = 0;
      theta[2] = 0;
      theta[3] = 0;
      theta[4] = 0;
      theta[5] = 0;
      theta[6] = 0;
    };
  }
}

void setGait() {
  updateGaitPhaseBias();

  for (float &i : theta) {
    i *= M_PI;
  }

  phaseBias[0][0] = 0;
  phaseBias[0][1] = theta[0];
  phaseBias[0][2] = theta[1];
  phaseBias[0][3] = 0;
  phaseBias[0][4] = 0;
  phaseBias[0][5] = 0;

  phaseBias[1][0] = theta[0];
  phaseBias[1][1] = 0;
  phaseBias[1][2] = 0;
  phaseBias[1][3] = theta[2];
  phaseBias[1][4] = 0;
  phaseBias[1][5] = 0;

  phaseBias[2][0] = theta[1];
  phaseBias[2][1] = 0;
  phaseBias[2][2] = 0;
  phaseBias[2][3] = theta[3];
  phaseBias[2][4] = theta[4];
  phaseBias[2][5] = 0;

  phaseBias[3][0] = 0;
  phaseBias[3][1] = theta[2];
  phaseBias[3][2] = theta[3];
  phaseBias[3][3] = 0;
  phaseBias[3][4] = 0;
  phaseBias[3][5] = theta[5];

  phaseBias[4][0] = 0;
  phaseBias[4][1] = 0;
  phaseBias[4][2] = theta[4];
  phaseBias[4][3] = 0;
  phaseBias[4][4] = 0;
  phaseBias[4][5] = theta[6];

  phaseBias[5][0] = 0;
  phaseBias[5][1] = 0;
  phaseBias[5][2] = 0;
  phaseBias[5][3] = theta[5];
  phaseBias[5][4] = theta[6];
  phaseBias[5][5] = 0;
}

Multiservo servo[N_SERVOS];

unsigned long timeStamp = 0;
unsigned long offsetInit = 0;

void setup() {
  Serial.begin(9600);

  timeStamp = millis();
  offsetInit = millis() + 10000;

  for (int i = 0; i < N_SERVOS; ++i) {
    servo[i].attach(i);
    servo[i].write((int) (initServo[i] * (SERVO_MAX - SERVO_MIN) / 180.f + SERVO_MIN));
  }

  for (double &i : phase) {
    i = random(1000) / 100.0f;
  }

  setGait();
}

void GetKneeJointPhaseOffset() {
  for (int i = 0; i < N_LEGS; ++i) {
    tPhaseOffset[i] = (i % 2 == 0 ? left : right);
  }
}

void GetShoulderJointAmp() {
  for (int i = 0; i < N_LEGS; ++i) {
    tAmp[i] = abs(i % 2 == 0 ? left : right);
  }
}

float lerp(float a, float b, float x) {
  return a + x * (b - a);
}

float clamp(float x, float a, float b) {
  return min(b, max(a, x));
}

float clamp01(float x) {
  return min(1.0f, max(.0f, x));
}

const float WEIGHT[N_OSC][N_OSC] = {
    //1, 2, 3, 4, 5, 6
    {0, 1, 1, 0, 0, 0},//1
    {1, 0, 0, 1, 0, 0},//2
    {1, 0, 0, 1, 1, 0},//3
    {0, 1, 1, 0, 0, 1},//4
    {0, 0, 1, 0, 0, 1},//5
    {0, 0, 0, 1, 1, 0} //6
};

float newPhase[N_OSC];

void CPGUpdate(float dt) {
  for (int i = 0; i < N_OSC; ++i) {
    float dPhase = freq;
    for (int j = 0; j < N_OSC; ++j) {
      dPhase += WEIGHT[i][j] * sin(phase[j] - phase[i] - phaseBias[i][j]);
    }
    newPhase[i] = phase[i] + dt * dPhase;
  }
  for (int i = 0; i < N_OSC; ++i) {
    phase[i] = newPhase[i];
  }
}

void updateLocomotion() {
  unsigned long newTimeStamp = millis();
  float dt = (float) (newTimeStamp - timeStamp) / 1000.0;

  CPGUpdate(dt);
  GetKneeJointPhaseOffset();
  GetShoulderJointAmp();

  int i = 0;
  for (i = 0; i < N_LEGS; ++i) {
    // Critically damped second order differential equation
    // http://mathproofs.blogspot.ca/2013/07/critically-damped-spring-smoothing.html
    float ddPhaseOffset =
        smoothing * ((smoothing / 4.0) * (tPhaseOffset[i] - phaseOffset[i]) - dPhaseOffset[i]); // 2nd derivative
    dPhaseOffset[i] = dPhaseOffset[i] + dt * ddPhaseOffset; // 1st derivative
    phaseOffset[i] = phaseOffset[i] + dt * dPhaseOffset[i];

    float ddAmp = smoothing * ((smoothing / 4.0) * (tAmp[i] - amp[i]) - dAmp[i]); // 2nd derivative
    dAmp[i] = dAmp[i] + dt * ddAmp; // 1st derivative
    amp[i] = amp[i] + dt * dAmp[i];
  }

  for (i = 0; i < N_LEGS; ++i) {
    float osc = sin(phase[i] + PHASE_OFFSET[i]);

    osc = (1.0f / (1.0f + exp(-sig * osc)) - .5f) * 2.0f;

    shoulderJoints[i] = lerp(shoulderAngleLimMin, shoulderAngleLimMax, .5f * (osc + 1.0f)) * amp[i];
  }

  // legs stops moving if the shoulder does
  float a = 1.0f - exp(-4.0f * clamp01(max(abs(left), abs(right))));

  for (i = 0; i < N_LEGS; ++i) {
    float osc = sin(phase[i] * 1.0 + phaseOffset[i] * PI / 2.0f);

    osc = (1.0f / (1.0f + exp(-sig * osc)) - .5) * 2.0f; // sigmoid

    kneeJoints[i] = lerp(kneeAngleLimMin, kneeAngleLimMax, .5f * (osc + 1.0f)) * a;
  }

  timeStamp = newTimeStamp;
}

void updateServos() {
  for (int i = 0; i < N_LEGS; ++i) {
    int shoulderAngle =
        (int) ((servoCalibration[i] + shoulderJoints[i]) * (SERVO_MAX - SERVO_MIN) / 180.f + SERVO_MIN);

    int kneeAngle =
        (int) ((servoCalibration[N_LEGS + i] + kneeJoints[i]) * (SERVO_MAX - SERVO_MIN) / 180.f + SERVO_MIN);

    servo[i].write(shoulderAngle);
    servo[N_LEGS + i].write(kneeAngle);
  }
}

void calibrateLoop() {
  for (int i = 0; i < N_LEGS; ++i) {
    servo[i].write(110);
    delay(1000);
    servo[i].write(90);
    delay(1000);
  }

  for (int i = 0; i < N_LEGS; ++i) {
    servo[i].write(90);
  }

  delay(3000);
}

void moveLoop() {
  updateLocomotion();

  if (millis() > offsetInit) {
    updateServos();
  }
}

int pos = 0;

void loop() {


//  int calb = map(analogRead(A0), 0, 1023, SERVO_MIN, SERVO_MAX);
//  servo[3].write(calb);
//  Serial.println(90 - map(calb, SERVO_MIN, SERVO_MAX, 0, 180));
//  Serial.println((int)(calb * (SERVO_MAX - SERVO_MIN) / 180.f + SERVO_MIN));

  moveLoop();

//  for (pos = 0; pos <= 180; pos += 1) { // in steps of 1 degree
//    for (Multiservo s : servo) {
//      s.write(pos);
//    }
//    delay(15);
//  }
//  for (pos = 180; pos >= 0; pos -= 1) {
//    for (Multiservo s : servo) {
//      s.write(pos);
//    }
//    delay(15);
//  }

//  servo[0].write(0);
//  delay(1000);
//  servo[0].write(180);
//  delay(1000);

//  sig = sigMod / 1000.f;



  delay(50);
}
