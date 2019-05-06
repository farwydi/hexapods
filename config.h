//
// Created by farwydi on 02.05.2019.
//

#ifndef HEXOPODS__CONFIG_H_
#define HEXOPODS__CONFIG_H_

// tuples: (name, default, min, max)
// "sig", 1.0, 0.0, 6.0
// Импульс
float sig = 1.5f;

// "kneeAngleLimMin", -18.0, -90.0, 0.0
float kneeAngleLimMin = 150.f;
// "kneeAngleLimMax", 18.0, 0.0, 90.0)
float kneeAngleLimMax = 180.f;

// "shoulderAngleLimMin", -18.0, -30.0, 0.0
float shoulderAngleLimMin = 70.f;
// "shoulderAngleLimMax", 18.0, 0.0, 30.0
float shoulderAngleLimMax = 110.f;

// "freq", 1.0, 0.0, 12.0
// Скорость
float freq = 6.5f;

// "smoothing", 1.0, 0.5, 8.0
float smoothing = 3.0f;

int gait = GAIT_TRIPOD;

float left = 1.f;
float right = 1.f;

float initServo[N_SERVOS] = {
    90.f,
    90.f,
    90.f,
    90.f,
    90.f,
    90.f,


    180.f,
    180.f,
    180.f,
    180.f,
    180.f,
    180.f,
};

float servoCalibration[N_SERVOS] = {
    0.f,
    0.f,
    0.f,
    0.f,
    0.f,
    0.f,

    0.f, 0.f, 0.f, 0.f, 0.f, 0.f
};

#endif //HEXOPODS__CONFIG_H_
