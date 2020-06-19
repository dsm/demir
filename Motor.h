#pragma once

#include <Arduino.h>

#include "MotionPlanner.h"
#include "PID.h"
#include "encoder.h"
#include "vnh7070.h"

class Motor {
   public:
    Motor() = default;

    vnh7070       Driver;
    encoder       Encoder;
    PID           posPID;
    PID           velPID;
    MotionPlanner Planner;
};

extern Motor  Motor_1;
extern Motor *Motors[ MOTORS ];

/*template <class DriverIC, class Enc, class Controller, class ProfileGen>
class Motor{
   public:
    Motor() = default;
    DriverIC   Driver;
    Enc        Encoder;
    Controller posController;
    Controller velController;
    ProfileGen Planner;

    void setSpeed( const float speed );

    void setDir( const bool dir );
    bool getDir();

    void enable();
    void disable();
    bool isEnabled();
    bool isDisabled();

    int32_t getPosition();
    void    resetPosition();
    void    setUserPosDir( const bool dir );
    bool    getUserPosDir();
};*/