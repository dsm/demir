#pragma once

#include <Arduino.h>

class PID {
   public:
    PID();

    float compute( const float err, const float deltaT );

    void enable();
    void disable();
    void setKp( const float kP );
    void setKi( const float kI );
    void setKd( const float kD );
    void setGains( const float kP = 0, const float kI = 0, const float kD = 0 );
    void setOutMinMax( const float outMin, const float outMax );

    float getU() const { return _u; }
    float getKp() const { return _kP; }
    float getKi() const { return _kI; }
    float getKd() const { return _kD; }
    float getUp() const { return _uP; }
    float getUi() const { return _uI; }
    float getUd() const { return _uD; }
    bool  getStatus() const { return _enabled; }

   private:
    bool  _enabled;
    float _u;
    float _kP;
    float _kI;
    float _kD;
    float _error;
    float _previousError;
    float _uP;
    float _uI;
    float _uD;
    float _outMax;
    float _outMin;
};