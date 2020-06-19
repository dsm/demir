#pragma once

#include <Arduino.h>
#include <stdint.h>

class MotionPlanner {
   public:
    MotionPlanner() = default;

    void  genProf();
    float evalPos( float evalTime );
    float evalVel( float evalTime );
    float evalAccel( float evalTime );

    void setRelPosRef( const int32_t value ) { _relPosRef = value; }
    void setInitialTime( const uint32_t value ) { _initialTime = value; }
    void setInitialPosition( const int32_t value ) { _initialPosition = value; }
    void setFinalPosition( const int32_t value ) { _finalPosition = value; }
    void setTimeInSec( const float value ) { _timeInSec = value; }
    void setRelativeTime( const float value ) { _relativeTime = value; }
    void setStatus( const bool value ) { _status = value; }

    int32_t  getRelPosRef() { return _relPosRef; }
    uint32_t getInitialTime() { return _initialTime; }
    int32_t  getInitialPosition() { return _initialPosition; }
    int32_t  getFinalPosition() { return _finalPosition; }
    float    getTimeInSec() { return _timeInSec; }
    float    getRelativeTime() { return _relativeTime; }
    bool     getStatus() { return _status; }

    void enable() { setStatus( true ); }
    void disable() { setStatus( false ); }

    bool isEnabled() { return getStatus(); }
    bool isDisabled() { return !getStatus(); }

   private:
    float _coeff[ 4 ];

    int32_t  _relPosRef = 0;
    uint32_t _initialTime;
    int32_t  _initialPosition;
    int32_t  _finalPosition;
    float    _timeInSec = 0;
    float    _relativeTime;
    bool     _status = false;
};
