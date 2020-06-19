#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "conf.h"

class vnh7070 {
   private:
    bool _motorDir = false;
    bool _status   = false;

    uint8_t _pinINA    = 0xFF;
    uint8_t _pinINB    = 0xFF;
    uint8_t _pinPWM    = 0xFF;
    uint8_t _pinSEL0   = 0xFF;
    uint8_t _pinCSENSE = 0xFF;

    void analogWrite16( uint8_t pinPWM, uint16_t val );

   public:
    vnh7070() = default;

    void init( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE );

    void uToPWM( float uIn );

    void setDir( bool dir ) { this->_motorDir = dir; }
    bool getDir() { return this->_motorDir; }

    void setStatus( bool status ) { this->_status = status; }
    bool getStatus() { return _status; }

    void enable() { setStatus( true ); }
    void disable() { setStatus( false ); }

    bool isEnabled() { return getStatus(); }
    bool isDisabled() { return getStatus(); }

    float getCurrent();
};