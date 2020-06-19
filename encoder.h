#pragma once

#include <Arduino.h>
#include <stdint.h>

class encoder {
   public:
    encoder() = default;

    void setup( const uint8_t pinENCA, const uint8_t pinENCB, void ( *isrA )(), void ( *isrB )() );

    int32_t getPosition();
    void    resetPosition() { _encPos = 0; }

    void setUserPosDir( const bool dir ) { _userPosDir = dir; }
    bool getUserPosDir() const { return _userPosDir; }

    void ChA_ISR();
    void ChB_ISR();

   private:
    uint8_t _pinENCA = 0xFF;
    uint8_t _pinENCB = 0xFF;

    volatile int32_t _encPos     = 0;
    bool             _userPosDir = false;

    void setupInterruptPins( const uint8_t pinENCA, const uint8_t pinENCB );
    void setupInterruptISR( void ( *isrA )(), void ( *isrB )() );
};
