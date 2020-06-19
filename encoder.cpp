#include "encoder.h"

void encoder::setup( const uint8_t pinENCA, const uint8_t pinENCB, void ( *isrA )(), void ( *isrB )() ) {
    setupInterruptPins( pinENCA, pinENCB );
    setupInterruptISR( isrA, isrB );
}

void encoder::setupInterruptPins( const uint8_t pinENCA, const uint8_t pinENCB ) {
    _pinENCA = pinENCA;
    _pinENCB = pinENCB;
    pinMode( _pinENCA, INPUT_PULLUP );
    pinMode( _pinENCB, INPUT_PULLUP );
}

void encoder::setupInterruptISR( void ( *isrA )(), void ( *isrB )() ) {
    attachInterrupt( digitalPinToInterrupt( _pinENCA ), isrA, CHANGE );
    attachInterrupt( digitalPinToInterrupt( _pinENCB ), isrB, CHANGE );
}

int32_t encoder::getPosition() { return ( getUserPosDir() ? _encPos : ( 0 - _encPos ) ); }

void encoder::ChA_ISR() { _encPos = _encPos + 1 - ( ( ( PIND & 0b00001000 ) >> 2 ) ^ ( ( PIND & 0b00000100 ) >> 1 ) ); }
void encoder::ChB_ISR() { _encPos = _encPos - 1 + ( ( ( PIND & 0b00001000 ) >> 2 ) ^ ( ( PIND & 0b00000100 ) >> 1 ) ); }
