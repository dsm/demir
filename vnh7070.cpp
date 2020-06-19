#include "vnh7070.h"

void vnh7070::init( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE ) {
    _pinINA    = pinINA;
    _pinINB    = pinINB;
    _pinPWM    = pinPWM;
    _pinSEL0   = pinSEL0;
    _pinCSENSE = pinCSENSE;

    digitalWrite( _pinINA, LOW );
    pinMode( _pinINA, OUTPUT );

    digitalWrite( _pinINB, LOW );
    pinMode( _pinINB, OUTPUT );

    digitalWrite( _pinSEL0, LOW );
    pinMode( _pinSEL0, OUTPUT );

    pinMode( _pinCSENSE, INPUT );
}

void vnh7070::uToPWM( const float uIn ) {
    analogWrite16( _pinPWM, abs( uIn ) );
    if ( uIn == 0 ) {
        digitalWrite( _pinINA, LOW );
        digitalWrite( _pinINB, LOW );
    } else if ( uIn > 0 ) {
        digitalWrite( _pinINA, _motorDir );
        digitalWrite( _pinINB, !_motorDir );
    } else {
        digitalWrite( _pinINA, !_motorDir );
        digitalWrite( _pinINB, _motorDir );
    }
}

void vnh7070::analogWrite16( uint8_t pinPWM, uint16_t val ) {
    if ( pinPWM == TIMER1_A_PIN )
        OCR1A = val;
    else if ( pinPWM == TIMER1_B_PIN )
        OCR1B = val;
}

float vnh7070::getCurrent() {
    auto Rsense = 1500;
    auto Vsense = ( analogRead( _pinCSENSE ) / 1023.0f ) * 5;  //*Rsense;
    auto Isense = ( Vsense / float( Rsense ) ) * 15000;

    return Isense;
}
