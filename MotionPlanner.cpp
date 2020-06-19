#include "MotionPlanner.h"

// pdf'ten
// geldi//http://ocw.snu.ac.kr/sites/default/files/NOTE/Chap07_Trajectory%20generation.pdf
void MotionPlanner::genProf() {
    _coeff[ 0 ] = 12 * ( ( float ) ( _finalPosition - _initialPosition ) ) / ( 2 * pow( _timeInSec, 5 ) );
    _coeff[ 1 ] = -30 * ( ( float ) ( _finalPosition - _initialPosition ) ) / ( 2 * pow( _timeInSec, 4 ) );
    _coeff[ 2 ] = 20 * ( ( float ) ( _finalPosition - _initialPosition ) ) / ( 2 * pow( _timeInSec, 3 ) );
    _coeff[ 3 ] = ( float ) _initialPosition;
}
// 5. dereceden poz. polinomunu verilen değer için hesaplar
float MotionPlanner::evalPos( float evalTime ) {
    return ( _coeff[ 0 ] * pow( evalTime, 5 ) + _coeff[ 1 ] * pow( evalTime, 4 ) + _coeff[ 2 ] * pow( evalTime, 3 )
             + _coeff[ 3 ] );
}
// 4. dereceden hız polinomunu verilen değer için hesaplar
float MotionPlanner::evalVel( float evalTime ) {
    return ( 5.0 * _coeff[ 0 ] * pow( evalTime, 4 ) + 4.0 * _coeff[ 1 ] * pow( evalTime, 3 )
             + 3.0 * _coeff[ 2 ] * pow( evalTime, 2 ) );
}
// 3. dereceden ivme polinomunu verilen değer için hesaplar
float MotionPlanner::evalAccel( float evalTime ) {
    return ( 20.0 * _coeff[ 0 ] * pow( evalTime, 3 ) + 12.0 * _coeff[ 1 ] * pow( evalTime, 2 )
             + 6.0 * _coeff[ 2 ] * evalTime );
}