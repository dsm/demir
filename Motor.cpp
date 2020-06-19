#include "Motor.h"
Motor Motor_1;

Motor *Motors[ MOTORS ] = { &Motor_1 };

/*template <class DriverIC, class Enc, class Controller, class ProfileGen>
void Motor<DriverIC, Enc, Controller, ProfileGen>::setSpeed( const float speed ) {
    Driver.uToPWM( speed );
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
void Motor<DriverIC, Enc, Controller, ProfileGen>::setDir( const bool dir ) {
    Driver.setMotorDir( dir );
}

// setSpeed(speed,MOTOR_1) like that
template <class DriverIC, class Enc, class Controller, class ProfileGen>
bool Motor<DriverIC, Enc, Controller, ProfileGen>::getDir() {
    return Driver.getMotorDir();
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
void Motor<DriverIC, Enc, Controller, ProfileGen>::enable() {
    Driver.setStatus( ENABLE );
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
void Motor<DriverIC, Enc, Controller, ProfileGen>::disable() {
    Driver.setStatus( DISABLE );
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
bool Motor<DriverIC, Enc, Controller, ProfileGen>::isEnabled() {
    return Driver.getStatus();
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
bool Motor<DriverIC, Enc, Controller, ProfileGen>::isDisabled() {
    return !Driver.getStatus();
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
int32_t Motor<DriverIC, Enc, Controller, ProfileGen>::getPosition() {
    return Encoder.getPosition();
}

// resetPosition(ENCODER_1) like that :)
template <class DriverIC, class Enc, class Controller, class ProfileGen>
void Motor<DriverIC, Enc, Controller, ProfileGen>::resetPosition() {
    Encoder.resetPosition();
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
void Motor<DriverIC, Enc, Controller, ProfileGen>::setUserPosDir( const bool dir ) {
    Encoder.setUserPosDir( dir );
}

template <class DriverIC, class Enc, class Controller, class ProfileGen>
bool Motor<DriverIC, Enc, Controller, ProfileGen>::getUserPosDir() {
    return Encoder.getUserPosDir();
}*/
