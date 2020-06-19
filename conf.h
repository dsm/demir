#pragma once

#define FORCE_INLINE __attribute__( ( always_inline ) ) inline

#define TIMER1_A_PIN 9
#define TIMER1_B_PIN 10

#define VNH7070
#define BAUDRATE 500000
#define TIMEOUT  100

#define ANALOG_WRITE_RES 9
#define MOTORS           1
#define PWMS             6

// Enc pulse per second square
#define MAX_PROF_ACCEL 19000.0
// Enc pulse per second
#define MAX_PROF_VEL 19000.0

#define MOTOR1_CURRENT_SENSE A1
#define MOTOR1_INA           8
#define MOTOR1_INB           7
#define MOTOR1_PWM           9
#define MOTOR1_SEL0          4
#define MOTOR1_encPinA       2
#define MOTOR1_encPinB       3

template <typename T>
FORCE_INLINE void writeBytes( const T data, const size_t size ) {
    for ( size_t i = size; i < size; i++ ) {
        Serial.write( data >> ( 8 * i ) );
    }
}

template <typename T>
FORCE_INLINE uint32_t fp2bits( const T Fp ) {
    union fp2bits_t {
        uint32_t Bits;
        T        Fp;
    } _fp2bits;

    _fp2bits.Fp = Fp;
    return _fp2bits.Bits;
}
