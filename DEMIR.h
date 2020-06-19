#pragma once

#include <Arduino.h>

#include "Motor.h"
#include "conf.h"

typedef enum : uint8_t { DRIVER_DISABLE = 0, DRIVER_ENABLE } driverState;

typedef enum : uint8_t {
    MANUAL_MODE = 0,
    POSITION_MODE,
    PROFILE_POSITION_MODE,
    VELOCITY_MODE,
    PROFILE_VELOCITY_MODE,
    CURRENT_MODE,
    HOMING_MODE,
    MODEL_BASE_DESIGN_MODE
} operatingMode;

typedef enum : uint8_t {
    LONG_POS_REPORT = 0,
    LONG_VEL_REPORT,
    SHORT_POS_REPORT,
    SHORT_VEL_REPORT,
    DISABLE_ALL_REPORT
} reportMode;

typedef enum : uint8_t {
    RED_LED_ON = 0,
    RED_LED_OFF,
    BLUE_LED_ON,
    BLUE_LED_OFF,
    FLIP_FLOP_LED,
    ALL_LED_ON,
    ALL_LED_OFF
} statusLedMode;

class DEMIR {
   public:
    DEMIR() = default;

    void init();
    void run();
    void update();
    void zeroOutput();
    void setuManuel( const int16_t uManuel, const uint8_t Motor );

    void manuelMode();
    void positionMode();
    void profilePositionMode();
    void velocityMode();
    void profileVelocityMode();
    void ModelBasedDesignMode();

    void updateMotorCurrent();
    void updateCurrentPosition();
    void updateCurrentPosition( const uint8_t _Motor );

    void updateCurrentVelocity();
    void updateCurrentVelocity( const uint8_t _Motor );

    void report();

    void longPosReport( const uint8_t _Motor );
    void shortPosReport( const uint8_t _Motor );
    void longVelReport( const uint8_t _Motor );
    void shortVelReport( const uint8_t _Motor );

    void reportMode( const uint8_t Mode );
    void statusLedMode( const uint8_t Mode );

    void enable() {
        _state  = DRIVER_ENABLE;
        _opMode = MANUAL_MODE;
    }

    void disable() {
        _state  = DRIVER_DISABLE;
        _opMode = MANUAL_MODE;
    }

    uint8_t state() const { return _state; }

    uint8_t isEnabled() { return _state; }
    uint8_t isDisabled() { return !_state; }

    void setDesignStepSize( const uint16_t stepSize ) { designStepSize = stepSize; }

    uint16_t getDesignStepSize() const { return designStepSize; }

    void    setMotorIndex( const uint8_t Motor ) { motorIndex = Motor; }
    uint8_t getMotorIndex() { return motorIndex; }

    void    setOperatingMode( const uint8_t mode ) { _opMode = mode; }
    uint8_t getOperatingMode() { return _opMode; }

    void    setLoopRate( const uint8_t loopRate ) { this->loopRate = loopRate; }
    uint8_t getLoopRate() const { return loopRate; }

    void    setVelocityReference( const int32_t ref, const uint8_t Motor ) { velocityReference[ Motor ] = ref; }
    int32_t getVelocityReference( const uint8_t Motor ) const { return velocityReference[ Motor ]; }

    void    setPositionReference( const int32_t ref, const uint8_t Motor ) { positionReference[ Motor ] = ref; }
    int32_t getPositionReference( const uint8_t Motor ) const { return positionReference[ Motor ]; }

    void     setPrintingPeriod( const uint16_t period ) { printingPeriod = period; }
    uint16_t getPrintingPeriod() const { return printingPeriod; }

    void enableAllMotor() {
        for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
            Motors[ _Motor ]->Driver.enable();
        }
    }

    void disableAllMotor() {
        for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
            Motors[ _Motor ]->Driver.disable();
        }
    }

    void enableMotor( const uint8_t _Motor ) {
        Motors[ _Motor ]->Driver.uToPWM( 0 );
        Motors[ _Motor ]->Driver.enable();
    }

    void disableMotor( const uint8_t _Motor ) {
        Motors[ _Motor ]->Driver.uToPWM( 0 );
        Motors[ _Motor ]->Driver.disable();
    }

    void enableDesignMode() { designModeFlag = 1; }

    void disableDesignMode() { designModeFlag = 0; }

    void setupPFC_PWM() {
        OCR1A = 0;
        OCR1B = 0;
        pinMode( TIMER1_A_PIN, OUTPUT );
        pinMode( TIMER1_B_PIN, OUTPUT );

        if ( ANALOG_WRITE_RES == 8 )
            ICR1 = 255;
        else if ( ANALOG_WRITE_RES == 9 )
            ICR1 = 511;
        else if ( ANALOG_WRITE_RES == 10 )
            ICR1 = 1023;

        TCCR1A = 0xA0;
        TCCR1B = 0x11;
    }

   private:
    uint8_t _state  = DRIVER_DISABLE;
    uint8_t _opMode = MANUAL_MODE;

    int16_t u[ MOTORS ]       = { 0 };
    int16_t uManuel[ MOTORS ] = { 0 };

    int32_t currentPosition[ MOTORS ]  = { 0 };
    int32_t previousPosition[ MOTORS ] = { 0 };

    int32_t positionReference[ MOTORS ] = { 0 };
    float   velocityReference[ MOTORS ] = { 0 };
    float   motorCurrent[ MOTORS ]      = { 0 };
    // for veloicty filtering
    float alpha                              = 0.2f;
    float rawVelocity[ MOTORS ]              = { 0 };
    float filteredVelocity[ MOTORS ]         = { 0 };
    float previousFilteredVelocity[ MOTORS ] = { 0 };

    uint8_t  designModeFlag = 0;
    uint8_t  motorIndex     = 0;
    uint16_t designStepSize = 0;

    uint16_t designModeCounter = 0;

    uint8_t measuredKIndex   = 0;
    uint8_t measuredTauIndex = 0;

    float measuredK[ 20 ]      = { 0 };
    float estimatedK[ MOTORS ] = { 0 };

    float measuredTau[ 20 ]      = { 0 };
    float estimatedTau[ MOTORS ] = { 0 };

    float   velocityLogBuf[ 20 ] = { 0 };
    uint8_t velBuffIndex         = 0;

    // for serial monitor report
    uint16_t printTick      = 0;
    uint16_t printingPeriod = 100;
    bool     longReportPos  = false;
    bool     longReportVel  = false;
    bool     shortReportPos = false;
    bool     shortReportVel = false;

    // for control loop timing
    unsigned long deltaT      = 0;
    unsigned long currentTime = 0;
    unsigned long oldTime     = 0;
    uint8_t       loopRate    = 3;

    // user led pins
    uint8_t STATUS_LED_BLUE = 5;
    uint8_t STATUS_LED_RED  = 6;
};

extern DEMIR Demir;
