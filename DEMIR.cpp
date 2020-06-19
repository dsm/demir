#include "DEMIR.h"
DEMIR Demir;

void DEMIR::init() {
    Serial.begin( BAUDRATE );
    while ( !Serial ) {
    }
    Serial.setTimeout( TIMEOUT );
    // Serial.println( F( "DEMIR v1.1.1" ) );

    Motors[ 0 ]->Driver.init( MOTOR1_INA, MOTOR1_INB, MOTOR1_PWM, MOTOR1_SEL0, MOTOR1_CURRENT_SENSE );

    Motors[ 0 ]->Encoder.setup(
        MOTOR1_encPinA, MOTOR1_encPinB, []() { Motors[ 0 ]->Encoder.ChA_ISR(); },
        []() { Motors[ 0 ]->Encoder.ChB_ISR(); } );

    Motors[ 0 ]->posPID.setGains( 0.05, 0.01, 0.0 );
    Motors[ 0 ]->velPID.setGains( 0.03, 0.05, 0.0 );

    setupPFC_PWM();

    // LED Test
    pinMode( STATUS_LED_BLUE, OUTPUT );
    pinMode( STATUS_LED_RED, OUTPUT );
    statusLedMode( FLIP_FLOP_LED );

    // Serial.println( "Ready!" );
}

void DEMIR::run() {
    currentTime = millis();  //Şimdiki zaman
    deltaT      = currentTime - oldTime;

    if ( deltaT >= loopRate ) {
        printTick++;
        oldTime = currentTime;
        updateCurrentPosition();
        updateCurrentVelocity();
        updateMotorCurrent();
        if ( state() ) {
            switch ( getOperatingMode() ) {
                case MANUAL_MODE:
                    manuelMode();
                    break;
                case POSITION_MODE:
                    positionMode();
                    break;
                case PROFILE_POSITION_MODE:
                    profilePositionMode();
                    break;
                case VELOCITY_MODE:
                    velocityMode();
                    break;
                case PROFILE_VELOCITY_MODE:
                    // Do nothing
                    break;
                case CURRENT_MODE:
                    // Do nothing
                    break;
                case HOMING_MODE:
                    // Do nothing
                    break;
                case MODEL_BASE_DESIGN_MODE:
                    ModelBasedDesignMode();
                    break;
            }
        } else {
            // zeroOutput();
        }
        update();
    }
}

void DEMIR::update() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            Motors[ _Motor ]->Driver.uToPWM( u[ _Motor ] );
        } else {
            Motors[ _Motor ]->Driver.uToPWM( 0 );
        }
    }
}

void DEMIR::zeroOutput() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        u[ _Motor ]       = 0;
        uManuel[ _Motor ] = 0;
    }
}

void DEMIR::setuManuel( const int16_t uManuel, const uint8_t _Motor ) { this->uManuel[ _Motor ] = uManuel; }

void DEMIR::manuelMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        u[ _Motor ] = uManuel[ _Motor ];
    }
}

void DEMIR::positionMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            u[ _Motor ] = Motors[ _Motor ]->posPID.compute(
                ( float ) ( positionReference[ _Motor ] - currentPosition[ _Motor ] ),
                ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor
        }
    }
}

void DEMIR::profilePositionMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            if ( Motors[ _Motor ]->Planner.isEnabled() )  // tetiklenmiş mi?
            {
                Motors[ _Motor ]->Planner.setRelativeTime(
                    ( ( float ) currentTime - ( float ) Motors[ _Motor ]->Planner.getInitialTime() ) );  // relativeTime
                if ( Motors[ _Motor ]->Planner.getRelativeTime() < Motors[ _Motor ]->Planner.getTimeInSec() * 1000 ) {
                    positionReference[ _Motor ]
                        = Motors[ _Motor ]->Planner.evalPos( Motors[ _Motor ]->Planner.getRelativeTime() / 1000 );
                } else {                                  // profil bitmiş demektir
                    Motors[ _Motor ]->Planner.disable();  // tetiği durdur
                    positionReference[ _Motor ] = Motors[ _Motor ]->Planner.getFinalPosition();  // :) çakallık
                    Serial.println( F( "!" ) );  // bittiğine dair seri porttan ! karakteri gönderir
                }
            }
            u[ _Motor ] = Motors[ _Motor ]->posPID.compute(
                ( float ) ( positionReference[ _Motor ] - currentPosition[ _Motor ] ),
                ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor}
        }
    }
}

void DEMIR::velocityMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            u[ _Motor ] = Motors[ _Motor ]->velPID.compute(
                ( float ) ( velocityReference[ _Motor ] - filteredVelocity[ _Motor ] ),
                ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor
        }
    }
}

void DEMIR::profileVelocityMode() {}

void DEMIR::ModelBasedDesignMode() {
    if ( designModeFlag ) {
        if ( designModeCounter < designStepSize ) {
            designModeCounter++;
            u[ motorIndex ] = 0;
        } else if ( designStepSize <= designModeCounter && designModeCounter < designStepSize * 2 ) {
            designModeCounter++;
            u[ motorIndex ] = uManuel[ motorIndex ];
            if ( designModeCounter > ( ( designStepSize * 2 ) - 20 ) ) {
                velocityLogBuf[ velBuffIndex++ ] = filteredVelocity[ motorIndex ];
                if ( velBuffIndex >= 20 ) {
                    auto tempVel = 0.0f;
                    for ( uint8_t i = 0; i < 20; i++ ) {
                        tempVel += velocityLogBuf[ i ];
                    }
                    velBuffIndex                  = 0;
                    measuredK[ measuredKIndex++ ] = tempVel / 20.0f;
                    Serial.print( measuredKIndex );
                    Serial.print( ". MeasuredK " );
                    Serial.print( measuredK[ measuredKIndex - 1 ] );
                    Serial.print( "\t" );
                }
            }
        } else if ( designStepSize <= designModeCounter * 2 && designModeCounter < designStepSize * 3 ) {
            designModeCounter++;
            u[ motorIndex ] = 0;
        } else if ( designStepSize * 3 <= designModeCounter && designModeCounter < designStepSize * 4 ) {
            designModeCounter++;
            u[ motorIndex ] = -uManuel[ motorIndex ];
            if ( -filteredVelocity[ motorIndex ] >= ( measuredK[ measuredKIndex - 1 ] * 0.632 ) ) {
                measuredTau[ measuredTauIndex++ ] = ( ( designModeCounter - 1 ) - designStepSize * 3 ) * loopRate;
                Serial.print( measuredTauIndex );
                Serial.print( ". MeasuredTau " );
                Serial.println( measuredTau[ measuredTauIndex - 1 ] );
                designModeCounter = 0;
                if ( measuredTauIndex >= 20 && measuredKIndex >= 20 ) {
                    designModeFlag = false;
                    auto tempK     = 0.0f;
                    auto tempTau   = 0.0f;
                    for ( uint8_t i = 0; i < 20; i++ ) {
                        tempK += measuredK[ i ];
                        tempTau += measuredTau[ i ];
                    }
                    estimatedK[ motorIndex ]   = tempK / 20 / uManuel[ motorIndex ];
                    estimatedTau[ motorIndex ] = tempTau / 20 * 1e-3;
                    Serial.print( "estimatedK " );
                    Serial.print( estimatedK[ motorIndex ], 6 );
                    Serial.print( " \t" );
                    Serial.print( "estimatedTau " );
                    Serial.println( estimatedTau[ motorIndex ], 6 );
                    measuredKIndex   = 0;
                    measuredTauIndex = 0;
                }
            }
        }
    } else {
        u[ motorIndex ] = 0;
    }
}

void DEMIR::updateMotorCurrent() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            motorCurrent[ _Motor ] = Motors[ _Motor ]->Driver.getCurrent();
        }
    }
}

void DEMIR::updateCurrentPosition() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            currentPosition[ _Motor ] = Motors[ _Motor ]->Encoder.getPosition();
        }
    }
}

void DEMIR::updateCurrentPosition( const uint8_t _Motor ) {
    if ( Motors[ _Motor ]->Driver.isEnabled() ) {
        currentPosition[ _Motor ] = Motors[ _Motor ]->Encoder.getPosition();
    }
}

void DEMIR::updateCurrentVelocity() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motors[ _Motor ]->Driver.isEnabled() ) {
            // encPulse per second
            rawVelocity[ _Motor ]
                = float( currentPosition[ _Motor ] - previousPosition[ _Motor ] ) / float( deltaT ) * 1000.0f;

            // velocity filtering
            filteredVelocity[ _Motor ]
                = alpha * rawVelocity[ _Motor ] + ( 1 - alpha ) * previousFilteredVelocity[ _Motor ];

            previousPosition[ _Motor ]         = currentPosition[ _Motor ];
            previousFilteredVelocity[ _Motor ] = filteredVelocity[ _Motor ];
        }
    }
}

void DEMIR::updateCurrentVelocity( const uint8_t _Motor ) {
    if ( Motors[ _Motor ]->Driver.isEnabled() ) {
        // encPulse per second
        rawVelocity[ _Motor ]
            = float( currentPosition[ _Motor ] - previousPosition[ _Motor ] ) / float( deltaT ) * 1000.0f;

        // velocity filtering
        filteredVelocity[ _Motor ] = alpha * rawVelocity[ _Motor ] + ( 1 - alpha ) * previousFilteredVelocity[ _Motor ];

        previousPosition[ _Motor ]         = currentPosition[ _Motor ];
        previousFilteredVelocity[ _Motor ] = filteredVelocity[ _Motor ];
    }
}

void DEMIR::report() {
    if ( printTick >= ( printingPeriod / loopRate ) )  // printing period default 1000 ms
    {
        printTick = 0;
        for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
            longPosReport( _Motor );
            shortPosReport( _Motor );
            longVelReport( _Motor );
            shortVelReport( _Motor );
        }
    }
}

void DEMIR::longPosReport( const uint8_t _Motor ) {
    if ( longReportPos ) {
        Serial.print( F( "M-ID " ) );
        Serial.print( _Motor );
        Serial.print( F( " opMode " ) );
        Serial.print( getOperatingMode() );
        Serial.print( F( " driverState " ) );
        Serial.print( state() );
        Serial.print( F( " posRef " ) );
        Serial.print( positionReference[ _Motor ] );
        Serial.print( F( " pos " ) );
        Serial.print( Motors[ _Motor ]->Encoder.getPosition() );
        Serial.print( F( " error " ) );
        Serial.print( positionReference[ _Motor ] - Motors[ _Motor ]->Encoder.getPosition() );
        Serial.print( F( " u_p " ) );
        Serial.print( Motors[ _Motor ]->posPID.getUp() );
        Serial.print( F( " u_i " ) );
        Serial.print( Motors[ _Motor ]->posPID.getUi() );
        Serial.print( F( " u_d " ) );
        Serial.print( Motors[ _Motor ]->posPID.getUd() );
        Serial.print( F( " u " ) );
        Serial.print( u[ _Motor ] );
        Serial.print( F( " uManual " ) );
        Serial.print( uManuel[ _Motor ] );
        Serial.print( F( " deltaT " ) );
        Serial.print( deltaT );
        Serial.print( F( " Kp " ) );
        Serial.print( Motors[ _Motor ]->posPID.getKp(), 4 );
        Serial.print( F( " Ki " ) );
        Serial.print( Motors[ _Motor ]->posPID.getKi(), 4 );
        Serial.print( F( " Kd " ) );
        Serial.println( Motors[ _Motor ]->posPID.getKd(), 4 );
    }
}

void DEMIR::shortPosReport( const uint8_t _Motor ) {
    if ( shortReportPos ) {
        Serial.print( F( "M-ID " ) );
        Serial.print( _Motor );
        Serial.print( F( " opMode " ) );
        Serial.print( getOperatingMode() );
        Serial.print( F( " driverState " ) );
        Serial.print( state() );
        Serial.print( F( " pos " ) );
        Serial.print( Motors[ _Motor ]->Encoder.getPosition() );
        Serial.print( F( " current " ) );
        Serial.println( motorCurrent[ _Motor ], 10 );
    }
}

void DEMIR::longVelReport( const uint8_t _Motor ) {
    if ( longReportVel ) {
        Serial.print( F( "M-ID " ) );
        Serial.print( _Motor );
        Serial.print( F( " opMode " ) );
        Serial.print( getOperatingMode() );
        Serial.print( F( " driverState " ) );
        Serial.print( state() );
        Serial.print( F( " velRef " ) );
        Serial.print( velocityReference[ _Motor ] );
        Serial.print( F( " vel " ) );
        Serial.print( filteredVelocity[ _Motor ] );
        Serial.print( F( " error " ) );
        Serial.print( velocityReference[ _Motor ] - filteredVelocity[ _Motor ] );
        Serial.print( F( " u_p " ) );
        Serial.print( Motors[ _Motor ]->velPID.getUp() );
        Serial.print( F( " u_i " ) );
        Serial.print( Motors[ _Motor ]->velPID.getUi() );
        Serial.print( F( " u_d " ) );
        Serial.print( Motors[ _Motor ]->velPID.getUd() );
        Serial.print( F( " u " ) );
        Serial.print( u[ _Motor ] );
        Serial.print( F( " uManual " ) );
        Serial.print( uManuel[ _Motor ] );
        Serial.print( F( " deltaT " ) );
        Serial.print( deltaT );
        Serial.print( F( " Kp " ) );
        Serial.print( Motors[ _Motor ]->velPID.getKp(), 4 );
        Serial.print( F( " Ki " ) );
        Serial.print( Motors[ _Motor ]->velPID.getKi(), 4 );
        Serial.print( F( " Kd " ) );
        Serial.println( Motors[ _Motor ]->velPID.getKd(), 4 );
    }
}

void DEMIR::shortVelReport( const uint8_t _Motor ) {
    if ( shortReportVel ) {
        Serial.print( F( "M-ID " ) );
        Serial.print( _Motor );
        Serial.print( F( " opMode " ) );
        Serial.print( getOperatingMode() );
        Serial.print( F( " driverState " ) );
        Serial.print( state() );
        Serial.print( F( " vel " ) );
        Serial.print( filteredVelocity[ _Motor ], 4 );
        Serial.print( F( " current " ) );
        Serial.println( motorCurrent[ _Motor ], 10 );
    }
}

void DEMIR::reportMode( const uint8_t Mode ) {
    switch ( Mode ) {
        case LONG_POS_REPORT: {
            longReportPos  = true;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
        } break;
        case LONG_VEL_REPORT: {
            longReportPos  = false;
            longReportVel  = true;
            shortReportPos = false;
            shortReportVel = false;
        } break;
        case SHORT_POS_REPORT: {
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = true;
            shortReportVel = false;
        } break;
        case SHORT_VEL_REPORT: {
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = true;
        } break;
        case DISABLE_ALL_REPORT: {
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
        } break;
    }
}

void DEMIR::statusLedMode( const uint8_t Mode ) {
    switch ( Mode ) {
        case RED_LED_ON: {
            digitalWrite( STATUS_LED_RED, HIGH );
        } break;
        case RED_LED_OFF: {
            digitalWrite( STATUS_LED_RED, LOW );
        } break;
        case BLUE_LED_ON: {
            digitalWrite( STATUS_LED_BLUE, HIGH );
        } break;
        case BLUE_LED_OFF: {
            digitalWrite( STATUS_LED_BLUE, LOW );
        } break;
        case FLIP_FLOP_LED: {
            digitalWrite( STATUS_LED_BLUE, HIGH );
            delay( 300 );
            digitalWrite( STATUS_LED_BLUE, LOW );

            digitalWrite( STATUS_LED_RED, HIGH );
            delay( 300 );
            digitalWrite( STATUS_LED_RED, LOW );
        } break;
        case ALL_LED_ON: {
            digitalWrite( STATUS_LED_BLUE, HIGH );
            digitalWrite( STATUS_LED_RED, HIGH );
        } break;
        case ALL_LED_OFF: {
            digitalWrite( STATUS_LED_BLUE, LOW );
            digitalWrite( STATUS_LED_RED, LOW );
        } break;
    }
}
