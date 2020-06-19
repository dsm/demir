#include "Parser.h"

void Parser::readCommands() {
    static int8_t _correctionFlag = -1;
    static int    dummy;

    while ( Serial.available() > 0 )  // Seri port bufferında bilgi varmı
    {
        if ( _correctionFlag == -1 ) delayMicroseconds( 1000 );
        _correctionFlag = 1;
        if ( _Index < 5 ) {
            _Data[ _Index++ ] = Serial.parseFloat();
            dummy             = Serial.read();

            if ( dummy == '\r' || dummy == '\n' ) {
                _correctionFlag = 2;

                /*for ( size_t i = 0; i < 5; i++ ) {
                    Serial.print( _Data[ i ] );
                    Serial.print( '\t' );
                  }
                  Serial.println();*/

                if ( Serial.available() > 0 ) {
                    Serial.read();
                }
            }
        } else {
            _correctionFlag = 2;
            Serial.parseFloat();
        }
    }

    if ( _correctionFlag == ( int8_t ) 1 ) {
        _Data[ --_Index ] = -1;
        _correctionFlag   = -1;
        _Status           = 1;
    } else if ( _correctionFlag == ( int8_t ) 2 ) {
        _correctionFlag = -1;
        _Status         = 1;
    }
}

void Parser::proccessCommands() {
    readCommands();

    if ( !_Status || !( ( int8_t ) _Data[ 0 ] < MOTORS ) || !( ( int8_t ) _Data[ 0 ] >= 0 ) ) {
        _Index  = 0;
        _Status = 0;
        return;
    }

    switch ( _Index ) {
        case 2: {
            _cmd.command = ( uint8_t ) _Data[ 1 ];
            proccessOneArgCmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
        case 3: {
            _cmd.command     = ( uint8_t ) _Data[ 1 ];
            _cmd.params[ 0 ] = _Data[ 2 ];
            proccessTwoArgCmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
        case 4: {
            _cmd.command     = ( uint8_t ) _Data[ 1 ];
            _cmd.params[ 0 ] = _Data[ 2 ];
            _cmd.params[ 1 ] = _Data[ 3 ];
            proccessThreeArgCmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
        case 5: {
            _cmd.command     = ( uint8_t ) _Data[ 1 ];
            _cmd.params[ 0 ] = _Data[ 2 ];
            _cmd.params[ 1 ] = _Data[ 3 ];
            _cmd.params[ 2 ] = _Data[ 4 ];
            proccessFourArgCmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
    }
    _Index  = 0;
    _Status = 0;
}

void Parser::proccessOneArgCmd( const uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_PRINT_LONG_VEL_REPORT: {
            Demir.reportMode( LONG_VEL_REPORT );
        } break;
        case COM_PRINT_SHORT_VEL_REPORT: {
            Demir.reportMode( SHORT_VEL_REPORT );
        } break;
        case COM_PRINT_LONG_POS_REPORT: {
            Demir.reportMode( LONG_POS_REPORT );
        } break;
        case COM_PRINT_SHORT_POS_REPORT: {
            Demir.reportMode( SHORT_POS_REPORT );
        } break;
        case COM_PRINT_NO_REPORT: {
            Demir.reportMode( DISABLE_ALL_REPORT );
        } break;
        case COM_SEND_DATA_FOR_MATLAB: {
            // sendLogToMatlab = true;
        } break;
        case COM_PRINT_HELP: {
            Demir.reportMode( DISABLE_ALL_REPORT );
            // print_help();
        } break;
        case COM_GET_POSITION_LONG: {
            auto tempPos = Motors[ _Motor ]->Encoder.getPosition();
            writeBytes( tempPos, sizeof( tempPos ) );
        } break;
        case COM_LOG_POSITION: {
            // logWhat = LOG_POSITION;
        } break;
        case COM_LOG_CURRENT: {
            // logWhat = LOG_CURRENT;
        } break;
        case COM_GET_LOG_SIZE_INTEGER: {
            // writeBytes( logSize, sizeof( logSize ) );
        } break;
        case COM_GET_LOOPRATE_UINT8: {
            Serial.write( Demir.getLoopRate() );
        } break;
        case COM_CHANGE_USER_POS_DIR: {
            Demir.reportMode( DISABLE_ALL_REPORT );

            Motors[ _Motor ]->posPID.disable();
            Motors[ _Motor ]->velPID.disable();
            Demir.setuManuel( 0, _Motor );

            Motors[ _Motor ]->Encoder.resetPosition();

            if ( Motors[ _Motor ]->Encoder.getUserPosDir() )
                Motors[ _Motor ]->Encoder.setUserPosDir( false );
            else
                Motors[ _Motor ]->Encoder.setUserPosDir( true );

            if ( Motors[ _Motor ]->Driver.getDir() )
                Motors[ _Motor ]->Driver.setDir( false );
            else
                Motors[ _Motor ]->Driver.setDir( true );
        } break;
        case COM_RESET_ENCODER_POS: {
            Demir.reportMode( DISABLE_ALL_REPORT );
            Motors[ _Motor ]->posPID.disable();
            Motors[ _Motor ]->velPID.disable();
            Demir.setuManuel( 0, _Motor );
            Motors[ _Motor ]->Encoder.resetPosition();
        } break;
        case COM_GET_POS_GAINS: {
            uint32_t temp;
            temp = fp2bits( Motors[ _Motor ]->posPID.getKp() );
            writeBytes( temp, sizeof( uint32_t ) );

            temp = fp2bits( Motors[ _Motor ]->posPID.getKi() );
            writeBytes( temp, sizeof( uint32_t ) );

            temp = fp2bits( Motors[ _Motor ]->posPID.getKd() );
            writeBytes( temp, sizeof( uint32_t ) );
        } break;
        case COM_GET_VEL_GAINS: {
            uint32_t temp;
            temp = fp2bits( Motors[ _Motor ]->velPID.getKp() );
            writeBytes( temp, sizeof( uint32_t ) );

            temp = fp2bits( Motors[ _Motor ]->velPID.getKi() );
            writeBytes( temp, sizeof( uint32_t ) );

            temp = fp2bits( Motors[ _Motor ]->velPID.getKd() );
            writeBytes( temp, sizeof( uint32_t ) );
        } break;
    }
}

void Parser::proccessTwoArgCmd( const uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_SET_DRIVER_STATE: {
            switch ( ( uint8_t ) _cmd.params[ 0 ] ) {
                case DRIVER_DISABLE:  // disable driver
                    Demir.disable();
                    Demir.zeroOutput();
                    Demir.disableMotor( _Motor );
                    Motors[ _Motor ]->Planner.disable();
                    Demir.statusLedMode( ALL_LED_OFF );
                    break;
                case DRIVER_ENABLE:  // enable driver
                    Demir.enable();
                    Demir.zeroOutput();
                    Demir.enableMotor( _Motor );
                    Motors[ _Motor ]->Planner.disable();
                    Demir.statusLedMode( ALL_LED_OFF );
                    for ( uint8_t i = 0; i < ( _Motor + 1 ); i++ ) {
                        Demir.statusLedMode( FLIP_FLOP_LED );
                    }
                    delay( 100 );
                    Demir.statusLedMode( RED_LED_ON );
                    break;
            }
        } break;
        case COM_SET_OPERATING_MODE: {
            Demir.setuManuel( 0, _Motor );
            switch ( ( uint8_t ) _cmd.params[ 0 ] ) {
                case MANUAL_MODE:
                    Demir.setOperatingMode( MANUAL_MODE );
                    Motors[ _Motor ]->posPID.disable();
                    Motors[ _Motor ]->velPID.disable();
                    break;
                case POSITION_MODE:
                    Demir.setOperatingMode( POSITION_MODE );
                    Demir.setPositionReference( Motors[ _Motor ]->Encoder.getPosition(), _Motor );
                    Motors[ _Motor ]->posPID.enable();
                    Motors[ _Motor ]->velPID.disable();
                    Demir.statusLedMode( BLUE_LED_ON );
                    break;
                case PROFILE_POSITION_MODE:
                    Motors[ _Motor ]->Planner.enable();
                    Demir.setOperatingMode( PROFILE_POSITION_MODE );
                    Demir.setPositionReference( Motors[ _Motor ]->Encoder.getPosition(), _Motor );
                    Motors[ _Motor ]->posPID.enable();
                    Motors[ _Motor ]->velPID.disable();
                    Demir.statusLedMode( BLUE_LED_ON );
                    break;
                case VELOCITY_MODE:
                    Demir.setOperatingMode( VELOCITY_MODE );
                    Motors[ _Motor ]->posPID.disable();
                    Motors[ _Motor ]->velPID.enable();
                    break;
                case PROFILE_VELOCITY_MODE:
                    Demir.setOperatingMode( PROFILE_VELOCITY_MODE );
                    break;
                case CURRENT_MODE:
                    Demir.setOperatingMode( CURRENT_MODE );
                    break;
                case HOMING_MODE:
                    Demir.setOperatingMode( HOMING_MODE );
                    break;
                case MODEL_BASE_DESIGN_MODE:
                    Demir.reportMode( DISABLE_ALL_REPORT );
                    Demir.statusLedMode( ALL_LED_OFF );
                    Demir.disableAllMotor();
                    Demir.enableMotor( _Motor );
                    Demir.setMotorIndex( _Motor );
                    Motors[ _Motor ]->posPID.disable();
                    Motors[ _Motor ]->velPID.disable();
                    Demir.setOperatingMode( MODEL_BASE_DESIGN_MODE );
                    for ( uint8_t i = 0; i < 2; i++ ) {
                        Demir.statusLedMode( ALL_LED_ON );
                        delay( 150 );
                        Demir.statusLedMode( ALL_LED_OFF );
                        delay( 150 );
                    }
                    delay( 50 );
                    Demir.statusLedMode( BLUE_LED_ON );
                    break;
            }
        } break;
        case COM_REL_MAX_PROF_POS_REF: {
            Motors[ _Motor ]->Planner.setRelPosRef( ( int32_t ) _cmd.params[ 0 ] );
            Motors[ _Motor ]->Planner.setInitialPosition( Motors[ _Motor ]->Encoder.getPosition() );

            Motors[ _Motor ]->Planner.setFinalPosition(
                ( Motors[ _Motor ]->Planner.getInitialPosition() + Motors[ _Motor ]->Planner.getRelPosRef() ) );

            Motors[ _Motor ]->Planner.genProf();

            // 6000 darbe öteye 0.7 saniye tatlı geçiş, o yüzden ref
            Motors[ _Motor ]->Planner.setTimeInSec(
                sqrt( abs( Motors[ _Motor ]->Planner.getRelPosRef() ) / 6000.0 * 0.7 ) );

            // 6000 darbe 0.7 saniye için max hız 16070
            if ( ( ( abs( Motors[ _Motor ]->Planner.getRelPosRef() ) / 6000.0 )
                   / ( Motors[ _Motor ]->Planner.getTimeInSec() / 0.7 ) * 16070.0 )
                 > MAX_PROF_VEL ) {
                Motors[ _Motor ]->Planner.setTimeInSec( ( abs( Motors[ _Motor ]->Planner.getRelPosRef() ) / 6000.0 )
                                                        / ( MAX_PROF_VEL / 0.7 ) * 16070 );
            }

            Motors[ _Motor ]->Planner.genProf();

            float maxComputedAccel
                = Motors[ _Motor ]->Planner.evalAccel( Motors[ _Motor ]->Planner.getTimeInSec() / 4.73 );

            if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                Motors[ _Motor ]->Planner.setStatus( true );
            else {
                Serial.print( F( "<" ) );
                Serial.print( _Motor );
                Serial.print( F( " ?>" ) );
            }

            Motors[ _Motor ]->Planner.setInitialTime( millis() );
        } break;
        case COM_ABS_MAX_PROF_POS_REF: {
            Motors[ _Motor ]->Planner.setFinalPosition( ( int32_t ) _cmd.params[ 0 ] );
            Motors[ _Motor ]->Planner.setInitialPosition( Motors[ _Motor ]->Encoder.getPosition() );
            Motors[ _Motor ]->Planner.setRelPosRef(
                ( Motors[ _Motor ]->Planner.getFinalPosition() - Motors[ _Motor ]->Planner.getInitialPosition() ) );
            Motors[ _Motor ]->Planner.setTimeInSec(
                sqrt( abs( Motors[ _Motor ]->Planner.getRelPosRef() ) / 6000.0 * 0.7 ) );

            if ( ( ( abs( Motors[ _Motor ]->Planner.getRelPosRef() ) / 6000.0 )
                   / ( Motors[ _Motor ]->Planner.getTimeInSec() / 0.7 ) * 16070.0 )
                 > MAX_PROF_VEL ) {
                Motors[ _Motor ]->Planner.setTimeInSec( ( abs( Motors[ _Motor ]->Planner.getRelPosRef() ) / 6000.0 )
                                                        / ( MAX_PROF_VEL / 0.7 ) * 16070 );
            }

            Motors[ _Motor ]->Planner.genProf();

            float maxComputedAccel
                = Motors[ _Motor ]->Planner.evalAccel( Motors[ _Motor ]->Planner.getTimeInSec() / 4.73 );

            if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                Motors[ _Motor ]->Planner.setStatus( true );
            else
                Serial.println( F( "?" ) );

            Motors[ _Motor ]->Planner.setInitialTime( millis() );
        } break;
        case COM_REL_VEL_REF: {
            int32_t temp = Demir.getVelocityReference( _Motor ) + ( int32_t ) _cmd.params[ 0 ];
            Demir.setVelocityReference( temp, _Motor );
        } break;
        case COM_ABS_VEL_REF: {
            Demir.setVelocityReference( ( int32_t ) _cmd.params[ 0 ], _Motor );
        } break;
        case COM_REL_POS_REF: {
            int32_t temp = Demir.getPositionReference( _Motor ) + ( int32_t ) _cmd.params[ 0 ];
            Demir.setPositionReference( temp, _Motor );
        } break;
        case COM_ABS_POS_REF: {
            Demir.setPositionReference( ( int32_t ) _cmd.params[ 0 ], _Motor );
        } break;
        case COM_SET_U: {
            int16_t uTemp = constrain( ( int16_t ) _cmd.params[ 0 ], -255, 255 );
            Demir.setuManuel( uTemp, _Motor );
            // logEnable = true;
        } break;
        case COM_TEST_MOTOR_CONNECTION: {
            Demir.reportMode( DISABLE_ALL_REPORT );

            Motors[ _Motor ]->posPID.disable();
            Motors[ _Motor ]->velPID.disable();

            Demir.setuManuel( 0, _Motor );

            uint8_t uTemp = constrain( ( int16_t ) _cmd.params[ 0 ], 0, 255 );

            int32_t tempPos = Motors[ _Motor ]->Encoder.getPosition();

            Motors[ _Motor ]->Driver.uToPWM( uTemp );
            delay( 100 );
            Motors[ _Motor ]->Driver.uToPWM( 0 );
            if ( ( Motors[ _Motor ]->Encoder.getPosition() - tempPos ) > 0 )
                Serial.println( F( "passed" ) );
            else {
                if ( Motors[ _Motor ]->Driver.getDir() )
                    Motors[ _Motor ]->Driver.setDir( false );
                else
                    Motors[ _Motor ]->Driver.setDir( true );
                Serial.println( F( "First failed but corrected automatically" ) );
            }
            delay( 100 );
            Motors[ _Motor ]->Driver.uToPWM( uTemp );
            delay( 100 );
            Motors[ _Motor ]->Driver.uToPWM( 0 );
        } break;
        case COM_SET_LOOPRATE: {
            uint8_t temp = ( uint8_t ) _cmd.params[ 0 ];
            if ( temp > 0 ) Demir.setLoopRate( temp );
        } break;
        case COM_SET_PRINTING_PERIOD: {
            uint16_t temp = ( uint16_t ) _cmd.params[ 0 ];
            if ( temp > 0 ) Demir.setPrintingPeriod( temp );
        } break;
    }
}

void Parser::proccessThreeArgCmd( const uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_REL_PROF_POS_REF: {
            Motors[ _Motor ]->Planner.setRelPosRef( ( int32_t ) _cmd.params[ 0 ] );
            Motors[ _Motor ]->Planner.setTimeInSec( _cmd.params[ 1 ] );

            if ( Motors[ _Motor ]->Planner.getTimeInSec() > 0 ) {
                Motors[ _Motor ]->Planner.setInitialPosition( Motors[ _Motor ]->Encoder.getPosition() );
                Motors[ _Motor ]->Planner.setFinalPosition(
                    ( Motors[ _Motor ]->Planner.getInitialPosition() + Motors[ _Motor ]->Planner.getRelPosRef() ) );
                Motors[ _Motor ]->Planner.genProf();

                float maxComputedAccel
                    = Motors[ _Motor ]->Planner.evalAccel( Motors[ _Motor ]->Planner.getTimeInSec() / 4.73 );

                if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                    Motors[ _Motor ]->Planner.setStatus( true );
                else
                    Serial.println( F( "?" ) );

                Motors[ _Motor ]->Planner.setInitialTime( millis() );
            }
        } break;
        case COM_ABS_PROF_POS_REF: {
            Motors[ _Motor ]->Planner.setFinalPosition( ( int32_t ) _cmd.params[ 0 ] );
            Motors[ _Motor ]->Planner.setTimeInSec( _cmd.params[ 1 ] );

            if ( Motors[ _Motor ]->Planner.getTimeInSec() > 0 ) {
                Motors[ _Motor ]->Planner.setInitialPosition( Motors[ _Motor ]->Encoder.getPosition() );

                Motors[ _Motor ]->Planner.genProf();

                float maxComputedAccel
                    = Motors[ _Motor ]->Planner.evalAccel( Motors[ _Motor ]->Planner.getTimeInSec() / 4.73 );

                if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                    Motors[ _Motor ]->Planner.setStatus( true );
                else
                    Serial.println( F( "?" ) );

                Motors[ _Motor ]->Planner.setInitialTime( millis() );
            }
        } break;
        case COM_SET_ESTIMATION_PARAMETERS: {
            int16_t uTemp = constrain( ( int16_t ) _cmd.params[ 0 ], -255, 255 );
            Demir.setuManuel( uTemp, _Motor );
            auto tempStepSize = ( int16_t ) _cmd.params[ 1 ] * Demir.getLoopRate();
            Demir.setDesignStepSize( tempStepSize );
            Demir.enableDesignMode();
        } break;
    }
}

void Parser::proccessFourArgCmd( const uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_SET_POS_GAINS: {
            Motors[ _Motor ]->posPID.setGains( _cmd.params[ 0 ], _cmd.params[ 1 ], _cmd.params[ 2 ] );
        } break;
        case COM_SET_VEL_GAINS: {
            Motors[ _Motor ]->velPID.setGains( _cmd.params[ 0 ], _cmd.params[ 1 ], _cmd.params[ 2 ] );
        } break;
    }
}
