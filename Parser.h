#pragma once

#include <Arduino.h>

#include "DEMIR.h"
#include "Motor.h"
#include "com_def.h"
#include "conf.h"

#define MAX_DATA_NUM 5

class Parser {
   public:
    Parser() = default;
    void readCommands();
    void proccessCommands();
    void proccessOneArgCmd( const uint8_t _Motor );
    void proccessTwoArgCmd( const uint8_t _Motor );
    void proccessThreeArgCmd( const uint8_t _Motor );
    void proccessFourArgCmd( const uint8_t _Motor );

    bool    _Status               = 0;
    float   _Data[ MAX_DATA_NUM ] = { 0 };
    uint8_t _Index                = 0;

    struct Commands {
        uint8_t command     = 0;
        float   params[ 3 ] = { 0 };
    } _cmd;

    // bool             logEnable                    = false;
    // uint8_t          logWhat                      = LOG_POSITION;
    // bool             sendLogToMatlab              = false;
    // uint16_t         logCounter                   = 0;
    // uint16_t         logSize                      = LOG_SIZE_MAX;
    // uint32_t         logForMatlab[ LOG_SIZE_MAX ] = { 0 };
};
