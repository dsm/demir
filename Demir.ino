#include <Arduino.h>

#include "DEMIR.h"
#include "Parser.h"

Parser SerialParser;

void setup() { Demir.init(); }

void loop() {
    Demir.run();
    Demir.report();
    SerialParser.proccessCommands();
}
