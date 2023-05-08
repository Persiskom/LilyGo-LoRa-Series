#include "loramac.h"
#include "boards.h"

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);
    Serial.println("GPS LilyGO v1");
    setupLMIC();
}

void loop()
{
    loopLMIC();
}