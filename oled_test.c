#include "oled_test.h"

#include "oled.h"

bool OLED_Test_initAndShowHelloWorld(void)
{
    return OLED_Init() && OLED_ShowMessage("HELLO WORLD");
}
