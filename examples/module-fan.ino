/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
 * @Hardwares: Basic v2.7+ Module Fan
 * @Dependent Library:
 * M5GFX@^0.2.3: https://github.com/m5stack/M5GFX
 * M5Unified@^0.2.2: https://github.com/m5stack/M5Unified
 * M5Module-Fan: https://github.com/m5stack/M5Module-Fan
 */

#include <M5Unified.h>
#include <m5_module_fan.hpp>

M5ModuleFan moduleFan;
uint8_t deviceAddr = MODULE_FAN_BASE_ADDR;
uint8_t dutyCycle  = 80;
void setup()
{
    M5.begin();
    Serial.begin(115200);
    while (!moduleFan.begin(&Wire1, deviceAddr, 21, 22, 400000)) {
        Serial.printf("Module FAN Init faile\r\n");
    }
    // Set the fan to rotate at 80% duty cycle
    moduleFan.setPWMDutyCycle(dutyCycle);
}

void loop()
{
    Serial.printf("\r\n");
    Serial.printf(" {\r\n");
    Serial.printf("    Work Status      : %d\r\n", moduleFan.getStatus());
    Serial.printf("    PWM  Frequency   : %d\r\n", moduleFan.getPWMFrequency());
    Serial.printf("    PWM  Duty Cycle  : %d\r\n", moduleFan.getPWMDutyCycle());
    Serial.printf("    RPM              : %d\r\n", moduleFan.getRPM());
    Serial.printf("    Signal Frequency : %d\r\n", moduleFan.getSignalFrequency());
    Serial.printf("    Firmware Version : %d\r\n", moduleFan.getFirmwareVersion());
    Serial.printf("         I2C Addrres : 0x%02X\r\n", moduleFan.getI2CAddress());
    Serial.printf("                             }\r\n");
    delay(500);
}
