/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "m5_module_fan.hpp"

void M5ModuleFan::writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    _wire->beginTransmission(addr);
    _wire->write(reg);
    for (int i = 0; i < length; i++) {
        _wire->write(*(buffer + i));
    }
    _wire->endTransmission();
#if defined MODULE_FAN_DEBUG
    Serial.print("Write bytes: [");
    Serial.print(addr);
    Serial.print(", ");
    Serial.print(reg);
    Serial.print(", ");
    for (int i = 0; i < length; i++) {
        Serial.print(buffer[i]);
        if (i < length - 1) {
            Serial.print(", ");
        }
    }
    Serial.println("]");
#else
#endif
}

void M5ModuleFan::readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t index = 0;
    _wire->beginTransmission(addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(addr, length);
    for (int i = 0; i < length; i++) {
        buffer[index++] = _wire->read();
    }
#if defined MODULE_FAN_DEBUG
    Serial.print("Read bytes: [");
    Serial.print(addr);
    Serial.print(", ");
    Serial.print(reg);
    Serial.print(", ");
    for (int i = 0; i < length; i++) {
        Serial.print(buffer[i]);
        if (i < length - 1) {
            Serial.print(", ");
        }
    }
    Serial.println("]");
#else
#endif
}

void M5ModuleFan::acquireMutex()
{
    while (mutexLocked) {
        delay(1);
    }
    mutexLocked = true;
}

void M5ModuleFan::releaseMutex()
{
    mutexLocked = false;
}

bool M5ModuleFan::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed)
{
    _wire  = wire;
    _addr  = addr;
    _sda   = sda;
    _scl   = scl;
    _speed = speed;
    _wire->begin(_sda, _scl);
    _wire->setClock(_speed);
    delay(10);
    _wire->beginTransmission(_addr);
    uint8_t error = _wire->endTransmission();
    if (error == 0) {
        return true;
    } else {
        return false;
    }
}
void M5ModuleFan::setWorkStatus(module_fan_work_t newStatus)
{
    acquireMutex();
    uint8_t reg = MODULE_FAN_CONTROL_REG;
    writeBytes(_addr, reg, (uint8_t *)&newStatus, 1);
    releaseMutex();
}
uint8_t M5ModuleFan::getWorkStatus(void)
{
    acquireMutex();
    uint8_t temp = 0;
    uint8_t reg  = MODULE_FAN_CONTROL_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 1);
    releaseMutex();
    return temp;
}
void M5ModuleFan::setPWMFrequency(pwm_frequency_t newFreq)
{
    acquireMutex();
    uint8_t reg = MODULE_FAN_PWM_FREQUENCY_REG;
    writeBytes(_addr, reg, (uint8_t *)&newFreq, 1);
    releaseMutex();
}
uint8_t M5ModuleFan::getPWMFrequency(void)
{
    acquireMutex();
    uint8_t temp = 0;
    uint8_t reg  = MODULE_FAN_PWM_FREQUENCY_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 1);
    releaseMutex();
    return temp;
}
void M5ModuleFan::setPWMDutyCycle(uint8_t newDutyCycle)
{
    acquireMutex();
    if (newDutyCycle > 100) {
        newDutyCycle = 100;
    }
    uint8_t reg = MODULE_FAN_PWM_DUTY_CYCLE_REG;
    writeBytes(_addr, reg, (uint8_t *)&newDutyCycle, 1);
    releaseMutex();
}
uint8_t M5ModuleFan::getPWMDutyCycle(void)
{
    acquireMutex();
    uint8_t temp = 0;
    uint8_t reg  = MODULE_FAN_PWM_DUTY_CYCLE_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 1);
    releaseMutex();
    return temp;
}
uint16_t M5ModuleFan::getRPM(void)
{
    acquireMutex();
    uint16_t temp = 0;
    uint8_t reg   = MODULE_FAN_RPM_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 2);
    releaseMutex();
    return temp;
}
uint16_t M5ModuleFan::getSignalFrequency(void)
{
    acquireMutex();
    uint16_t temp = 0;
    uint8_t reg   = MODULE_FAN_SIGNAL_FREQUENCY_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 2);
    releaseMutex();
    return temp;
}
void M5ModuleFan::saveToFlash(void)
{
    acquireMutex();
    uint8_t temp = 1;
    uint8_t reg  = MODULE_FAN_SAVE_FLASH_REG;
    writeBytes(_addr, reg, (uint8_t *)&temp, 1);
    releaseMutex();
}
uint8_t M5ModuleFan::getFirmwareVersion(void)
{
    acquireMutex();
    uint8_t temp = 0;
    uint8_t reg  = MODULE_FAN_FIRMWARE_VERSION_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 1);
    releaseMutex();
    return temp;
}
uint8_t M5ModuleFan::setI2CAddress(uint8_t newAddr)
{
    acquireMutex();
    newAddr     = constrain(newAddr, I2C_ADDR_MIN, I2C_ADDR_MAX);
    uint8_t reg = MODULE_FAN_I2C_ADDRESS_REG;
    writeBytes(_addr, reg, (uint8_t *)&newAddr, 1);
    _addr = newAddr;
    releaseMutex();
    return _addr;
}
uint8_t M5ModuleFan::getI2CAddress(void)
{
    acquireMutex();
    uint8_t temp;
    uint8_t reg = MODULE_FAN_I2C_ADDRESS_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 1);
    releaseMutex();
    return temp;
}