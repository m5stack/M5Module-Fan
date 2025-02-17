/*
 *SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __M5_MODULE_FAN_H
#define __M5_MODULE_FAN_H

#include "Arduino.h"
#include "Wire.h"

// #define MODULE_FAN_DEBUG Serial // The corresponding serial port must be initialized before use
// This macro definition can be annotated without sending and receiving data prints
// Define the serial port you want to use, e.g., Serial1 or Serial2
#if defined MODULE_FAN_DEBUG
#define serialPrint(...)   MODULE_FAN_DEBUG.print(__VA_ARGS__)
#define serialPrintln(...) MODULE_FAN_DEBUG.println(__VA_ARGS__)
#define serialPrintf(...)  MODULE_FAN_DEBUG.printf(__VA_ARGS__)
#define serialFlush()      MODULE_FAN_DEBUG.flush()
#else
#endif

/**
 * @brief Default address of the device.
 *
 * This is the default address used to communicate with the device.
 * The value is 0x18.
 */
#define MODULE_FAN_BASE_ADDR (0x18)

/**
 * @brief Minimum valid I2C address.
 *
 * This is the minimum valid I2C address for the device. The I2C address must be greater than or equal to this value.
 */
#define I2C_ADDR_MIN (0x08)

/**
 * @brief Maximum valid I2C address.
 *
 * This is the maximum valid I2C address for the device. The I2C address must be less than or equal to this value.
 */
#define I2C_ADDR_MAX (0x77)

/**
 * @brief Module fan control register, used to enable or disable the fan.
 *
 * Setting the value to 0 disables the fan, and setting it to 1 enables the fan.
 * The register address is typically 0x00.
 */
#define MODULE_FAN_CONTROL_REG (0x00)

/**
 * @brief Control register for setting the frequency of the PWM waveform.
 *
 * The specific frequency setting can be referenced through the `pwm_frequency_t` enumeration.
 * The register address is typically 0x10.
 */
#define MODULE_FAN_PWM_FREQUENCY_REG (0x10)

/**
 * @brief Control register for setting the duty cycle of the PWM waveform.
 *
 * The duty cycle range is from 0 to 100, where 0 represents 0% duty cycle (off) and 100 represents 100% duty cycle
 * (fully on). The register address is typically 0x20.
 */
#define MODULE_FAN_PWM_DUTY_CYCLE_REG (0x20)

/**
 * @brief Register for recording the fan's RPM (revolutions per minute).
 *
 * This register is used to store the current speed (RPM) of the fan.
 * The register address is typically 0x30.
 */
#define MODULE_FAN_RPM_REG (0x30)

/**
 * @brief Register for recording the frequency of the fan's output signal.
 *
 * This register is used to store the frequency of the signal output from the fan's output pin.
 * The register address is typically 0x40.
 */
#define MODULE_FAN_SIGNAL_FREQUENCY_REG (0x40)

/**
 * @brief Register for saving configuration to internal flash memory.
 *
 * This register is used to save the configuration settings, such as FAN_CONTROL_REG, FAN_PWM_FREQUENCY_REG, and
 * FAN_PWM_DUTY_CYCLE_REG. Writing 1 to this register will trigger the saving of the configuration, ensuring the
 * settings are retained even after power loss. The register address is typically 0xF0.
 */
#define MODULE_FAN_SAVE_FLASH_REG (0xF0)

/**
 * @brief Firmware software version register.
 *
 * This register holds the firmware software version of the fan module.
 * The register address is typically 0xFE.
 */
#define MODULE_FAN_FIRMWARE_VERSION_REG (0xFE)

/**
 * @brief I2C address register of the device.
 *
 * This register holds the I2C address of the fan module.
 * The register address is typically 0xFF.
 */
#define MODULE_FAN_I2C_ADDRESS_REG (0xFF)

/**
 * @brief Enum for the fan module's working state.
 *
 * This enumeration defines the possible states for enabling or disabling the fan.
 * - MODULE_FAN_DISABLE: Disables the fan.
 * - MODULE_FAN_ENABLE: Enables the fan.
 */
typedef enum {
    MODULE_FAN_DISABLE = 0x00,  // Disable the fan
    MODULE_FAN_ENABLE  = 0x01,  // Enable the fan
} module_fan_work_t;            // Fan's working state

/**
 * @brief Enum for PWM waveform frequency types.
 *
 * This enumeration defines the available frequencies for PWM waveform generation.
 * - PWM_1KHZ: 1 kHz frequency.
 * - PWM_12KHZ: 12 kHz frequency.
 * - PWM_24KHZ: 24 kHz frequency.
 * - PWM_48KHZ: 48 kHz frequency.
 */
typedef enum {
    PWM_1KHZ  = 0x00,  // 1 kHz frequency
    PWM_12KHZ = 0x01,  // 12 kHz frequency
    PWM_24KHZ = 0x02,  // 24 kHz frequency
    PWM_48KHZ = 0x03,  // 48 kHz frequency
} pwm_frequency_t;     // PWM waveform frequency types

class M5ModuleFan {
public:
    /**
     * @brief Initializes the device with optional I2C settings.
     *
     * This function configures the I2C communication settings, allowing the user to specify
     * custom SDA and SCL pins as well as the I2C speed. If no parameters are provided, default values are used.
     * The device is initialized using the provided I2C settings, and it returns a success flag.
     *
     * @param wire   Pointer to the TwoWire object for I2C communication (default is &Wire).
     * @param addr   The I2C address of the device (default is -1, meaning use the default address).
     * @param sda    The SDA pin number (default is -1, meaning use the default SDA pin).
     * @param scl    The SCL pin number (default is -1, meaning use the default SCL pin).
     * @param speed  The I2C bus speed in Hz (default is 4000000L).
     *
     * @return True if initialization was successful, false otherwise.
     */
    bool begin(TwoWire* wire = &Wire, uint8_t addr = -1, uint8_t sda = -1, uint8_t scl = -1, uint32_t speed = 4000000L);

    /**
     * @brief Sets the working status of the fan.
     *
     * This function allows the user to set the fan's operational status, either enabling or disabling the fan
     * based on the provided `newStatus` argument.
     *
     * @param newStatus The desired fan working status, which can be:
     *                  - MODULE_FAN_DISABLE: Disable the fan.
     *                  - MODULE_FAN_ENABLE: Enable the fan.
     */
    void setWorkStatus(module_fan_work_t newStatus);

    /**
     * @brief Gets the current working status of the fan.
     *
     * This function retrieves the fan's current operational status.
     *
     * @return The current working status, which can be:
     *         - MODULE_FAN_DISABLE: Fan is disabled.
     *         - MODULE_FAN_ENABLE: Fan is enabled.
     */
    uint8_t getWorkStatus(void);

    /**
     * @brief Sets the frequency of the PWM waveform.
     *
     * This function allows the user to set the desired frequency for the PWM waveform.
     *
     * @param newFreq The desired PWM frequency, which can be one of the following:
     *                - PWM_1KHZ: 1 kHz frequency.
     *                - PWM_12KHZ: 12 kHz frequency.
     *                - PWM_24KHZ: 24 kHz frequency.
     *                - PWM_48KHZ: 48 kHz frequency.
     */
    void setPWMFrequency(pwm_frequency_t newFreq);

    /**
     * @brief Gets the current frequency of the PWM waveform.
     *
     * This function retrieves the current frequency set for the PWM waveform.
     *
     * @return The current PWM frequency, which can be one of the following:
     *         - PWM_1KHZ: 1 kHz frequency.
     *         - PWM_12KHZ: 12 kHz frequency.
     *         - PWM_24KHZ: 24 kHz frequency.
     *         - PWM_48KHZ: 48 kHz frequency.
     */
    uint8_t getPWMFrequency(void);

    /**
     * @brief Sets the duty cycle of the PWM waveform.
     *
     * This function allows the user to set the desired duty cycle for the PWM waveform.
     * The duty cycle value can range from 0 to 100.
     *
     * @param newDutyCycle The desired duty cycle (0-100).
     */
    void setPWMDutyCycle(uint8_t newDutyCycle);

    /**
     * @brief Gets the current duty cycle of the PWM waveform.
     *
     * This function retrieves the current duty cycle value set for the PWM waveform.
     * The duty cycle value can range from 0 to 100.
     *
     * @return The current duty cycle (0-100).
     */
    uint8_t getPWMDutyCycle(void);

    /**
     * @brief Gets the current RPM (Revolutions Per Minute) of the fan.
     *
     * This function retrieves the current speed of the fan in revolutions per minute (RPM).
     *
     * @return The current RPM of the fan.
     */
    uint16_t getRPM(void);

    /**
     * @brief Gets the frequency of the signal output by the fan's output pin.
     *
     * This function retrieves the current frequency of the signal output from the fan's pin.
     *
     * @return The current signal frequency of the fan's output pin.
     */
    uint16_t getSignalFrequency(void);

    /**
     * @brief Saves the configuration data to the internal flash.
     *
     * This function saves the current configuration settings to the internal flash memory,
     * ensuring that the settings are retained even after a power cycle.
     */
    void saveToFlash(void);

    /**
     * @brief Gets the firmware version number.
     *
     * This function retrieves the current software version number of the device's firmware.
     *
     * Please note that this operation involves writing to the device's flash memory, which may take more than 20ms to
     * complete.
     *
     * @return The current firmware version number.
     */
    uint8_t getFirmwareVersion(void);

    /**
     * @brief Sets the I2C device address.
     *
     * This function allows the user to set the I2C address for the device.
     * It returns the newly set I2C address. The valid I2C address range is from 0x08 to 0x77.
     * If the provided address is greater than the maximum (0x77), it will be set to 0x77.
     * If the provided address is less than the minimum (0x08), it will be set to 0x08.
     *
     * Please note that this operation involves writing to the device's flash memory, which may take more than 20ms to
     * complete.
     *
     * @param newAddr The new I2C address to be set for the device.
     *                The address should be within the range 0x08 to 0x77.
     *                If it is outside this range, the closest valid address will be used.
     *
     * @return The newly set I2C address.
     */
    uint8_t setI2CAddress(uint8_t newAddr);

    /**
     * @brief Gets the current I2C device address.
     *
     * This function retrieves the current I2C address of the device.
     *
     * @return The current I2C address of the device.
     */
    uint8_t getI2CAddress(void);

private:
    TwoWire* _wire;
    uint8_t _addr;
    uint8_t _scl;
    uint8_t _sda;
    uint32_t _speed;

    // Mutex flag for indicating whether the mutex is locked.
    bool mutexLocked = false;  // Mutex semaphore.

    /**
     * @brief Writes multiple bytes to a specified register.
     *
     * This function writes a sequence of bytes from the provided buffer
     * to the device located at the specified I2C address and register.
     *
     * @param addr   The I2C address of the device.
     * @param reg    The register address where the data will be written.
     * @param buffer A pointer to the data buffer that contains the bytes to be written.
     * @param length The number of bytes to write from the buffer.
     */
    void writeBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length);

    /**
     * @brief Reads multiple bytes from a specified register.
     *
     * This function reads a sequence of bytes from the device located at
     * the specified I2C address and register into the provided buffer.
     *
     * @param addr   The I2C address of the device.
     * @param reg    The register address from which the data will be read.
     * @param buffer A pointer to the data buffer where the read bytes will be stored.
     * @param length The number of bytes to read into the buffer.
     */
    void readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length);

    /**
     * @brief Acquires a mutex lock.
     *
     * This function attempts to acquire a mutex lock to ensure thread-safe access
     * to shared resources. It should be paired with a corresponding call to
     * releaseMutex() to prevent deadlocks.
     */
    void acquireMutex();

    /**
     * @brief Releases a mutex lock.
     *
     * This function releases a previously acquired mutex lock, allowing other
     * threads to access shared resources. It should only be called after
     * successfully acquiring the mutex with acquireMutex().
     */
    void releaseMutex();
};

#endif