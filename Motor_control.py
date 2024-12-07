# ------------------------------------------------------------------------------
# Filename: Motor_control.py
# Author: Arjun
# License: This code is provided for personal and educational use. Redistribution and use in source and binary forms are permitted, provided that the above notice and this permission notice are included.
# ------------------------------------------------------------------------------
# Description: Python script to control 3 DC motors using the PCA9685 PWM driver with a Raspberry Pi / Pico / Pico2.
# ------------------------------------------------------------------------------
import time
from machine import Pin, I2C

# I2C setup (using GP0 and GP1 for SDA and SCL)
i2c = I2C(0, scl=Pin(1), sda=Pin(0))  # GP0 (SDA) and GP1 (SCL) for I2C(0)
address = 0x40  # Default I2C address for PCA9685

# Register addresses for PCA9685
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09

# Function to write a byte to the PCA9685
def write_byte(reg, value):
    i2c.writeto(address, bytearray([reg, value]))

# Function to read a byte from the PCA9685
def read_byte(reg):
    return i2c.readfrom_mem(address, reg, 1)[0]

# Function to initialize the PCA9685
def init_pca9685():
    # Set the PCA9685 to sleep mode to reset
    write_byte(PCA9685_MODE1, 0x10)  # Sleep mode
    time.sleep(0.1)
    write_byte(PCA9685_MODE1, 0x00)  # Normal mode
    time.sleep(0.1)

# Function to set PWM frequency
def set_pwm_frequency(freq):
    prescale_val = int(25000000.0 / 4096.0 / freq - 1)
    current_mode = read_byte(PCA9685_MODE1)
    write_byte(PCA9685_MODE1, current_mode | 0x10)  # Sleep mode
    write_byte(PCA9685_PRESCALE, prescale_val)  # Set prescaler
    write_byte(PCA9685_MODE1, current_mode)  # Wake up from sleep mode
    time.sleep(0.1)

# Function to set the PWM duty cycle for a motor
def set_pwm(channel, on_time, off_time):
    # Set the ON and OFF time for the given PWM channel
    write_byte(PCA9685_LED0_ON_L + 4 * channel, on_time & 0xFF)
    write_byte(PCA9685_LED0_ON_H + 4 * channel, on_time >> 8)
    write_byte(PCA9685_LED0_OFF_L + 4 * channel, off_time & 0xFF)
    write_byte(PCA9685_LED0_OFF_H + 4 * channel, off_time >> 8)

# Function to control the first motor's speed and direction
def set_motor1_speed(Lpwm, Rpwm):
    print(f"Motor 1 - Setting Lpwm: {Lpwm}, Rpwm: {Rpwm}")
    if Lpwm > 0 and Rpwm == 0:
        # Motor 1 Forward: Set Lpwm to move forward
        set_pwm(15, 0, Lpwm)  # Forward (Left Motor)
        set_pwm(14, 0, 0)     # Stop Reverse (Right Motor)
        print("Motor 1 moving forward")
    elif Rpwm > 0 and Lpwm == 0:
        # Motor 1 Reverse: Set Rpwm to move reverse
        set_pwm(15, 0, 0)     # Stop Forward (Left Motor)
        set_pwm(14, 0, Rpwm)  # Reverse (Right Motor)
        print("Motor 1 moving reverse")
    else:
        # Stop motor: Set both to 0
        set_pwm(15, 0, 0)
        set_pwm(14, 0, 0)
        print("Motor 1 stopped")

# Function to control the second motor's speed and direction
def set_motor2_speed(Lpwm, Rpwm):
    print(f"Motor 2 - Setting Lpwm: {Lpwm}, Rpwm: {Rpwm}")
    if Lpwm > 0 and Rpwm == 0:
        # Motor 2 Forward: Set Lpwm to move forward
        set_pwm(1, 0, Lpwm)  # Forward (Left Motor)
        set_pwm(0, 0, 0)     # Stop Reverse (Right Motor)
        print("Motor 2 moving forward")
    elif Rpwm > 0 and Lpwm == 0:
        # Motor 2 Reverse: Set Rpwm to move reverse
        set_pwm(1, 0, 0)     # Stop Forward (Left Motor)
        set_pwm(0, 0, Rpwm)  # Reverse (Right Motor)
        print("Motor 2 moving reverse")
    else:
        # Stop motor: Set both to 0
        set_pwm(1, 0, 0)
        set_pwm(0, 0, 0)
        print("Motor 2 stopped")

# Initialize the PCA9685 and set the PWM frequency
init_pca9685()
set_pwm_frequency(50)  # Set PWM frequency to 50Hz (standard for motors)

try:
    while True:
        # Test motor 1 forward
        print("Testing Motor 1 forward")
        set_motor1_speed(1500, 0)
        time.sleep(2)

        # Test motor 1 reverse
        print("Testing Motor 1 reverse")
        set_motor1_speed(0, 1500)
        time.sleep(2)

        # Stop both motors
        set_motor1_speed(0, 0)
        set_motor2_speed(0, 0)
        time.sleep(2)
        
        # Test motor 2 forward
        print("Testing Motor 2 forward")
        set_motor2_speed(1500, 0)
        time.sleep(2)

        # Test motor 2 reverse
        print("Testing Motor 2 reverse")
        set_motor2_speed(0, 1500)
        time.sleep(2)

        # Stop both motors
        set_motor1_speed(0, 0)
        set_motor2_speed(0, 0)
        time.sleep(2)

except KeyboardInterrupt:
    print("Program interrupted")
    # Stop motors on exit
    set_motor1_speed(0, 0)
    set_motor2_speed(0, 0)

