# ------------------------------------------------------------------------------
# Filename: Motor_control.py
# Author: Charles Dickens
# License: This code is provided for personal and educational use. Redistribution and use in source and binary forms are permitted, provided that the above notice and this permission notice are included.
# ------------------------------------------------------------------------------
# Description: Python script to control 3 DC motors using the PCA9685 PWM driver with a Raspberry Pi / Pico / Pico2.
# ------------------------------------------------------------------------------
import time
from machine import Pin, I2C

# I2C setup (using GP0 for SDA and GP1 for SCL)
i2c = I2C(0, scl=Pin(1), sda=Pin(0))  # GP0 (SDA) and GP1 (SCL)
ADDRESS = 0x40  # Default I2C address for PCA9685

# PCA9685 register addresses
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09

# Helper functions
def write_byte(reg, value):
    """Write a byte to a specified register."""
    i2c.writeto(ADDRESS, bytearray([reg, value]))

def read_byte(reg):
    """Read a byte from a specified register."""
    return i2c.readfrom_mem(ADDRESS, reg, 1)[0]

def init_pca9685():
    """Initialize the PCA9685, resetting it to a known state."""
    write_byte(MODE1, 0x10)  # Sleep mode to reset
    time.sleep(0.1)
    write_byte(MODE1, 0x00)  # Normal mode
    time.sleep(0.1)

def set_pwm_frequency(freq):
    """Set the PWM frequency for the PCA9685."""
    prescale_val = int(25000000.0 / 4096.0 / freq - 1)
    current_mode = read_byte(MODE1)
    write_byte(MODE1, current_mode | 0x10)  # Sleep mode
    write_byte(PRESCALE, prescale_val)      # Set prescaler
    write_byte(MODE1, current_mode)         # Wake up from sleep mode
    time.sleep(0.1)

def set_pwm(channel, on_time, off_time):
    """Set the PWM duty cycle for the given channel."""
    write_byte(LED0_ON_L + 4 * channel, on_time & 0xFF)
    write_byte(LED0_ON_H + 4 * channel, on_time >> 8)
    write_byte(LED0_OFF_L + 4 * channel, off_time & 0xFF)
    write_byte(LED0_OFF_H + 4 * channel, off_time >> 8)

# Motor control functions
def set_motor_speed(channel_forward, channel_reverse, forward_pwm, reverse_pwm):
    """
    Set the speed and direction for a motor.
    
    Parameters:
    channel_forward: PWM channel for forward motion.
    channel_reverse: PWM channel for reverse motion.
    forward_pwm: PWM value for forward speed.
    reverse_pwm: PWM value for reverse speed.
    """
    if forward_pwm > 0 and reverse_pwm == 0:
        set_pwm(channel_forward, 0, forward_pwm)  # Move forward
        set_pwm(channel_reverse, 0, 0)            # Stop reverse
        print(f"Motor on channels {channel_forward}/{channel_reverse} moving forward")
    elif reverse_pwm > 0 and forward_pwm == 0:
        set_pwm(channel_forward, 0, 0)            # Stop forward
        set_pwm(channel_reverse, 0, reverse_pwm)  # Move reverse
        print(f"Motor on channels {channel_forward}/{channel_reverse} moving reverse")
    else:
        set_pwm(channel_forward, 0, 0)            # Stop forward
        set_pwm(channel_reverse, 0, 0)            # Stop reverse
        print(f"Motor on channels {channel_forward}/{channel_reverse} stopped")

# Main program setup
init_pca9685()
set_pwm_frequency(50)  # Set PWM frequency to 50Hz (standard for motors)

try:
    while True:
        # Move all motors forward for 5 seconds
        print("All motors moving forward")
        time.sleep(3)
        set_motor_speed(15, 14, 3000, 0)  # Motor 1
        set_motor_speed(1, 0, 3000, 0)    # Motor 2
        set_motor_speed(10, 9, 3000, 0)   # Motor 3
        time.sleep(5)

        # Move all motors in reverse for 5 seconds
        print("All motors moving in reverse")
        set_motor_speed(15, 14, 0, 3000)  # Motor 1
        set_motor_speed(1, 0, 0, 3000)   # Motor 2
        set_motor_speed(10, 9, 0, 3000)  # Motor 3
        time.sleep(5)

        # Stop all motors
        print("Stopping all motors")
        set_motor_speed(15, 14, 0, 0)  # Motor 1
        set_motor_speed(1, 0, 0, 0)   # Motor 2
        set_motor_speed(9, 10, 0, 0)  # Motor 3
        time.sleep(2)

except KeyboardInterrupt:
    print("Program interrupted")
    # Stop motors on exit
    set_motor_speed(15, 14, 0, 0)  # Motor 1
    set_motor_speed(1, 0, 0, 0)   # Motor 2
    set_motor_speed(10, 9, 0, 0)  # Motor 3

