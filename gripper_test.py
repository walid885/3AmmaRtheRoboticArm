import pigpio
import time

SERVO_PIN = 17

# Connect to pigpio daemon
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

def move_servo_slowly(pin, start_pw, end_pw, duration=2.0):
    """
    Move servo slowly from start position to end position
    
    Args:
        pin: GPIO pin number
        start_pw: Starting pulse width in microseconds
        end_pw: Ending pulse width in microseconds
        duration: Time to complete movement in seconds
    """
    steps = 50  # Number of incremental steps
    delay = duration / steps  # Time between steps
    step_size = (end_pw - start_pw) / steps
    
    print(f"Moving servo on pin {pin} slowly from {start_pw}us to {end_pw}us...")
    
    # Move in small increments
    for i in range(steps + 1):
        pulse_width = start_pw + (step_size * i)
        pi.set_servo_pulsewidth(pin, pulse_width)
        time.sleep(delay)

try:
    # Move slowly from 0 degree to 90 degree
    move_servo_slowly(SERVO_PIN, 1000, 2000, 3.0)
    time.sleep(1)
    
    # Move slowly back to 0 degree
    move_servo_slowly(SERVO_PIN, 2000, 1000, 3.0)
    
    # Turn off servo
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    
except KeyboardInterrupt:
    print("Program interrupted")
    
finally:
    pi.stop()
    print("Test completed")
