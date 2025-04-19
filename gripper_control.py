import pigpio
import time
import sys
import tty
import termios

# Servo configuration
GRIPPER_PIN = 17
OPEN_PW = 2000   # Pulse width for open position (?s)
CLOSED_PW = 1000  # Pulse width for closed position (?s)
SPEED = 3.0      # Movement duration in seconds

# Connect to pigpio daemon
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

def getch():
    """Get a single character from stdin without waiting for newline"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def get_current_pw(pin):
    """Get current pulse width or return middle position if servo is off"""
    current_pw = pi.get_servo_pulsewidth(pin)
    if current_pw == 0:  # If servo is off
        return 1500  # Return middle position
    return current_pw

try:
    # Set gripper to middle position initially
    pi.set_servo_pulsewidth(GRIPPER_PIN, 1500)
    time.sleep(1)
    
    print("\nGripper Control Ready!")
    print("---------------------")
    print("Press 'o' to open the gripper")
    print("Press 'c' to close the gripper")
    print("Press 'q' to quit the program")
    print("---------------------")
    
    while True:
        # Get current position
        current_pw = get_current_pw(GRIPPER_PIN)
        
        # Get key press non-blocking
        key = getch()
        
        if key == 'o':
            # Open gripper with a single movement
            print("Opening gripper...")
            target_pw = OPEN_PW
            pi.set_servo_pulsewidth(GRIPPER_PIN, target_pw)
            print(f"Gripper at: {target_pw}?s")
            
        elif key == 'c':
            # Close gripper with a single movement
            print("Closing gripper...")
            target_pw = CLOSED_PW
            pi.set_servo_pulsewidth(GRIPPER_PIN, target_pw)
            print(f"Gripper at: {target_pw}?s")
            
        elif key == 'm':
            # Move to middle position
            print("Moving to middle position...")
            pi.set_servo_pulsewidth(GRIPPER_PIN, 1500)
            print("Gripper at middle position")
            
        elif key == 'q':
            print("\nExiting program...")
            break
    
except KeyboardInterrupt:
    print("\nProgram interrupted")
    
finally:
    # Clean up
    print("\nCleaning up...")
    pi.set_servo_pulsewidth(GRIPPER_PIN, 0)  # Turn off servo
    pi.stop()
    print("Test completed")
