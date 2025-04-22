import pigpio
import time
import tkinter as tk
from tkinter import Scale, Label, Button, Frame

# GPIO Pin for servo shoulder 
SERVO_PIN = 23

# Servo configuration parameters
MIN_PULSE_WIDTH = 500    # Minimum pulse width in microseconds (0 degrees)
MAX_PULSE_WIDTH = 2500   # Maximum pulse width in microseconds (180 degrees)
MIN_DEGREE = 0           # Minimum angle in degrees
MAX_DEGREE = 180         # Maximum angle in degrees

# Connect to pigpio daemon
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

def pulse_width_to_degree(pulse_width):
    """Convert pulse width in microseconds to degrees"""
    return (pulse_width - MIN_PULSE_WIDTH) * (MAX_DEGREE - MIN_DEGREE) / (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) + MIN_DEGREE

def degree_to_pulse_width(degree):
    """Convert degrees to pulse width in microseconds"""
    return (degree - MIN_DEGREE) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (MAX_DEGREE - MIN_DEGREE) + MIN_PULSE_WIDTH

def move_servo_slowly(pin, start_degree, end_degree, duration=5.0):
    """
    Move servo slowly from start position to end position using degrees
    
    Args:
        pin: GPIO pin number
        start_degree: Starting position in degrees
        end_degree: Ending position in degrees
        duration: Time to complete movement in seconds
    """
    # Convert degrees to pulse width
    start_pw = degree_to_pulse_width(start_degree)
    end_pw = degree_to_pulse_width(end_degree)
    
    steps = 50  # Number of incremental steps
    delay = duration / steps  # Time between steps
    step_size_pw = (end_pw - start_pw) / steps
    step_size_degree = (end_degree - start_degree) / steps
    
    print(f"Moving servo on pin {pin} from {start_degree}° to {end_degree}°...")
    
    # Move in small increments
    for i in range(steps + 1):
        pulse_width = start_pw + (step_size_pw * i)
        current_degree = start_degree + (step_size_degree * i)
        pi.set_servo_pulsewidth(pin, pulse_width)
        print(f"Step {i}: {current_degree:.1f}° (PW: {pulse_width:.1f}μs)")
        time.sleep(delay)

def move_to_position(degree):
    """Move servo to specific degree position"""
    pulse_width = degree_to_pulse_width(degree)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
    position_label.config(text=f"Current Position: {degree:.1f}° (PW: {pulse_width:.1f}μs)")

def on_slider_change(value):
    """Handle slider value change"""
    degree = float(value)
    move_to_position(degree)

def move_increment(increment):
    """Move servo by increment amount"""
    current = float(position_slider.get())
    new_position = current + increment
    
    # Ensure position stays within limits
    if new_position < MIN_DEGREE:
        new_position = MIN_DEGREE
    elif new_position > MAX_DEGREE:
        new_position = MAX_DEGREE
    
    position_slider.set(new_position)
    move_to_position(new_position)

def move_slowly_to_position():
    """Move servo slowly to the position set in the slider"""
    current_pw = pi.get_servo_pulsewidth(SERVO_PIN)
    if current_pw == 0:  # If servo is off
        current_degree = 0
    else:
        current_degree = pulse_width_to_degree(current_pw)
    
    target_degree = float(position_slider.get())
    move_servo_slowly(SERVO_PIN, current_degree, target_degree, duration=float(speed_slider.get()))

def turn_off_servo():
    """Turn off the servo"""
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    position_label.config(text="Servo OFF")

# Create GUI
root = tk.Tk()
root.title("Servo Controller")
root.geometry("500x450")

# Main frame
main_frame = Frame(root, padx=20, pady=20)
main_frame.pack(fill="both", expand=True)

# Position slider
position_frame = Frame(main_frame)
position_frame.pack(fill="x", pady=10)
Label(position_frame, text="Position (degrees):").pack(side="left")
position_slider = Scale(position_frame, from_=MIN_DEGREE, to=MAX_DEGREE, 
                        orient="horizontal", length=300, command=on_slider_change)
position_slider.pack(side="right", fill="x", expand=True)
position_slider.set(90)  # Default to middle position

# Speed control
speed_frame = Frame(main_frame)
speed_frame.pack(fill="x", pady=10)
Label(speed_frame, text="Movement Speed (seconds):").pack(side="left")
speed_slider = Scale(speed_frame, from_=0.5, to=5.0, resolution=0.1,
                    orient="horizontal", length=300)
speed_slider.pack(side="right", fill="x", expand=True)
speed_slider.set(2.0)  # Default speed

# Increment buttons
increment_frame = Frame(main_frame)
increment_frame.pack(fill="x", pady=10)
Label(increment_frame, text="Move by increment:").pack(side="left")

Button(increment_frame, text="-10°", command=lambda: move_increment(-10), 
       width=5).pack(side="left", padx=5)
Button(increment_frame, text="-5°", command=lambda: move_increment(-5), 
       width=5).pack(side="left", padx=5)
Button(increment_frame, text="-1°", command=lambda: move_increment(-1), 
       width=5).pack(side="left", padx=5)
Button(increment_frame, text="+1°", command=lambda: move_increment(1), 
       width=5).pack(side="left", padx=5)
Button(increment_frame, text="+5°", command=lambda: move_increment(5), 
       width=5).pack(side="left", padx=5)
Button(increment_frame, text="+10°", command=lambda: move_increment(10), 
       width=5).pack(side="left", padx=5)

# Control buttons
control_frame = Frame(main_frame)
control_frame.pack(fill="x", pady=10)

Button(control_frame, text="Move Slowly", command=move_slowly_to_position, 
       bg="green", fg="white", width=15).pack(side="left", padx=5)
Button(control_frame, text="Turn Off Servo", command=turn_off_servo, 
       bg="red", fg="white", width=15).pack(side="right", padx=5)

# Position display
position_label = Label(main_frame, text="Current Position: 90.0° (PW: 1500.0μs)", 
                      font=("Arial", 12))
position_label.pack(pady=10)

# Information frame
info_frame = Frame(main_frame)
info_frame.pack(fill="x", pady=10)

info_text = """
Tunable Parameters:
- MIN_PULSE_WIDTH (500μs): Minimum pulse width for 0 degrees
- MAX_PULSE_WIDTH (2500μs): Maximum pulse width for 180 degrees
- Movement Steps (50): Number of increments for smooth movement
- Movement Duration: Time to complete movement (adjustable)
"""
Label(info_frame, text=info_text, justify="left").pack()

# Initialize the servo position
move_to_position(90)

try:
    # Start the GUI main loop
    root.mainloop()
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    # Clean up when done
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()
    print("Program terminated")