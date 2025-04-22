import tkinter as tk
import pigpio
import threading
import time

class RoboticArmController:
    # Servo pin configuration
    SERVO_PINS = {
        'BASE': 24,       # Base rotation
        'SHOULDER': 23,   # Shoulder joint (restricted to 40°-60°)
        'ELBOW': 22,      # Elbow joint
        'WRIST_PITCH': 27,  # Wrist up/down
        'WRIST_ROLL': 18,   # Wrist rotation
        'GRIPPER': 17     # End effector/gripper
    }
    
    # Min/Max pulse widths (in μs) for each servo
    SERVO_LIMITS = {
        'BASE': (500, 2500),
        'SHOULDER': (500, 2500),  # We'll apply angle restrictions in code
        'ELBOW': (500, 2500),
        'WRIST_PITCH': (500, 2500),
        'WRIST_ROLL': (500, 2500),
        'GRIPPER': (1000, 2000)  # Closed to Open
    }
    
    # Angle limits for shoulder (40° to 60°)
    SHOULDER_MIN_ANGLE = 40
    SHOULDER_MAX_ANGLE = 60
    
    # Default step size and interval for servo movements
    DEFAULT_STEP = 20      # Step size in μs
    DEFAULT_INTERVAL = 50  # Interval between steps in ms
    
    # Conversion constants for shoulder
    SHOULDER_MIN_PW = 500  # Pulse width for 0°
    SHOULDER_MAX_PW = 2500  # Pulse width for 180°
    SHOULDER_PW_PER_DEGREE = (SHOULDER_MAX_PW - SHOULDER_MIN_PW) / 180

    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Robotic Arm Control")
        self.root.geometry("800x600")
        
        # Button states tracking
        self.button_states = {}
        
        # Set up attributes but don't initialize hardware
        self.pi = None
        self.current_pw = {}
        self.position_labels = {}
        self.angle_labels = {}
        
        # Set up UI only
        self._setup_ui()

    def initialize_hardware(self):
        """Initialize the hardware connections - must be called explicitly"""
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon!")
            return False
            
        # Current pulse width for each servo
        self.current_pw = {pin_name: 1500 for pin_name in self.SERVO_PINS}
        
        # Set shoulder to middle of restricted range
        shoulder_middle_angle = (self.SHOULDER_MIN_ANGLE + self.SHOULDER_MAX_ANGLE) / 2
        shoulder_middle_pw = self.angle_to_pw('SHOULDER', shoulder_middle_angle)
        self.current_pw['SHOULDER'] = shoulder_middle_pw
        
        # Initialize all servos
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            
            # Update angle display
            if name == 'SHOULDER':
                angle = self.pw_to_angle('SHOULDER', self.current_pw[name])
                self.angle_labels[name].config(text=f"{angle:.1f}°")
        
        self.servo_status.config(text="Hardware initialized")
        return True

    def _setup_ui(self):
        """Set up the user interface"""
        # Configure grid layout
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        
        # Main frame
        main_frame = tk.Frame(self.root, bg="#f0f0f0", padx=20, pady=20)
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Title and status section
        title_frame = tk.Frame(main_frame, bg="#f0f0f0")
        title_frame.pack(fill=tk.X, pady=10)
        
        title_label = tk.Label(title_frame, text="ROBOTIC ARM CONTROLLER", 
                             font=("Arial", 16, "bold"), bg="#f0f0f0")
        title_label.pack()
        
        # Status display
        self.status_label = tk.Label(main_frame, text="Robotic Arm Status:", 
                                     font=("Arial", 14), bg="#f0f0f0")
        self.status_label.pack(pady=(10, 5))
        
        self.servo_status = tk.Label(main_frame, text="Not initialized", 
                                     font=("Arial", 12), bg="white", width=30, height=2, 
                                     relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 15))
        
        # Add initialize hardware button
        init_btn = tk.Button(main_frame, text="INITIALIZE HARDWARE", 
                            command=self.initialize_hardware,
                            bg="#33AA33", fg="white", font=("Arial", 12, "bold"),
                            width=20, height=1)
        init_btn.pack(pady=5)
        
        # Position feedback
        self.feedback_frame = tk.Frame(main_frame, bg="#f0f0f0")
        self.feedback_frame.pack(fill=tk.X, pady=10)
        
        self.position_labels = {}
        self.angle_labels = {}
        
        # Create position and angle labels
        for i, name in enumerate(self.SERVO_PINS.keys()):
            row = i // 3
            col = i % 3
            
            label_frame = tk.Frame(self.feedback_frame, bg="#f0f0f0")
            label_frame.grid(row=row, column=col, padx=10, pady=5)
            
            servo_label = tk.Label(label_frame, text=f"{name}:", font=("Arial", 10), 
                     bg="#f0f0f0")
            servo_label.pack(side=tk.LEFT)
            
            self.position_labels[name] = tk.Label(label_frame, text="N/A", 
                                                 font=("Arial", 10), width=8, 
                                                 bg="white", relief=tk.SUNKEN)
            self.position_labels[name].pack(side=tk.LEFT, padx=5)
            
            # Add angle display for shoulder only
            if name == 'SHOULDER':
                self.angle_labels[name] = tk.Label(label_frame, text="N/A", 
                                                   font=("Arial", 10), width=6,
                                                   bg="#FFEEEE", relief=tk.SUNKEN)
                self.angle_labels[name].pack(side=tk.LEFT, padx=5)
                
                # Add range indicator
                range_label = tk.Label(label_frame, text=f"[{self.SHOULDER_MIN_ANGLE}°-{self.SHOULDER_MAX_ANGLE}°]", 
                                      font=("Arial", 8), bg="#f0f0f0", fg="#CC0000")
                range_label.pack(side=tk.LEFT, padx=2)
        
        # Create servo control sections
        controls_container = tk.Frame(main_frame, bg="#f0f0f0")
        controls_container.pack(fill=tk.BOTH, expand=True, pady=15)
        
        # Upper arm controls (Row 1)
        self.create_servo_section(controls_container, "UPPER ARM CONTROLS", 
                                 ["BASE", "SHOULDER", "ELBOW"], 
                                 "#335577", "white", 0)
        
        # Lower arm controls (Row 2)
        self.create_servo_section(controls_container, "LOWER ARM CONTROLS", 
                                 ["WRIST_PITCH", "WRIST_ROLL", "GRIPPER"], 
                                 "#773355", "white", 1)
        
        # Create preset positions buttons
        preset_frame = tk.Frame(main_frame, bg="#f0f0f0")
        preset_frame.pack(fill=tk.X, pady=15)
        
        preset_label = tk.Label(preset_frame, text="PRESET POSITIONS", 
                               font=("Arial", 12, "bold"), bg="#f0f0f0")
        preset_label.pack(pady=(0, 10))
        
        btn_frame = tk.Frame(preset_frame, bg="#f0f0f0")
        btn_frame.pack()
        
        presets = [
            ("Home", self.home_position),
            ("Pick", self.pick_position),
            ("Place", self.place_position),
            ("Rest", self.rest_position)
        ]
        
        for i, (name, command) in enumerate(presets):
            btn = tk.Button(btn_frame, text=name, command=command,
                          bg="#444444", fg="white", font=("Arial", 10),
                          width=8, height=2)
            btn.grid(row=0, column=i, padx=10)
        
        # Emergency stop button
        stop_btn = tk.Button(main_frame, text="EMERGENCY STOP", command=self.emergency_stop,
                           bg="#FF3333", fg="white", font=("Arial", 12, "bold"),
                           width=20, height=2)
        stop_btn.pack(pady=15)

    def create_servo_section(self, parent, group_name, servo_names, bg_color, fg_color, row):
        """Create a section of servo control buttons"""
        frame = tk.Frame(parent, bg="#f0f0f0", padx=10, pady=10)
        frame.grid(row=row, column=0, sticky="ew")
        
        group_label = tk.Label(frame, text=group_name, font=("Arial", 12, "bold"), bg="#f0f0f0")
        group_label.pack(pady=(0, 10))
        
        button_container = tk.Frame(frame, bg="#f0f0f0")
        button_container.pack()
        
        for i, name in enumerate(servo_names):
            button_frame = tk.Frame(button_container, bg="#f0f0f0", padx=10, pady=5)
            button_frame.grid(row=0, column=i)
            
            label = tk.Label(button_frame, text=name, font=("Arial", 10, "bold"), bg="#f0f0f0")
            label.pack(pady=(0, 5))
            
            btn_frame = tk.Frame(button_frame, bg="#f0f0f0")
            btn_frame.pack()
            
            # CCW button (usually increases pulse width)
            ccw_btn = tk.Button(btn_frame, text="←", bg=bg_color, fg=fg_color,
                               font=("Arial", 12, "bold"), width=4, height=2)
            ccw_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind events with proper lambda to avoid late binding issues
            button_id = f"{name} CCW"
            ccw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            ccw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))
            
            # CW button (usually decreases pulse width)
            cw_btn = tk.Button(btn_frame, text="→", bg=bg_color, fg=fg_color,
                              font=("Arial", 12, "bold"), width=4, height=2)
            cw_btn.pack(side=tk.LEFT, padx=2)
            
            button_id = f"{name} CW"
            cw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            cw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))

    def verify_initialized(self):
        """Check if hardware is initialized before performing operations"""
        if not self.pi or not self.pi.connected:
            self.servo_status.config(text="Hardware not initialized!")
            return False
        return True
    
    def angle_to_pw(self, servo_name, angle):
        """Convert angle to pulse width for the given servo"""
        if servo_name == 'SHOULDER':
            # Ensure angle is within the allowed range
            angle = max(self.SHOULDER_MIN_ANGLE, min(angle, self.SHOULDER_MAX_ANGLE))
            return int(self.SHOULDER_MIN_PW + angle * self.SHOULDER_PW_PER_DEGREE)
        else:
            # Standard conversion for other servos (assuming 0-180° maps to min-max pulse width)
            min_pw, max_pw = self.SERVO_LIMITS[servo_name]
            return int(min_pw + (angle / 180.0) * (max_pw - min_pw))
    
    def pw_to_angle(self, servo_name, pw):
        """Convert pulse width to angle for the given servo"""
        if servo_name == 'SHOULDER':
            angle = (pw - self.SHOULDER_MIN_PW) / self.SHOULDER_PW_PER_DEGREE
            return max(self.SHOULDER_MIN_ANGLE, min(angle, self.SHOULDER_MAX_ANGLE))
        else:
            # Standard conversion for other servos
            min_pw, max_pw = self.SERVO_LIMITS[servo_name]
            return (pw - min_pw) * 180.0 / (max_pw - min_pw)

    def home_position(self):
        """Move all servos to home position"""
        if not self.verify_initialized():
            return
            
        home_positions = {
            'BASE': 1500,
            'SHOULDER': self.angle_to_pw('SHOULDER', 50),  # Middle of shoulder range (40°-60°)
            'ELBOW': 1500,
            'WRIST_PITCH': 1500,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1500
        }
        self.move_to_preset(home_positions)

    def pick_position(self):
        """Move to picking position"""
        if not self.verify_initialized():
            return
            
        pick_positions = {
            'BASE': 1800,
            'SHOULDER': self.angle_to_pw('SHOULDER', 40),  # Min shoulder angle
            'ELBOW': 1000,
            'WRIST_PITCH': 1700,
            'WRIST_ROLL': 1500,
            'GRIPPER': 2000  # Open
        }
        self.move_to_preset(pick_positions)

    def place_position(self):
        """Move to placing position"""
        if not self.verify_initialized():
            return
            
        place_positions = {
            'BASE': 1200,
            'SHOULDER': self.angle_to_pw('SHOULDER', 60),  # Max shoulder angle
            'ELBOW': 1800,
            'WRIST_PITCH': 1300,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1000  # Closed
        }
        self.move_to_preset(place_positions)

    def rest_position(self):
        """Move to rest position"""
        if not self.verify_initialized():
            return
            
        rest_positions = {
            'BASE': 1500,
            'SHOULDER': self.angle_to_pw('SHOULDER', 45),  # Within shoulder range
            'ELBOW': 2000,
            'WRIST_PITCH': 1500,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1500
        }
        self.move_to_preset(rest_positions)

    def move_to_preset(self, positions):
        """Move all servos to preset positions gradually"""
        if not self.verify_initialized():
            return
            
        # Stop any ongoing movements
        self.button_states.clear()
        self.servo_status.config(text="Moving to preset position...")
        
        # Create a separate thread for movement to avoid blocking GUI
        threading.Thread(target=self._move_to_preset_thread, 
                        args=(positions,), daemon=True).start()

    def _move_to_preset_thread(self, target_positions):
        """Thread function for smooth movement to preset positions"""
        # Calculate steps for each servo
        steps = {}
        current_positions = self.current_pw.copy()
        
        for name, target in target_positions.items():
            if name in self.SERVO_PINS:
                steps[name] = (target - current_positions[name]) / 50  # 50 steps total
        
        # Move in small increments
        for step in range(50):
            for name, increment in steps.items():
                pin = self.SERVO_PINS[name]
                new_pw = int(current_positions[name] + increment * (step + 1))
                
                # For shoulder, make sure we're within range
                if name == 'SHOULDER':
                    angle = self.pw_to_angle(name, new_pw)
                    new_pw = self.angle_to_pw(name, angle)  # This will enforce limits
                else:
                    # For other servos, ensure within limits
                    min_pw, max_pw = self.SERVO_LIMITS[name]
                    new_pw = max(min_pw, min(new_pw, max_pw))
                
                # Update position
                self.current_pw[name] = new_pw
                self.pi.set_servo_pulsewidth(pin, new_pw)
                
                # Update UI in main thread
                if name == 'SHOULDER':
                    angle = self.pw_to_angle(name, new_pw)
                    self.root.after(0, lambda n=name, p=new_pw, a=angle: 
                                   [self.position_labels[n].config(text=f"{p}μs"),
                                    self.angle_labels[n].config(text=f"{a:.1f}°")])
                else:
                    self.root.after(0, lambda n=name, p=new_pw: 
                                   self.position_labels[n].config(text=f"{p}μs"))
            
            time.sleep(0.02)  # 20ms between steps
        
        # Update status when done
        self.root.after(0, lambda: self.servo_status.config(text="Preset position reached"))

    def emergency_stop(self):
        """Stop all servo movements immediately"""
        if not self.verify_initialized():
            return
            
        # Clear button states
        self.button_states.clear()
        
        # Update status
        self.servo_status.config(text="EMERGENCY STOP ACTIVATED")
        
        # Stop all servo pulses
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, 0)
            time.sleep(0.01)  # Brief pause between commands
        
        # Reactivate servos at current position after a short delay
        self.root.after(500, self.reactivate_servos)

    def reactivate_servos(self):
        """Reactivate servos after emergency stop"""
        if not self.verify_initialized():
            return
            
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            
            if name == 'SHOULDER':
                angle = self.pw_to_angle(name, self.current_pw[name])
                self.position_labels[name].config(text=f"{self.current_pw[name]}μs")
                self.angle_labels[name].config(text=f"{angle:.1f}°")
            else:
                self.position_labels[name].config(text=f"{self.current_pw[name]}μs")
        
        self.servo_status.config(text="Ready")

    def button_press(self, button_id):
        """Handle button press events"""
        if not self.verify_initialized():
            return
            
        # Parse button ID to get servo name and direction
        parts = button_id.split()
        servo_name = parts[0]
        direction = parts[1]  # CCW or CW
        
        # Mark button as pressed
        self.button_states[button_id] = True
        
        # Update status display
        self.servo_status.config(text=f"Moving: {servo_name} {direction}")
        
        # Start continuous movement
        self.move_servo_continuously(button_id)

    def button_release(self, button_id):
        """Handle button release events"""
        if not self.verify_initialized():
            return
            
        # Check if this button was active
        if button_id in self.button_states:
            # Mark button as released
            self.button_states[button_id] = False
            
            # Remove from button states
            del self.button_states[button_id]
            
            # Get servo name
            servo_name = button_id.split()[0]
            
            # Clean up: make sure pulse width is stable
            pin = self.SERVO_PINS[servo_name]
            self.pi.set_servo_pulsewidth(pin, self.current_pw[servo_name])
            
            # Update status if no buttons are pressed
            if not self.button_states:
                self.servo_status.config(text="Ready")

    def move_servo_continuously(self, button_id):
        """Move servo continuously while button is pressed"""
        # Only proceed if button is still pressed and hardware is initialized
        if not self.pi or not self.pi.connected:
            return
            
        if button_id in self.button_states and self.button_states[button_id]:
            # Parse button ID
            parts = button_id.split()
            servo_name = parts[0]
            direction = parts[1]  # CCW or CW
            
            # Get servo pin
            pin = self.SERVO_PINS[servo_name]
            
            # Get current pulse width
            current = self.current_pw[servo_name]
            
            # Calculate new pulse width based on direction
            if servo_name == 'SHOULDER':
                # For shoulder, we need to work with angles first to enforce limits
                current_angle = self.pw_to_angle(servo_name, current)
                
                if direction == "CCW":
                    new_angle = min(current_angle + 0.5, self.SHOULDER_MAX_ANGLE)
                else:  # CW
                    new_angle = max(current_angle - 0.5, self.SHOULDER_MIN_ANGLE)
                
                # Convert back to pulse width
                new_pw = self.angle_to_pw(servo_name, new_angle)
                
                # Update angle display
                self.angle_labels[servo_name].config(text=f"{new_angle:.1f}°")
            else:
                # For other servos, use standard min/max limits
                min_pw, max_pw = self.SERVO_LIMITS[servo_name]
                
                if direction == "CCW":
                    new_pw = min(current + self.DEFAULT_STEP, max_pw)
                else:  # CW
                    new_pw = max(current - self.DEFAULT_STEP, min_pw)
            
            # Update position if it changed
            if new_pw != current:
                self.current_pw[servo_name] = new_pw
                self.pi.set_servo_pulsewidth(pin, new_pw)
                
                # Update position label
                self.position_labels[servo_name].config(text=f"{new_pw}μs")
            
            # Schedule the next movement if button is still pressed
            self.root.after(self.DEFAULT_INTERVAL, 
                           lambda: self.move_servo_continuously(button_id))

    def cleanup(self):
        """Clean up resources when closing the application"""
        print("Cleaning up resources...")
        
        # Clear any button states
        self.button_states.clear()
        
        # Turn off all servos by stopping pulses
        if hasattr(self, 'pi') and self.pi and self.pi.connected:
            print("Turning off servo motors...")
            for name, pin in self.SERVO_PINS.items():
                self.pi.set_servo_pulsewidth(pin, 0)  # Stop the pulse
                time.sleep(0.1)  # Brief pause between commands
            
            # Stop pigpio
            self.pi.stop()
            print("All servo motors turned off.")

def main():
    # Start GUI
    root = tk.Tk()
    app = RoboticArmController(root)
    
    # Close GPIO properly when window is closed
    root.protocol("WM_DELETE_WINDOW", lambda: [app.cleanup(), root.destroy()])
    
    root.mainloop()

if __name__ == "__main__":
    main()