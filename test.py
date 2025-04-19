import tkinter as tk
import pigpio
import threading
import time

class RoboticArmController:
    # Servo pin configuration
    SERVO_PINS = {
        'BASE': 17,       # Base rotation
        'SHOULDER': 18,   # Shoulder joint
        'ELBOW': 27,      # Elbow joint
        'WRIST_PITCH': 22,  # Wrist up/down
        'WRIST_ROLL': 23,   # Wrist rotation
        'GRIPPER': 24     # End effector/gripper
    }
    
    # Min/Max pulse widths (in μs) for each servo
    SERVO_LIMITS = {
        'BASE': (600, 2400),        # Adjusted limits
        'SHOULDER': (800, 2200),    # Adjusted limits
        'ELBOW': (500, 2500),
        'WRIST_PITCH': (500, 2500),
        'WRIST_ROLL': (500, 2500),
        'GRIPPER': (800, 2200)      # Adjusted limits for better grip range
    }
    
    # Step sizes for different servos (different servos may need different step sizes)
    STEP_SIZES = {
        'BASE': 10,       # Smaller steps for smoother base movement
        'SHOULDER': 15,   # Smaller steps for smoother shoulder movement
        'ELBOW': 20,
        'WRIST_PITCH': 20,
        'WRIST_ROLL': 20,
        'GRIPPER': 25     # Larger steps for responsive gripper
    }
    
    # Speed intervals for different servos (ms)
    SPEED_INTERVALS = {
        'BASE': 30,      # Slower for base (more stable)
        'SHOULDER': 30,  # Slower for shoulder (more stable)
        'ELBOW': 40,
        'WRIST_PITCH': 40,
        'WRIST_ROLL': 40,
        'GRIPPER': 30    # Quick gripper response
    }

    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Robotic Arm Control")
        self.root.geometry("800x600")
        
        # Initialize pigpio with multiple connection attempts
        self.pi = self.connect_to_pigpio(max_attempts=3)
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon!")
            tk.messagebox.showerror("Connection Error", "Failed to connect to pigpio daemon!")
            return
            
        # Current pulse width for each servo
        self.current_pw = {pin_name: 1500 for pin_name in self.SERVO_PINS}
        
        # Active servo tracking (None means no servo is active)
        self.active_servos = set()  # Allow multiple active servos
        
        # Button states tracking
        self.button_states = {}
        
        # Movement threads dict
        self.movement_threads = {}
        
        self._setup_ui()
        
        # Initialize all servos to middle position then turn them off
        self.initialize_servos()
        
        # Add heartbeat to verify servo connection
        self.root.after(5000, self.check_servo_connections)

    def connect_to_pigpio(self, max_attempts=3):
        """Try to connect to pigpio daemon with multiple attempts"""
        for attempt in range(max_attempts):
            pi = pigpio.pi()
            if pi.connected:
                print(f"Connected to pigpio daemon on attempt {attempt+1}")
                return pi
            print(f"Connection attempt {attempt+1} failed, retrying...")
            time.sleep(1)
        return pi  # Return last attempt even if failed

    def check_servo_connections(self):
        """Periodically check if servos are responsive"""
        if not self.pi.connected:
            self.servo_status.config(text="ERROR: Lost connection to pigpio!")
            # Try to reconnect
            self.pi = self.connect_to_pigpio()
            if self.pi.connected:
                self.servo_status.config(text="Reconnected to pigpio")
        
        # Test problematic servos
        problem_servos = []
        test_servos = ['BASE', 'SHOULDER', 'GRIPPER']
        
        for servo in test_servos:
            pin = self.SERVO_PINS[servo]
            # Quick pulse test
            current = self.current_pw[servo]
            try:
                self.pi.set_servo_pulsewidth(pin, current)
                time.sleep(0.1)
                self.pi.set_servo_pulsewidth(pin, 0)  # Turn off
            except Exception as e:
                problem_servos.append(f"{servo} ({e})")
        
        if problem_servos:
            self.servo_status.config(text=f"WARNING: Issues with {', '.join(problem_servos)}")
        
        # Schedule next check
        self.root.after(30000, self.check_servo_connections)  # Check every 30 seconds

    def initialize_servos(self):
        """Initialize all servos to middle position then turn them off"""
        # First set all to middle position
        for name, pin in self.SERVO_PINS.items():
            # Use safe middle position within limits
            min_pw, max_pw = self.SERVO_LIMITS[name]
            middle_pw = (min_pw + max_pw) // 2
            self.current_pw[name] = middle_pw
            
            try:
                self.pi.set_servo_pulsewidth(pin, middle_pw)
                # Update position label
                if hasattr(self, 'position_labels') and name in self.position_labels:
                    self.position_labels[name].config(text=f"{middle_pw}μs")
            except Exception as e:
                print(f"Error initializing {name}: {e}")
            
            time.sleep(0.2)  # Longer pause to allow servo to reach position
        
        # Then turn all off to save power
        time.sleep(1.0)  # Wait for servos to reach position
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, 0)  # Stop the pulse

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
        
        self.servo_status = tk.Label(main_frame, text="Ready - All servos off", 
                                     font=("Arial", 12), bg="white", width=30, height=2, 
                                     relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 15))
        
        # Position feedback
        self.feedback_frame = tk.Frame(main_frame, bg="#f0f0f0")
        self.feedback_frame.pack(fill=tk.X, pady=10)
        
        self.position_labels = {}
        for i, name in enumerate(self.SERVO_PINS.keys()):
            row = i // 3
            col = i % 3
            
            label_frame = tk.Frame(self.feedback_frame, bg="#f0f0f0")
            label_frame.grid(row=row, column=col, padx=10, pady=5)
            
            # Show valid range for each servo
            min_pw, max_pw = self.SERVO_LIMITS[name]
            range_text = f"{name} ({min_pw}-{max_pw}):"
            
            tk.Label(label_frame, text=range_text, font=("Arial", 10), 
                     bg="#f0f0f0").pack(side=tk.LEFT)
            
            # Initialize position label with middle position
            self.position_labels[name] = tk.Label(label_frame, text=f"{self.current_pw[name]}μs", 
                                                  font=("Arial", 10), width=8, 
                                                  bg="white", relief=tk.SUNKEN)
            self.position_labels[name].pack(side=tk.LEFT, padx=5)
        
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
        
        # Add speed control slider
        speed_frame = tk.Frame(main_frame, bg="#f0f0f0")
        speed_frame.pack(fill=tk.X, pady=10)
        
        tk.Label(speed_frame, text="Movement Speed:", bg="#f0f0f0").pack(side=tk.LEFT, padx=10)
        
        self.speed_factor = tk.DoubleVar(value=1.0)
        speed_slider = tk.Scale(speed_frame, from_=0.5, to=2.0, resolution=0.1,
                                orient=tk.HORIZONTAL, length=200, variable=self.speed_factor,
                                bg="#f0f0f0", label="Speed Factor")
        speed_slider.pack(side=tk.LEFT, padx=10)
        
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
            ("Rest", self.rest_position),
            ("Test Servos", self.test_all_servos)  # Added test function
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
            
            # Show servo name with its specific step size
            step_size = self.STEP_SIZES[name]
            label = tk.Label(button_frame, text=f"{name} (Step: {step_size}μs)", 
                           font=("Arial", 10, "bold"), bg="#f0f0f0")
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

    def test_all_servos(self):
        """Test all servos with a small movement to verify they're working"""
        self.stop_all_servos()
        self.servo_status.config(text="Testing all servos...")
        
        # Create a thread for testing
        threading.Thread(target=self._test_servos_thread, daemon=True).start()

    def _test_servos_thread(self):
        """Thread for testing each servo individually"""
        servo_list = list(self.SERVO_PINS.keys())
        
        for name in servo_list:
            pin = self.SERVO_PINS[name]
            min_pw, max_pw = self.SERVO_LIMITS[name]
            middle_pw = (min_pw + max_pw) // 2
            
            # Update status
            self.root.after(0, lambda n=name: 
                           self.servo_status.config(text=f"Testing servo: {n}"))
            
            try:
                # Move to middle
                self.pi.set_servo_pulsewidth(pin, middle_pw)
                self.current_pw[name] = middle_pw
                self.root.after(0, lambda n=name, p=middle_pw: 
                               self.position_labels[n].config(text=f"{p}μs"))
                time.sleep(0.5)
                
                # Move slightly one way
                test_pw = middle_pw + 100
                if test_pw > max_pw:
                    test_pw = middle_pw - 100
                
                self.pi.set_servo_pulsewidth(pin, test_pw)
                self.current_pw[name] = test_pw
                self.root.after(0, lambda n=name, p=test_pw: 
                               self.position_labels[n].config(text=f"{p}μs"))
                time.sleep(0.5)
                
                # Move back to middle
                self.pi.set_servo_pulsewidth(pin, middle_pw)
                self.current_pw[name] = middle_pw
                self.root.after(0, lambda n=name, p=middle_pw: 
                               self.position_labels[n].config(text=f"{p}μs"))
                time.sleep(0.5)
                
                # Turn off servo
                self.pi.set_servo_pulsewidth(pin, 0)
                
            except Exception as e:
                self.root.after(0, lambda n=name, err=str(e): 
                               self.servo_status.config(text=f"Error with {n}: {err}"))
                time.sleep(1)
        
        # Update status when done
        self.root.after(0, lambda: 
                       self.servo_status.config(text="Testing complete - All servos off"))

    def home_position(self):
        """Move all servos to home position"""
        home_positions = {
            'BASE': 1500,
            'SHOULDER': 1500,
            'ELBOW': 1500,
            'WRIST_PITCH': 1500,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1500
        }
        self.move_to_preset(home_positions)

    def pick_position(self):
        """Move to picking position"""
        # Adjusted positions based on servo limits
        pick_positions = {
            'BASE': 1800,
            'SHOULDER': 1200,
            'ELBOW': 1000,
            'WRIST_PITCH': 1700,
            'WRIST_ROLL': 1500,
            'GRIPPER': 2000  # Open
        }
        self.move_to_preset(pick_positions)

    def place_position(self):
        """Move to placing position"""
        # Adjusted positions based on servo limits
        place_positions = {
            'BASE': 1200,
            'SHOULDER': 1500,
            'ELBOW': 1800,
            'WRIST_PITCH': 1300,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1000  # Closed
        }
        self.move_to_preset(place_positions)

    def rest_position(self):
        """Move to rest position"""
        # Adjusted positions based on servo limits
        rest_positions = {
            'BASE': 1500,
            'SHOULDER': 2000,
            'ELBOW': 2000,
            'WRIST_PITCH': 1500,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1500
        }
        self.move_to_preset(rest_positions)

    def move_to_preset(self, positions):
        """Move all servos to preset positions gradually"""
        # Stop any ongoing movements
        self.button_states.clear()
        self.stop_all_servos()
        self.servo_status.config(text="Moving to preset position...")
        
        # Create a separate thread for movement to avoid blocking GUI
        move_thread = threading.Thread(target=self._move_to_preset_thread, 
                        args=(positions,), daemon=True)
        move_thread.start()

    def _move_to_preset_thread(self, target_positions):
        """Thread function for smooth movement to preset positions"""
        # First, validate all target positions are within limits
        for name, target in target_positions.items():
            min_pw, max_pw = self.SERVO_LIMITS[name]
            if target < min_pw or target > max_pw:
                adjusted = max(min_pw, min(target, max_pw))
                target_positions[name] = adjusted
                print(f"Warning: {name} target of {target}μs out of range, adjusted to {adjusted}μs")
        
        # Turn on all servos that need to move
        for name in target_positions:
            pin = self.SERVO_PINS[name]
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            time.sleep(0.05)  # Short delay between activating each servo
        
        # Calculate the maximum number of steps needed for any servo
        current_positions = self.current_pw.copy()
        max_steps = 1  # Minimum 1 step
        
        for name, target in target_positions.items():
            distance = abs(target - current_positions[name])
            steps_needed = distance // 10  # Move in 10μs increments for smoothness
            max_steps = max(max_steps, steps_needed)
        
        # Cap the max steps to avoid too long movements
        max_steps = min(max_steps, 100)
        
        # Calculate increments for each servo
        increments = {}
        for name, target in target_positions.items():
            increments[name] = (target - current_positions[name]) / max_steps
        
        # Move in small increments
        for step in range(max_steps):
            for name, increment in increments.items():
                pin = self.SERVO_PINS[name]
                new_pw = int(current_positions[name] + increment * (step + 1))
                
                # Ensure within limits
                min_pw, max_pw = self.SERVO_LIMITS[name]
                new_pw = max(min_pw, min(new_pw, max_pw))
                
                # Update position
                self.current_pw[name] = new_pw
                try:
                    self.pi.set_servo_pulsewidth(pin, new_pw)
                    
                    # Update UI in main thread
                    self.root.after(0, lambda n=name, p=new_pw: 
                                   self.position_labels[n].config(text=f"{p}μs"))
                except Exception as e:
                    print(f"Error moving {name}: {e}")
            
            # Adjust sleep time based on speed factor (inverse - lower factor means slower)
            sleep_time = 0.02 / self.speed_factor.get()
            time.sleep(sleep_time)
        
        # Hold position briefly
        time.sleep(0.5)
        
        # Turn off all servos to save power
        self.stop_all_servos()
        
        # Update status when done
        self.root.after(0, lambda: self.servo_status.config(text="Preset position reached - All servos off"))

    def stop_all_servos(self):
        """Turn off all servos to save power"""
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, 0)
        self.active_servos.clear()

    def emergency_stop(self):
        """Stop all servo movements immediately"""
        # Clear button states
        self.button_states.clear()
        
        # Stop all movement threads
        for thread_id in list(self.movement_threads.keys()):
            self.movement_threads[thread_id] = False
        self.movement_threads.clear()
        
        # Update status
        self.servo_status.config(text="EMERGENCY STOP ACTIVATED")
        
        # Stop all servo pulses
        self.stop_all_servos()
        
        # Update status after a short delay
        self.root.after(500, lambda: self.servo_status.config(text="Ready - All servos off"))

    def button_press(self, button_id):
        """Handle button press events"""
        # Parse button ID to get servo name and direction
        parts = button_id.split()
        servo_name = parts[0]
        direction = parts[1]  # CCW or CW
        
        # Mark button as pressed
        self.button_states[button_id] = True
        
        # Add servo to active set
        self.active_servos.add(servo_name)
        
        # Activate the requested servo
        pin = self.SERVO_PINS[servo_name]
        self.pi.set_servo_pulsewidth(pin, self.current_pw[servo_name])
        
        # Update status display
        self.servo_status.config(text=f"Moving: {servo_name} {direction}")
        
        # Start continuous movement thread
        thread_id = f"movement_{button_id}"
        self.movement_threads[thread_id] = True
        threading.Thread(target=self.move_servo_continuously, 
                       args=(button_id, thread_id), daemon=True).start()

    def button_release(self, button_id):
        """Handle button release events"""
        # Check if this button was active
        if button_id in self.button_states:
            # Mark button as released
            self.button_states[button_id] = False
            
            # Remove from button states
            del self.button_states[button_id]
            
            # Stop the movement thread
            thread_id = f"movement_{button_id}"
            if thread_id in self.movement_threads:
                self.movement_threads[thread_id] = False
            
            # Parse servo name
            parts = button_id.split()
            servo_name = parts[0]
            
            # If no buttons for this servo are pressed, turn it off
            servo_still_active = False
            for active_button in self.button_states:
                if servo_name in active_button:
                    servo_still_active = True
                    break
            
            if not servo_still_active:
                pin = self.SERVO_PINS[servo_name]
                self.pi.set_servo_pulsewidth(pin, 0)
                self.active_servos.remove(servo_name)
            
            # If no buttons are pressed at all, update status
            if not self.button_states:
                self.servo_status.config(text="Ready - All servos off")

    def move_servo_continuously(self, button_id, thread_id):
        """Move servo continuously while button is pressed"""
        # Parse button ID
        parts = button_id.split()
        servo_name = parts[0]
        direction = parts[1]  # CCW or CW
        
        # Get servo pin and step size
        pin = self.SERVO_PINS[servo_name]
        step_size = self.STEP_SIZES[servo_name]  # Use servo-specific step size
        interval = self.SPEED_INTERVALS[servo_name]  # Use servo-specific interval
        
        # Adjust for speed factor
        effective_interval = interval / self.speed_factor.get()
        
        # Continue moving while thread is active
        while self.movement_threads.get(thread_id, False):
            # Get current pulse width
            current = self.current_pw[servo_name]
            
            # Get limits for this servo
            min_pw, max_pw = self.SERVO_LIMITS[servo_name]
            
            # Calculate new pulse width based on direction
            if direction == "CCW":
                new_pw = min(current + step_size, max_pw)
            else:  # CW
                new_pw = max(current - step_size, min_pw)
            
            # Update position if it changed
            if new_pw != current:
                self.current_pw[servo_name] = new_pw
                try:
                    self.pi.set_servo_pulsewidth(pin, new_pw)
                    
                    # Update position label in main thread
                    self.root.after(0, lambda n=servo_name, p=new_pw: 
                                   self.position_labels[n].config(text=f"{p}μs"))
                except Exception as e:
                    print(f"Error moving {servo_name}: {e}")
                    # If error, exit the loop
                    break
            
            # Sleep for interval
            time.sleep(effective_interval / 1000)  # Convert ms to seconds

    def cleanup(self):
        """Clean up resources when closing the application"""
        print("Cleaning up resources...")
        
        # Stop all movement threads
        for thread_id in list(self.movement_threads.keys()):
            self.movement_threads[thread_id] = False
        
        # Clear any button states
        self.button_states.clear()
        
        # Turn off all servos
        if hasattr(self, 'pi') and self.pi.connected:
            print("Turning off servo motors...")
            self.stop_all_servos()
            
            # Stop pigpio
            self.pi.stop()
            print("All servo motors turned off.")

def main():
    # Check if pigpio daemon is running
    try:
        pi_test = pigpio.pi()
        if not pi_test.connected:
            print("Error: pigpiod daemon is not running.")
            print("Start it with: sudo pigpiod")
            exit(1)
        pi_test.stop()
    except:
        print("Error connecting to pigpio daemon.")
        print("Make sure it's installed and running with: sudo pigpiod")
        exit(1)
    
    # Start GUI
    root = tk.Tk()
    app = RoboticArmController(root)
    
    # Close GPIO properly when window is closed
    root.protocol("WM_DELETE_WINDOW", lambda: [app.cleanup(), root.destroy()])
    
    root.mainloop()

if __name__ == "__main__":
    main()