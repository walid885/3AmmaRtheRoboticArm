import tkinter as tk
import pigpio
import threading
import time
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RoboticArmController:
    # Updated servo pin configuration as specified
    SERVO_PINS = {
        'BASE': 24,        # Base rotation
        'SHOULDER': 23,    # Shoulder joint
        'ELBOW': 22,       # Elbow joint
        'WRIST_PITCH': 27, # Wrist up/down
        'WRIST_ROLL': 18,  # Wrist rotation
        'GRIPPER': 17      # End effector/gripper
    }
    
    # Min/Max pulse widths (in μs) for each servo
    SERVO_LIMITS = {
        'BASE': (600, 2400),
        'SHOULDER': (700, 2300),  # MG996R Pro specific range
        'ELBOW': (600, 2400),
        'WRIST_PITCH': (600, 2400),
        'WRIST_ROLL': (600, 2400),
        'GRIPPER': (1000, 2000)   # Closed to Open
    }
    
    # Default step size and interval for servo movements
    DEFAULT_STEP = 10      # Smaller step size for smoother movement
    DEFAULT_INTERVAL = 30  # Faster updates for more responsive control
    COMMAND_DELAY = 0.08  # 30ms delay
    ACCEL_STEPS = 5        # Number of steps for acceleration/deceleration
        
    # Safety delay between servo commands to prevent signal interference
    COMMAND_DELAY = 0.5   # 20ms delay

    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Robotic Arm Control")
        self.root.geometry("800x600")
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            logger.error("Failed to connect to pigpio daemon!")
            return
            
        # Current pulse width for each servo
        self.current_pw = {pin_name: 1500 for pin_name in self.SERVO_PINS}
        
        # Active movement locks to prevent conflicting commands
        self.movement_lock = threading.Lock()
        
        # Button states tracking
        self.button_states = {}
        
        # Initialize all servos with a delay between each to prevent power spikes
        logger.info("Initializing servos...")
        for name, pin in self.SERVO_PINS.items():
            logger.info(f"Setting {name} (pin {pin}) to 1500μs")
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            time.sleep(0.2)  # Longer delay during initialization
            
        self._setup_ui()
        logger.info("Robotic arm controller initialized and ready")
    
    def _move_with_acceleration(self, servo_name, target_pw):
        """Move a servo with an acceleration/deceleration profile"""
        pin = self.SERVO_PINS[servo_name]
        current = self.current_pw[servo_name]
        
        # Calculate total movement and direction
        total_move = target_pw - current
        if total_move == 0:
            return  # Already at target
            
        direction = 1 if total_move > 0 else -1
        total_steps = abs(total_move)
        
        # Make acceleration profile
        if total_steps <= 10:
            # For very small movements, move directly
            increments = [direction * 1] * total_steps
        else:
            # For larger movements, create an acceleration curve
            accel_steps = min(self.ACCEL_STEPS, total_steps // 3)
            
            # Acceleration phase
            accel = [direction * (i+1) for i in range(accel_steps)]
            
            # Constant velocity phase
            mid_steps = total_steps - (2 * accel_steps)
            constant = [direction * accel_steps] * max(0, mid_steps)
            
            # Deceleration phase
            decel = [direction * (accel_steps - i) for i in range(accel_steps)]
            
            # Combine phases
            increments = accel + constant + decel
            
            # Adjust for total distance
            actual_distance = sum(increments)
            if actual_distance != total_move:
                adjustment = total_move - actual_distance
                if mid_steps > 0:
                    # Adjust the constant velocity phase
                    constant = [direction * accel_steps] * (mid_steps + adjustment)
                    increments = accel + constant + decel
                else:
                    # If no constant phase, adjust last increment
                    increments[-1] += adjustment
        
        # Execute the movement profile
        current_pos = current
        for increment in increments:
            current_pos += increment
            
            # Ensure within limits
            min_pw, max_pw = self.SERVO_LIMITS[servo_name]
            current_pos = max(min_pw, min(current_pos, max_pw))
            
            # Move servo
            self.pi.set_servo_pulsewidth(pin, current_pos)
            self.current_pw[servo_name] = current_pos
            
            # Update UI
            self.root.after(0, lambda n=servo_name, p=current_pos: 
                        self.position_labels[n].config(text=f"{p}μs"))
            
            # Power load monitoring
            self._check_movement_frequency()
            
            # Delay based on power stability
            delay = self.COMMAND_DELAY * (2.0 if not self.power_stable else 1.0)
            time.sleep(delay)
    def _check_movement_frequency(self):
        """Check if movements are happening too frequently which might indicate power issues"""
        current_time = time.time()
        time_diff = current_time - self.last_movement_time
        
        # If movements are happening in rapid succession
        if time_diff < 0.1:
            self.movement_count += 1
            
            # If too many rapid movements, might indicate power issues
            if self.movement_count > 20 and self.power_stable:
                self.power_stable = False
                self.root.after(0, lambda: self.power_status.config(
                    text="Power: Unstable", bg="red"))
                logger.warning("Possible power instability detected")
        else:
            # Reset counter for normal operation
            self.movement_count = 0
        
        self.last_movement_time = current_time

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
        
        self.servo_status = tk.Label(main_frame, text="Ready", 
                                     font=("Arial", 12), bg="white", width=30, height=2, 
                                     relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 15))
        
        # Debug info section
        self.debug_var = tk.BooleanVar(value=False)
        debug_frame = tk.Frame(main_frame, bg="#f0f0f0")
        debug_frame.pack(fill=tk.X)
        
        debug_cb = tk.Checkbutton(debug_frame, text="Enable Debug Info", 
                                variable=self.debug_var, bg="#f0f0f0")
        debug_cb.pack(side=tk.RIGHT)
        
        # Position feedback
        self.feedback_frame = tk.Frame(main_frame, bg="#f0f0f0")
        self.feedback_frame.pack(fill=tk.X, pady=10)
        
        self.position_labels = {}
        for i, name in enumerate(self.SERVO_PINS.keys()):
            row = i // 3
            col = i % 3
            
            label_frame = tk.Frame(self.feedback_frame, bg="#f0f0f0")
            label_frame.grid(row=row, column=col, padx=10, pady=5)
            
            tk.Label(label_frame, text=f"{name} (Pin {self.SERVO_PINS[name]}):", 
                    font=("Arial", 10), bg="#f0f0f0").pack(side=tk.LEFT)
            
            self.position_labels[name] = tk.Label(label_frame, text="1500μs", 
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
            ("Test Pins", self.test_all_servos)  # New test function
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
            
            label = tk.Label(button_frame, text=f"{name}\nPin: {self.SERVO_PINS[name]}", 
                           font=("Arial", 10, "bold"), bg="#f0f0f0")
            label.pack(pady=(0, 5))
            
            btn_frame = tk.Frame(button_frame, bg="#f0f0f0")
            btn_frame.pack()
            
            # CCW button (increases pulse width)
            ccw_btn = tk.Button(btn_frame, text="↑", bg=bg_color, fg=fg_color,
                               font=("Arial", 12, "bold"), width=4, height=2)
            ccw_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind events with proper lambda to avoid late binding issues
            button_id = f"{name} CCW"
            ccw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            ccw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))
            
            # CW button (decreases pulse width)
            cw_btn = tk.Button(btn_frame, text="↓", bg=bg_color, fg=fg_color,
                              font=("Arial", 12, "bold"), width=4, height=2)
            cw_btn.pack(side=tk.LEFT, padx=2)
            
            button_id = f"{name} CW"
            cw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            cw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))

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
        self.move_to_preset(home_positions, "Home")

    def pick_position(self):
        """Move to picking position"""
        pick_positions = {
            'BASE': 1800,
            'SHOULDER': 1200,
            'ELBOW': 1000,
            'WRIST_PITCH': 1700,
            'WRIST_ROLL': 1500,
            'GRIPPER': 2000  # Open
        }
        self.move_to_preset(pick_positions, "Pick")

    def place_position(self):
        """Move to placing position"""
        place_positions = {
            'BASE': 1200,
            'SHOULDER': 1500,
            'ELBOW': 1800,
            'WRIST_PITCH': 1300,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1000  # Closed
        }
        self.move_to_preset(place_positions, "Place")

    def rest_position(self):
        """Move to rest position"""
        rest_positions = {
            'BASE': 1500,
            'SHOULDER': 2000,
            'ELBOW': 2000,
            'WRIST_PITCH': 1500,
            'WRIST_ROLL': 1500,
            'GRIPPER': 1500
        }
        self.move_to_preset(rest_positions, "Rest")

    def test_all_servos(self):
        """Test function to verify all servos and pins"""
        self.servo_status.config(text="Testing all servos sequentially...")
        
        # Stop any ongoing movements
        self.button_states.clear()
        
        # Create a separate thread for the test
        threading.Thread(target=self._test_servos_thread, daemon=True).start()
        
    def _test_servos_thread(self):
        """Thread function for testing each servo individually"""
        test_range = 300  # Amount to move in each direction
        
        try:
            # Test each servo individually
            for name, pin in self.SERVO_PINS.items():
                # Update status in main thread
                self.root.after(0, lambda n=name: 
                               self.servo_status.config(text=f"Testing {n} (Pin {pin})..."))
                
                logger.info(f"Testing {name} on pin {pin}")
                
                # Get current position and limits
                current = self.current_pw[name]
                min_pw, max_pw = self.SERVO_LIMITS[name]
                
                # Test sequence: current -> current+test_range -> current -> current-test_range -> current
                test_positions = [
                    current,
                    min(current + test_range, max_pw),
                    current,
                    max(current - test_range, min_pw),
                    current
                ]
                
                # Move through test positions
                for pos in test_positions:
                    # Move to position
                    logger.info(f"  Moving {name} to {pos}μs")
                    self.pi.set_servo_pulsewidth(pin, pos)
                    self.current_pw[name] = pos
                    
                    # Update UI in main thread
                    self.root.after(0, lambda n=name, p=pos: 
                                   self.position_labels[n].config(text=f"{p}μs"))
                    
                    # Wait for movement to complete
                    time.sleep(0.5)
                
                # Pause between servos
                time.sleep(0.5)
                
            # Update status when done
            self.root.after(0, lambda: self.servo_status.config(text="Testing complete"))
            logger.info("All servo tests completed")
            
        except Exception as e:
            logger.error(f"Error during servo test: {e}")
            self.root.after(0, lambda: self.servo_status.config(text=f"Error: {e}"))

    def move_to_preset(self, positions, preset_name):
        """Move all servos to preset positions gradually"""
        # Stop any ongoing movements
        self.button_states.clear()
        self.servo_status.config(text=f"Moving to {preset_name} position...")
        
        # Create a separate thread for movement to avoid blocking GUI
        threading.Thread(target=self._move_to_preset_thread, 
                        args=(positions, preset_name), daemon=True).start()

    def _move_to_preset_thread(self, target_positions, preset_name):
        """Thread function for smooth movement to preset positions"""
        try:
            with self.movement_lock:
                # Calculate steps for each servo
                steps = {}
                current_positions = self.current_pw.copy()
                
                for name, target in target_positions.items():
                    if name in self.SERVO_PINS:
                        steps[name] = (target - current_positions[name]) / 50  # 50 steps total
                
                # Move in small increments
                for step in range(50):
                    # Move one servo at a time to reduce power load
                    for name, increment in steps.items():
                        pin = self.SERVO_PINS[name]
                        new_pw = int(current_positions[name] + increment * (step + 1))
                        
                        # Ensure within limits
                        min_pw, max_pw = self.SERVO_LIMITS[name]
                        new_pw = max(min_pw, min(new_pw, max_pw))
                        
                        # Update position
                        self.current_pw[name] = new_pw
                        self.pi.set_servo_pulsewidth(pin, new_pw)
                        
                        # Update UI in main thread
                        self.root.after(0, lambda n=name, p=new_pw: 
                                       self.position_labels[n].config(text=f"{p}μs"))
                        
                        # Short delay between servo commands to prevent interference
                        time.sleep(self.COMMAND_DELAY)
                    
                    # Delay between steps
                    time.sleep(0.01)
                
                # Update status when done
                self.root.after(0, lambda: self.servo_status.config(text=f"{preset_name} position reached"))
                logger.info(f"Moved to {preset_name} position")
                
        except Exception as e:
            logger.error(f"Error moving to preset position: {e}")
            self.root.after(0, lambda: self.servo_status.config(text=f"Error: {e}"))

    def emergency_stop(self):
        """Stop all servo movements immediately"""
        # Clear button states
        self.button_states.clear()
        
        # Update status
        self.servo_status.config(text="EMERGENCY STOP ACTIVATED")
        logger.warning("Emergency stop activated")
        
        # Stop all servo pulses
        for name, pin in self.SERVO_PINS.items():
            logger.info(f"Stopping {name} on pin {pin}")
            self.pi.set_servo_pulsewidth(pin, 0)
            time.sleep(0.05)  # Longer pause between commands during emergency
        
        # Reactivate servos at current position after a short delay
        self.root.after(1000, self.reactivate_servos)

    def reactivate_servos(self):
        """Reactivate servos after emergency stop"""
        logger.info("Reactivating servos...")
        
        # Reactivate one servo at a time with delay to prevent power spike
        for name, pin in self.SERVO_PINS.items():
            logger.info(f"Reactivating {name} on pin {pin} at {self.current_pw[name]}μs")
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            self.position_labels[name].config(text=f"{self.current_pw[name]}μs")
            time.sleep(0.2)  # Longer delay during reactivation
        
        self.servo_status.config(text="Ready")
        logger.info("All servos reactivated")

    def button_press(self, button_id):
        """Handle button press events"""
        # Parse button ID to get servo name and direction
        parts = button_id.split()
        servo_name = parts[0]
        direction = parts[1]  # CCW or CW
        
        # Get servo pin
        pin = self.SERVO_PINS[servo_name]
        
        # Debug info
        if self.debug_var.get():
            logger.info(f"Button press: {button_id} (Pin {pin})")
        
        # Mark button as pressed
        self.button_states[button_id] = True
        
        # Update status display
        self.servo_status.config(text=f"Moving: {servo_name} {direction}")
        
        # Start continuous movement
        threading.Thread(target=self.move_servo_continuously, 
                        args=(button_id,), daemon=True).start()

    def button_release(self, button_id):
        """Handle button release events"""
        # Check if this button was active
        if button_id in self.button_states:
            # Mark button as released
            self.button_states[button_id] = False
            
            # Get servo name
            servo_name = button_id.split()[0]
            pin = self.SERVO_PINS[servo_name]
            
            # Debug info
            if self.debug_var.get():
                logger.info(f"Button release: {button_id} (Pin {pin})")
            
            # Remove from button states
            del self.button_states[button_id]
            
            # Clean up: make sure pulse width is stable
            self.pi.set_servo_pulsewidth(pin, self.current_pw[servo_name])
            
            # Update status if no buttons are pressed
            if not self.button_states:
                self.servo_status.config(text="Ready")

    def move_servo_continuously(self, button_id):
        """Move servo continuously while button is pressed with acceleration control"""
        # Initial delay to prevent bounce
        time.sleep(0.05)
        
        # Skip if button already released
        if button_id not in self.button_states:
            return
            
        # Start with slow movements
        current_step = self.DEFAULT_STEP // 2
            
        with self.movement_lock:
            while button_id in self.button_states and self.button_states[button_id]:
                # Parse button ID
                parts = button_id.split()
                servo_name = parts[0]
                direction = parts[1]  # CCW or CW
                
                # Get servo pin and current position
                pin = self.SERVO_PINS[servo_name]
                current = self.current_pw[servo_name]
                
                # Get limits for this servo
                min_pw, max_pw = self.SERVO_LIMITS[servo_name]
                
                # Gradually increase step size for acceleration
                if current_step < self.DEFAULT_STEP:
                    current_step += 1
                
                # Calculate new pulse width based on direction
                if direction == "CCW":
                    new_pw = min(current + current_step, max_pw)
                else:  # CW
                    new_pw = max(current - current_step, min_pw)
                
                # Update position if it changed
                if new_pw != current:
                    # Debug info
                    if self.debug_var.get():
                        logger.info(f"Moving {servo_name} to {new_pw}μs (Pin {pin})")
                    
                    # Update internal state first
                    self.current_pw[servo_name] = new_pw
                    
                    # Then send command to servo
                    self.pi.set_servo_pulsewidth(pin, new_pw)
                    
                    # Update position label in main thread
                    self.root.after(0, lambda n=servo_name, p=new_pw: 
                                self.position_labels[n].config(text=f"{p}μs"))
                    
                    # Check for potential power issues
                    self._check_movement_frequency()
                
                # Wait before next movement - adjust based on power stability
                interval = self.DEFAULT_INTERVAL * (1.5 if not self.power_stable else 1.0)
                time.sleep(interval / 1000)  # Convert ms to seconds
    def cleanup(self):
        """Clean up resources when closing the application"""
        logger.info("Cleaning up resources...")
        
        # Clear any button states
        self.button_states.clear()
        
        # Turn off all servos by stopping pulses
        if hasattr(self, 'pi') and self.pi.connected:
            logger.info("Turning off servo motors...")
            for name, pin in self.SERVO_PINS.items():
                logger.info(f"Turning off {name} on pin {pin}")
                self.pi.set_servo_pulsewidth(pin, 0)  # Stop the pulse
                time.sleep(0.1)  # Brief pause between commands
            
            # Stop pigpio
            self.pi.stop()
            logger.info("All servo motors turned off.")
    def calibrate_servos(self):
        """Calibrate each servo to find its actual range"""
        self.servo_status.config(text="Starting servo calibration...")
        
        # Stop any ongoing movements
        self.button_states.clear()
        
        # Create a separate thread for calibration
        threading.Thread(target=self._calibrate_servos_thread, daemon=True).start()
    
    def _calibrate_servos_thread(self):
        """Thread function for calibrating each servo individually"""
        try:
            # For each servo, move slowly through entire range to detect limits
            for name, pin in self.SERVO_PINS.items():
                # Update status
                self.root.after(0, lambda n=name: 
                            self.servo_status.config(text=f"Calibrating {n}..."))
                
                min_pw, max_pw = self.SERVO_LIMITS[name]
                
                # First, move to center position gently
                center = (min_pw + max_pw) // 2
                self._move_with_acceleration(name, center)
                time.sleep(1.0)  # Wait for stability
                
                # Then move slowly toward minimum
                self.root.after(0, lambda n=name: 
                            self.servo_status.config(text=f"Finding minimum for {n}..."))
                
                # Test lower range with extra caution
                for test_pw in range(center, min_pw - 100, -10):
                    # Safety boundary check
                    if test_pw < min_pw - 50:
                        break
                        
                    self.pi.set_servo_pulsewidth(pin, test_pw)
                    self.current_pw[name] = test_pw
                    self.root.after(0, lambda n=name, p=test_pw: 
                                self.position_labels[n].config(text=f"{p}μs"))
                    time.sleep(0.1)  # Slower movement for calibration
                
                # Return to center
                self._move_with_acceleration(name, center)
                time.sleep(1.0)
                
                # Test upper range
                self.root.after(0, lambda n=name: 
                            self.servo_status.config(text=f"Finding maximum for {n}..."))
                
                for test_pw in range(center, max_pw + 100, 10):
                    # Safety boundary check
                    if test_pw > max_pw + 50:
                        break
                        
                    self.pi.set_servo_pulsewidth(pin, test_pw)
                    self.current_pw[name] = test_pw
                    self.root.after(0, lambda n=name, p=test_pw: 
                                self.position_labels[n].config(text=f"{p}μs"))
                    time.sleep(0.1)
                
                # Return to center again
                self._move_with_acceleration(name, center)
                time.sleep(1.0)
            
            # Complete calibration
            self.root.after(0, lambda: self.servo_status.config(text="Calibration complete"))
            logger.info("Servo calibration completed")
            
        except Exception as e:
            logger.error(f"Error during calibration: {e}")
            self.root.after(0, lambda: self.servo_status.config(text=f"Calibration error: {e}"))

def main():
    # Check if pigpio daemon is running
    try:
        pi_test = pigpio.pi()
        if not pi_test.connected:
            logger.error("Error: pigpiod daemon is not running.")
            print("Error: pigpiod daemon is not running.")
            print("Start it with: sudo pigpiod")
            exit(1)
        pi_test.stop()
    except Exception as e:
        logger.error(f"Error connecting to pigpio daemon: {e}")
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