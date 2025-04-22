import tkinter as tk
from tkinter import ttk
import pigpio
import threading
import time
import math

class ElbowTuningTest:
    """Test harness for fine-tuning elbow servo movement"""
    
    # Default settings for the elbow movement
    DEFAULT_SETTINGS = {
        'GRAVITY_FACTOR': 0.3,         # Strength of gravity compensation
        'EXTRA_SMOOTHING': 0.4,        # Additional smoothing factor for elbow
        'GRAVITY_RATIO_FACTOR': 1.0,   # Multiplier for position ratio effect
        'UPWARD_BOOST': 1.0,           # Extra power for upward movement
        'DOWNWARD_REDUCTION': 0.5,     # Reduction factor for downward movement
        'STEP_SIZE': 15,               # Step size for movement
        'SMOOTHING': 0.8,              # Base smoothing factor
        'ACCELERATION': 0.2,           # Acceleration factor
        'DECELERATION': 0.3,           # Deceleration factor
        'SPEED_INTERVAL': 30,          # Update interval in ms
    }
    
    # Elbow servo limits
    ELBOW_LIMITS = (500, 2500)  # Min/Max pulse widths (μs)
    
    def __init__(self, root):
        self.root = root
        self.root.title("Elbow Servo Tuning Test")
        self.root.geometry("800x600")
        
        # Initialize settings
        self.settings = self.DEFAULT_SETTINGS.copy()
        
        # Connect to pigpio
        self.pi = self.connect_to_pigpio()
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon!")
            return
            
        # Elbow servo configuration
        self.ELBOW_PIN = 27  # Default elbow pin
        
        # Movement tracking variables
        self.current_pw = 1500  # Current pulse width
        self.target_pw = 1500   # Target pulse width
        self.velocity = 0       # Current velocity
        self.active = False     # Is servo active?
        
        # Movement data for plotting
        self.movement_data = []
        
        # Create the UI
        self._setup_ui()
        
        # Start the movement update thread
        self.running = True
        threading.Thread(target=self.movement_loop, daemon=True).start()
    
    def connect_to_pigpio(self):
        """Connect to pigpio daemon"""
        pi = pigpio.pi()
        if pi.connected:
            print("Connected to pigpio daemon")
        else:
            print("Failed to connect to pigpio daemon")
        return pi
    
    def _setup_ui(self):
        """Set up the user interface"""
        # Create main frames
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        feedback_frame = ttk.Frame(self.root, padding=10)
        feedback_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Add title
        title = ttk.Label(control_frame, text="Elbow Servo Tuning", font=("Arial", 16, "bold"))
        title.pack(pady=(0, 10))
        
        # Add pin selector
        pin_frame = ttk.Frame(control_frame)
        pin_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(pin_frame, text="Elbow Pin:").pack(side=tk.LEFT)
        
        self.pin_var = tk.StringVar(value=str(self.ELBOW_PIN))
        pin_entry = ttk.Entry(pin_frame, textvariable=self.pin_var, width=5)
        pin_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(pin_frame, text="Update", command=self.update_pin).pack(side=tk.LEFT)
        
        # Add parameter sliders
        self._create_parameter_sliders(control_frame)
        
        # Add servo control
        control_group = ttk.LabelFrame(control_frame, text="Servo Control", padding=10)
        control_group.pack(fill=tk.X, pady=10)
        
        # Position display
        position_frame = ttk.Frame(control_group)
        position_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(position_frame, text="Current:").pack(side=tk.LEFT)
        self.position_label = ttk.Label(position_frame, text="1500 μs", width=10)
        self.position_label.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(position_frame, text="Target:").pack(side=tk.LEFT)
        self.target_label = ttk.Label(position_frame, text="1500 μs", width=10)
        self.target_label.pack(side=tk.LEFT, padx=5)
        
        # Move buttons
        button_frame = ttk.Frame(control_group)
        button_frame.pack(fill=tk.X, pady=5)
        
        up_btn = ttk.Button(button_frame, text="Move Up (↑)", command=self.move_up)
        up_btn.pack(fill=tk.X, pady=2)
        
        down_btn = ttk.Button(button_frame, text="Move Down (↓)", command=self.move_down)
        down_btn.pack(fill=tk.X, pady=2)
        
        # Add position slider
        self.position_slider = ttk.Scale(
            control_group,
            from_=self.ELBOW_LIMITS[0],
            to=self.ELBOW_LIMITS[1],
            orient=tk.HORIZONTAL,
            command=self.slider_changed
        )
        self.position_slider.set(1500)
        self.position_slider.pack(fill=tk.X, pady=5)
        
        # Test preset buttons
        preset_frame = ttk.Frame(control_group)
        preset_frame.pack(fill=tk.X, pady=5)
        
        # Create a grid of preset positions
        positions = [
            ("Full Retract", self.ELBOW_LIMITS[0]),
            ("1/4", self.ELBOW_LIMITS[0] + (self.ELBOW_LIMITS[1] - self.ELBOW_LIMITS[0])/4),
            ("Half", (self.ELBOW_LIMITS[0] + self.ELBOW_LIMITS[1])/2),
            ("3/4", self.ELBOW_LIMITS[0] + 3*(self.ELBOW_LIMITS[1] - self.ELBOW_LIMITS[0])/4),
            ("Full Extend", self.ELBOW_LIMITS[1])
        ]
        
        for i, (name, pos) in enumerate(positions):
            btn = ttk.Button(preset_frame, text=name, 
                          command=lambda p=pos: self.set_position(p))
            btn.grid(row=i//3, column=i%3, padx=2, pady=2, sticky="ew")
        
        # Add test buttons
        test_frame = ttk.LabelFrame(control_frame, text="Test Sequences", padding=10)
        test_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(test_frame, text="Smooth Up/Down Test", 
                   command=self.test_smooth_updown).pack(fill=tk.X, pady=2)
        
        ttk.Button(test_frame, text="Quick Movement Test", 
                   command=self.test_quick_movements).pack(fill=tk.X, pady=2)
        
        ttk.Button(test_frame, text="Stop Tests", 
                   command=self.stop_tests).pack(fill=tk.X, pady=2)
        
        # Feedback display
        feedback_group = ttk.LabelFrame(feedback_frame, text="Movement Feedback", padding=10)
        feedback_group.pack(fill=tk.BOTH, expand=True)
        
        # Text output area for movement data
        self.feedback_text = tk.Text(feedback_group, height=20, width=40)
        scrollbar = ttk.Scrollbar(feedback_group, command=self.feedback_text.yview)
        self.feedback_text.configure(yscrollcommand=scrollbar.set)
        
        self.feedback_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Save config button
        save_btn = ttk.Button(control_frame, text="Save Configuration", 
                           command=self.save_config)
        save_btn.pack(fill=tk.X, pady=10)
    
    def _create_parameter_sliders(self, parent):
        """Create sliders for all tuning parameters"""
        params_group = ttk.LabelFrame(parent, text="Tuning Parameters", padding=10)
        params_group.pack(fill=tk.X, pady=10)
        
        # Parameter configuration - name, min, max, increment
        params = [
            ("GRAVITY_FACTOR", 0.0, 1.0, 0.05),
            ("EXTRA_SMOOTHING", 0.0, 0.8, 0.05),
            ("GRAVITY_RATIO_FACTOR", 0.0, 2.0, 0.1),
            ("UPWARD_BOOST", 0.5, 2.0, 0.1),
            ("DOWNWARD_REDUCTION", 0.1, 1.0, 0.1),
            ("STEP_SIZE", 5, 30, 1),
            ("SMOOTHING", 0.0, 0.95, 0.05),
            ("ACCELERATION", 0.05, 0.5, 0.05),
            ("DECELERATION", 0.05, 0.5, 0.05),
            ("SPEED_INTERVAL", 10, 100, 5)
        ]
        
        self.param_vars = {}
        
        for name, min_val, max_val, increment in params:
            frame = ttk.Frame(params_group)
            frame.pack(fill=tk.X, pady=2)
            
            # Use shorter display name if possible
            display_name = name.replace("_", " ").title()
            if len(display_name) > 15:
                display_name = ''.join(word[0] for word in display_name.split())
            
            # Create label
            label = ttk.Label(frame, text=display_name, width=15)
            label.pack(side=tk.LEFT)
            
            # Create variable
            if isinstance(self.settings[name], float):
                var = tk.DoubleVar(value=self.settings[name])
            else:
                var = tk.IntVar(value=self.settings[name])
            
            self.param_vars[name] = var
            
            # Create slider
            slider = ttk.Scale(
                frame,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                variable=var,
                command=lambda val, name=name: self.update_param(name, val)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            # Create value display
            value_label = ttk.Label(frame, text=str(var.get()), width=5)
            value_label.pack(side=tk.RIGHT)
            
            # Store reference to label for updates
            var.label = value_label
    
    def update_param(self, name, value):
        """Update a parameter when slider changes"""
        try:
            if isinstance(self.settings[name], float):
                value = float(value)
                self.settings[name] = value
                self.param_vars[name].label.config(text=f"{value:.2f}")
            else:
                value = int(float(value))
                self.settings[name] = value
                self.param_vars[name].label.config(text=str(value))
        except Exception as e:
            print(f"Error updating parameter {name}: {e}")
    
    def update_pin(self):
        """Update the elbow pin"""
        try:
            new_pin = int(self.pin_var.get())
            # Make sure old pin is off
            self.pi.set_servo_pulsewidth(self.ELBOW_PIN, 0)
            # Update pin
            self.ELBOW_PIN = new_pin
            self.status_var.set(f"Pin updated to {new_pin}")
        except ValueError:
            self.status_var.set("Invalid pin number")
    
    def update_feedback(self, message):
        """Update the feedback text area"""
        self.feedback_text.insert(tk.END, message + "\n")
        self.feedback_text.see(tk.END)
    
    def slider_changed(self, value):
        """Handle position slider change"""
        value = float(value)
        self.target_pw = value
        self.target_label.config(text=f"{int(value)} μs")
        self.active = True
        self.pi.set_servo_pulsewidth(self.ELBOW_PIN, int(self.current_pw))
    
    def set_position(self, position):
        """Set a specific position"""
        self.target_pw = position
        self.target_label.config(text=f"{int(position)} μs")
        self.position_slider.set(position)
        self.active = True
        self.pi.set_servo_pulsewidth(self.ELBOW_PIN, int(self.current_pw))
        self.status_var.set(f"Moving to position {int(position)}")
    
    def move_up(self):
        """Move elbow up (decrease pulse width)"""
        new_target = max(self.ELBOW_LIMITS[0], 
                         self.target_pw - self.settings['STEP_SIZE'] * 10)
        self.set_position(new_target)
        self.status_var.set("Moving Up")
    
    def move_down(self):
        """Move elbow down (increase pulse width)"""
        new_target = min(self.ELBOW_LIMITS[1], 
                         self.target_pw + self.settings['STEP_SIZE'] * 10)
        self.set_position(new_target)
        self.status_var.set("Moving Down")
    
    def test_smooth_updown(self):
        """Run a smooth up/down test"""
        self.stop_tests()  # Stop any running tests
        
        self.update_feedback("Starting smooth up/down test")
        
        # Start a new test thread
        self.test_running = True
        threading.Thread(target=self._run_updown_test, daemon=True).start()
    
    def _run_updown_test(self):
        """Run a smooth up/down test sequence"""
        try:
            # Start at middle position
            mid_pos = (self.ELBOW_LIMITS[0] + self.ELBOW_LIMITS[1]) / 2
            self.set_position(mid_pos)
            time.sleep(1)
            
            # Move to retracted position
            self.update_feedback("Moving to retracted position...")
            self.set_position(self.ELBOW_LIMITS[0])
            time.sleep(2)
            
            # Move to extended position
            self.update_feedback("Moving to extended position...")
            self.set_position(self.ELBOW_LIMITS[1])
            time.sleep(2)
            
            # Move back to middle
            self.update_feedback("Moving back to middle...")
            self.set_position(mid_pos)
            time.sleep(1)
            
            # Run three up/down cycles
            for i in range(3):
                if not self.test_running:
                    break
                    
                self.update_feedback(f"Starting cycle {i+1}...")
                
                # Move up
                self.update_feedback("Moving up...")
                self.set_position(self.ELBOW_LIMITS[0])
                time.sleep(1.5)
                
                # Move down
                self.update_feedback("Moving down...")
                self.set_position(self.ELBOW_LIMITS[1])
                time.sleep(1.5)
            
            # Return to middle and stop
            self.set_position(mid_pos)
            self.update_feedback("Test complete")
            
        except Exception as e:
            self.update_feedback(f"Test error: {e}")
        finally:
            time.sleep(0.5)
            self.active = False
            self.pi.set_servo_pulsewidth(self.ELBOW_PIN, 0)
            self.status_var.set("Test Complete")
    
    def test_quick_movements(self):
        """Run a quick movement test"""
        self.stop_tests()  # Stop any running tests
        
        self.update_feedback("Starting quick movement test")
        
        # Start a new test thread
        self.test_running = True
        threading.Thread(target=self._run_quick_test, daemon=True).start()
    
    def _run_quick_test(self):
        """Run a quick movement test sequence"""
        try:
            # Start at middle position
            mid_pos = (self.ELBOW_LIMITS[0] + self.ELBOW_LIMITS[1]) / 2
            self.set_position(mid_pos)
            time.sleep(1)
            
            # Positions to test
            test_positions = [
                self.ELBOW_LIMITS[0],  # Full retract
                mid_pos,               # Middle
                self.ELBOW_LIMITS[1],  # Full extend
                mid_pos,               # Middle
                self.ELBOW_LIMITS[0] + (self.ELBOW_LIMITS[1] - self.ELBOW_LIMITS[0])/4,  # 1/4
                self.ELBOW_LIMITS[0] + 3*(self.ELBOW_LIMITS[1] - self.ELBOW_LIMITS[0])/4,  # 3/4
                mid_pos                # Back to middle
            ]
            
            # Run through positions
            for i, pos in enumerate(test_positions):
                if not self.test_running:
                    break
                    
                self.update_feedback(f"Moving to position {i+1}: {int(pos)}μs")
                self.set_position(pos)
                time.sleep(1)
            
            self.update_feedback("Test complete")
            
        except Exception as e:
            self.update_feedback(f"Test error: {e}")
        finally:
            time.sleep(0.5)
            self.active = False
            self.pi.set_servo_pulsewidth(self.ELBOW_PIN, 0)
            self.status_var.set("Test Complete")
    
    def stop_tests(self):
        """Stop any running tests"""
        self.test_running = False
        time.sleep(0.1)  # Give time for threads to see the flag
        self.active = False
        self.pi.set_servo_pulsewidth(self.ELBOW_PIN, 0)
        self.status_var.set("Tests Stopped")
    
    def save_config(self):
        """Save the current configuration to a file"""
        try:
            with open("elbow_config.txt", "w") as f:
                f.write("# Elbow Movement Configuration\n")
                f.write(f"ELBOW_PIN = {self.ELBOW_PIN}\n\n")
                
                f.write("# Movement Parameters\n")
                for name, value in self.settings.items():
                    f.write(f"{name} = {value}\n")
            
            self.status_var.set("Configuration saved to elbow_config.txt")
            self.update_feedback("Configuration saved to elbow_config.txt")
        except Exception as e:
            self.status_var.set(f"Error saving configuration: {e}")
    
    def smooth_elbow_movement(self, current, target, velocity):
        """
        Specialized elbow movement function with gravity compensation
        
        Parameters:
            current: Current pulse width
            target: Target pulse width
            velocity: Current velocity
            
        Returns:
            new_position: Smoothed position incorporating gravity effects
        """
        # Extract settings for readability
        gravity_factor = self.settings['GRAVITY_FACTOR']
        extra_smoothing = self.settings['EXTRA_SMOOTHING']
        gravity_ratio_factor = self.settings['GRAVITY_RATIO_FACTOR']
        upward_boost = self.settings['UPWARD_BOOST']
        downward_reduction = self.settings['DOWNWARD_REDUCTION']
        base_smoothing = self.settings['SMOOTHING']
        
        # Calculate movement direction
        diff = target - current
        
        # Apply gravity compensation based on current position
        # More compensation when elbow is extended (higher pulse width)
        min_pw, max_pw = self.ELBOW_LIMITS
        position_ratio = (current - min_pw) / (max_pw - min_pw)  # 0 = retracted, 1 = extended
        
        # Adjust position ratio effect
        adjusted_ratio = position_ratio * gravity_ratio_factor
        
        # Create gravity assistance that varies with position and direction
        if diff < 0:  # Moving up against gravity
            gravity_assist = -gravity_factor * adjusted_ratio * self.settings['STEP_SIZE'] * upward_boost
        else:  # Moving down with gravity
            gravity_assist = gravity_factor * (1 - adjusted_ratio) * self.settings['STEP_SIZE'] * downward_reduction
        
        # Log data for analysis
        self.movement_data.append({
            'current': current,
            'target': target,
            'velocity': velocity,
            'gravity_assist': gravity_assist,
            'position_ratio': position_ratio
        })
        
        # Limit data history
        if len(self.movement_data) > 100:
            self.movement_data.pop(0)
        
        # Adjust velocity with gravity compensation
        adjusted_velocity = velocity + gravity_assist
        
        # Calculate new position
        new_position = current + adjusted_velocity
        
        # Apply combined smoothing (base + extra)
        effective_smoothing = min(0.95, base_smoothing + extra_smoothing)
        smoothed_position = current * effective_smoothing + new_position * (1 - effective_smoothing)
        
        # Ensure within limits
        smoothed_position = max(min_pw, min(smoothed_position, max_pw))
        
        return smoothed_position
    
    def movement_loop(self):
        """Main movement loop for handling elbow motion"""
        last_update = time.time()
        feedback_counter = 0
        
        while self.running:
            try:
                # Calculate time delta
                now = time.time()
                dt = now - last_update
                last_update = now
                
                # Skip if not active
                if not self.active:
                    time.sleep(self.settings['SPEED_INTERVAL'] / 1000.0)
                    continue
                
                # Get current values
                current = self.current_pw
                target = self.target_pw
                diff = target - current
                
                # Only move if difference is significant
                if abs(diff) > 1:
                    # Calculate velocity target based on distance
                    distance_factor = min(1.0, abs(diff) / 100)
                    max_velocity = self.settings['STEP_SIZE'] * distance_factor
                    
                    # Apply acceleration/deceleration
                    if abs(self.velocity) < max_velocity:
                        # Accelerate
                        self.velocity += math.copysign(
                            self.settings['ACCELERATION'] * max_velocity, 
                            diff
                        )
                        # Cap velocity
                        self.velocity = math.copysign(
                            min(abs(self.velocity), max_velocity),
                            self.velocity
                        )
                    elif abs(diff) < abs(self.velocity) * 2:
                        # Decelerate as we approach target
                        self.velocity *= (1.0 - self.settings['DECELERATION'])
                    
                    # Ensure velocity direction matches target direction
                    if (self.velocity * diff) < 0:
                        # If velocity direction opposes target direction, decelerate faster
                        self.velocity *= (1.0 - self.settings['DECELERATION'] * 2)
                    
                    # Apply the smooth elbow movement function
                    new_position = self.smooth_elbow_movement(
                        current, target, self.velocity
                    )
                    
                    # Update current position
                    self.current_pw = new_position
                    
                    # Update servo position
                    self.pi.set_servo_pulsewidth(self.ELBOW_PIN, int(new_position))
                    
                    # Update UI
                    self.root.after_idle(
                        lambda p=int(new_position): self.position_label.config(text=f"{p} μs")
                    )
                    
                    # Periodically update feedback (not every cycle to avoid overload)
                    feedback_counter += 1
                    if feedback_counter >= 10:
                        feedback_counter = 0
                        data = self.movement_data[-1] if self.movement_data else {}
                        self.root.after_idle(
                            lambda: self.update_feedback(
                                f"Pos: {int(new_position)}, Target: {int(target)}, "
                                f"Vel: {self.velocity:.2f}, Assist: {data.get('gravity_assist', 0):.2f}"
                            )
                        )
                else:
                    # Reached target, stop velocity
                    self.velocity = 0
                    self.current_pw = target
                    
                    # Update final position
                    self.pi.set_servo_pulsewidth(self.ELBOW_PIN, int(target))
                    
                    # Update UI
                    self.root.after_idle(
                        lambda p=int(target): self.position_label.config(text=f"{p} μs")
                    )
                    
                    # Log reached target
                    feedback_counter += 1
                    if feedback_counter >= 10:
                        feedback_counter = 0
                        self.root.after_idle(
                            lambda: self.update_feedback(f"Reached target: {int(target)}")
                        )
            
            except Exception as e:
                print(f"Error in movement loop: {e}")
            
            # Pause for movement interval
            time.sleep(self.settings['SPEED_INTERVAL'] / 1000.0)
    
    def on_closing(self):
        """Clean shutdown when window is closed"""
        self.running = False
        time.sleep(0.1)  # Give time for threads to close
        
        # Make sure servo is off
        self.pi.set_servo_pulsewidth(self.ELBOW_PIN, 0)
        
        # Stop pigpio connection
        self.pi.stop()
        
        # Destroy window
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ElbowTuningTest(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()