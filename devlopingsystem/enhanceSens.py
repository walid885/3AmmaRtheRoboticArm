import tkinter as tk
from tkinter import ttk, messagebox
import pigpio
import threading
import time
import math

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
    
    # Default sensitivity settings (can be tuned via UI)
    DEFAULT_SETTINGS = {
        'STEP_SIZE': 15,       # Default step size for button control
        'SPEED_INTERVAL': 30,  # Default speed interval (ms)
        'ACCELERATION': 0.2,   # Acceleration factor (0-1), higher = faster acceleration
        'DECELERATION': 0.3,   # Deceleration factor (0-1), higher = faster stop
        'SMOOTHING': 0.8,      # Movement smoothing factor (0-1), higher = smoother
        'MOUSE_SENSITIVITY': 5 # Mouse sensitivity factor
    }
    
    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Robotic Arm Control")
        self.root.geometry("1000x700")
        
        # Initialize settings with defaults
        self.settings = self.DEFAULT_SETTINGS.copy()
        
        # Initialize pigpio with multiple connection attempts
        self.pi = self.connect_to_pigpio(max_attempts=3)
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon!")
            messagebox.showerror("Connection Error", "Failed to connect to pigpio daemon!")
            return
            
        # Current pulse width for each servo
        self.current_pw = {pin_name: 1500 for pin_name in self.SERVO_PINS}
        
        # Target pulse width for smooth movement
        self.target_pw = {pin_name: 1500 for pin_name in self.SERVO_PINS}
        
        # Current movement velocities (for smoother acceleration/deceleration)
        self.velocities = {pin_name: 0 for pin_name in self.SERVO_PINS}
        
        # Active servo tracking
        self.active_servos = set()
        
        # Button states tracking
        self.button_states = {}
        
        # Movement threads dict
        self.movement_threads = {}
        
        # Mouse drag state
        self.mouse_servo = None
        self.mouse_start_x = 0
        self.mouse_start_y = 0
        self.mouse_last_pos = (0, 0)
        self.mouse_last_time = 0
        
        self._setup_ui()
        
        # Initialize all servos to middle position then turn them off
        self.initialize_servos()
        
        # Start the smooth movement updater thread
        threading.Thread(target=self.smooth_movement_loop, daemon=True).start()
        
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
            self.target_pw[name] = middle_pw
            
            try:
                self.pi.set_servo_pulsewidth(pin, middle_pw)
                # Update position label
                if hasattr(self, 'position_labels') and name in self.position_labels:
                    self.position_labels[name].config(text=f"{middle_pw}μs")
                # Update slider position
                if hasattr(self, 'servo_sliders') and name in self.servo_sliders:
                    self.servo_sliders[name].set(middle_pw)
            except Exception as e:
                print(f"Error initializing {name}: {e}")
            
            time.sleep(0.2)  # Longer pause to allow servo to reach position
        
        # Then turn all off to save power
        time.sleep(1.0)  # Wait for servos to reach position
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, 0)  # Stop the pulse

    def _setup_ui(self):
        """Set up the user interface"""
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 10))
        style.configure('TLabel', font=('Arial', 10))
        style.configure('TFrame', background='#f0f0f0')
        
        # Create a tabbed interface
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Main control tab
        control_tab = ttk.Frame(self.notebook)
        settings_tab = ttk.Frame(self.notebook)
        visual_tab = ttk.Frame(self.notebook)
        
        self.notebook.add(control_tab, text="Controls")
        self.notebook.add(visual_tab, text="Visual Control")
        self.notebook.add(settings_tab, text="Settings")
        
        # Set up the main control tab
        self._setup_control_tab(control_tab)
        
        # Set up the visual/mouse control tab
        self._setup_visual_tab(visual_tab)
        
        # Set up the settings tab
        self._setup_settings_tab(settings_tab)

    def _setup_control_tab(self, parent):
        """Set up the main control tab"""
        # Configure grid layout
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=0)  # Title
        parent.rowconfigure(1, weight=0)  # Status
        parent.rowconfigure(2, weight=0)  # Position feedback
        parent.rowconfigure(3, weight=1)  # Controls container
        parent.rowconfigure(4, weight=0)  # Speed control
        parent.rowconfigure(5, weight=0)  # Presets
        parent.rowconfigure(6, weight=0)  # Emergency stop
        
        # Title and status section
        title_frame = ttk.Frame(parent)
        title_frame.grid(row=0, column=0, sticky="ew", pady=5)
        
        title_label = ttk.Label(title_frame, text="ROBOTIC ARM CONTROLLER", 
                             font=("Arial", 16, "bold"))
        title_label.pack()
        
        # Status display
        status_frame = ttk.Frame(parent)
        status_frame.grid(row=1, column=0, sticky="ew", pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Robotic Arm Status:", 
                                     font=("Arial", 12))
        self.status_label.pack(pady=(5, 2))
        
        self.servo_status = ttk.Label(status_frame, text="Ready - All servos off", 
                                     font=("Arial", 10), background="white", width=40)
        self.servo_status.pack(pady=(0, 5))
        
        # Position feedback with sliders
        feedback_frame = ttk.Frame(parent)
        feedback_frame.grid(row=2, column=0, sticky="ew", pady=5)
        
        self.position_labels = {}
        self.servo_sliders = {}
        
        # Create a frame for each row of servos (3 per row)
        for i, name in enumerate(self.SERVO_PINS.keys()):
            row = i // 3
            col = i % 3
            
            servo_frame = ttk.Frame(feedback_frame, padding=5)
            servo_frame.grid(row=row, column=col, padx=10, pady=5, sticky="nsew")
            
            # Label with servo name and range
            min_pw, max_pw = self.SERVO_LIMITS[name]
            label_text = f"{name} ({min_pw}-{max_pw}):"
            ttk.Label(servo_frame, text=label_text).pack(anchor="w")
            
            # Add position display
            self.position_labels[name] = ttk.Label(servo_frame, text="1500μs", 
                                              background="white", width=8)
            self.position_labels[name].pack(anchor="w", pady=2)
            
            # Add slider for direct manual control
            self.servo_sliders[name] = ttk.Scale(
                servo_frame, 
                from_=min_pw, 
                to=max_pw,
                orient="horizontal",
                length=150,
                command=lambda value, n=name: self.on_slider_change(n, value)
            )
            self.servo_sliders[name].set(1500)  # Middle position
            self.servo_sliders[name].pack(fill="x", pady=2)
        
        # Create servo control sections (buttons)
        controls_container = ttk.Frame(parent)
        controls_container.grid(row=3, column=0, sticky="nsew", pady=5)
        
        # Upper arm controls (Row 1)
        self.create_servo_section(controls_container, "UPPER ARM CONTROLS", 
                                ["BASE", "SHOULDER", "ELBOW"], 0)
        
        # Lower arm controls (Row 2)
        self.create_servo_section(controls_container, "LOWER ARM CONTROLS", 
                                ["WRIST_PITCH", "WRIST_ROLL", "GRIPPER"], 1)
        
        # Add speed control slider
        speed_frame = ttk.Frame(parent)
        speed_frame.grid(row=4, column=0, sticky="ew", pady=5)
        
        ttk.Label(speed_frame, text="Movement Speed:").pack(side=tk.LEFT, padx=10)
        
        self.speed_factor = tk.DoubleVar(value=1.0)
        speed_slider = ttk.Scale(speed_frame, from_=0.5, to=2.0, 
                               orient=tk.HORIZONTAL, length=200, 
                               variable=self.speed_factor)
        speed_slider.pack(side=tk.LEFT, padx=10)
        ttk.Label(speed_frame, text="Slow").pack(side=tk.LEFT)
        ttk.Label(speed_frame, text="Fast").pack(side=tk.RIGHT, padx=10)
        
        # Create preset positions buttons
        preset_frame = ttk.Frame(parent)
        preset_frame.grid(row=5, column=0, sticky="ew", pady=5)
        
        ttk.Label(preset_frame, text="PRESET POSITIONS", 
                font=("Arial", 12, "bold")).pack(pady=(0, 5))
        
        btn_frame = ttk.Frame(preset_frame)
        btn_frame.pack()
        
        presets = [
            ("Home", self.home_position),
            ("Pick", self.pick_position),
            ("Place", self.place_position),
            ("Rest", self.rest_position),
            ("Test Servos", self.test_all_servos)
        ]
        
        for i, (name, command) in enumerate(presets):
            btn = ttk.Button(btn_frame, text=name, command=command, width=12)
            btn.grid(row=0, column=i, padx=5)
        
        # Emergency stop button
        stop_frame = ttk.Frame(parent)
        stop_frame.grid(row=6, column=0, sticky="ew", pady=10)
        
        stop_btn = ttk.Button(stop_frame, text="EMERGENCY STOP", 
                           command=self.emergency_stop, 
                           style="Stop.TButton", width=20)
        style.configure("Stop.TButton", background="red", foreground="white", 
                       font=("Arial", 12, "bold"))
        stop_btn.pack()

    def _setup_visual_tab(self, parent):
        """Set up the visual control tab with mouse interaction"""
        # Configure grid layout
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=0)  # Title
        parent.rowconfigure(1, weight=1)  # Canvas area
        parent.rowconfigure(2, weight=0)  # Instructions
        
        # Title
        title_label = ttk.Label(parent, text="VISUAL ARM CONTROL", 
                             font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, pady=10)
        
        # Create frame for the visual representation
        visual_frame = ttk.Frame(parent, padding=10)
        visual_frame.grid(row=1, column=0, sticky="nsew")
        
        # Canvas for the arm visualization
        self.canvas_frame = tk.Frame(visual_frame, bg="white", 
                                   width=800, height=400, bd=2, relief=tk.SUNKEN)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create individual control areas for each servo
        self.visual_controls = {}
        
        # Organize servos in a 2x3 grid
        servo_names = list(self.SERVO_PINS.keys())
        rows, cols = 2, 3
        
        for i, name in enumerate(servo_names):
            row = i // cols
            col = i % cols
            
            # Create control box for each servo
            ctrl_frame = tk.Frame(self.canvas_frame, bg="#e0e0e0", 
                                bd=1, relief=tk.RAISED)
            ctrl_frame.place(relx=col/cols + 0.02, rely=row/rows + 0.05, 
                           relwidth=1/cols - 0.04, relheight=1/rows - 0.1)
            
            # Add servo name label
            tk.Label(ctrl_frame, text=name, font=("Arial", 12, "bold"), 
                   bg="#e0e0e0").pack(pady=5)
            
            # Add control surface (where mouse will interact)
            control_surface = tk.Canvas(ctrl_frame, bg="white", 
                                      highlightthickness=1, highlightbackground="gray")
            control_surface.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            # Add indicator
            indicator = control_surface.create_oval(
                75, 75, 125, 125, fill="blue", outline="black"
            )
            
            # Store reference
            self.visual_controls[name] = {
                "canvas": control_surface,
                "indicator": indicator
            }
            
            # Bind mouse events
            control_surface.bind("<ButtonPress-1>", 
                               lambda e, n=name: self.mouse_control_start(e, n))
            control_surface.bind("<ButtonRelease-1>", 
                               lambda e, n=name: self.mouse_control_end(e, n))
            control_surface.bind("<B1-Motion>", 
                               lambda e, n=name: self.mouse_control_move(e, n))
        
        # Instructions
        instr_frame = ttk.Frame(parent, padding=10)
        instr_frame.grid(row=2, column=0, sticky="ew")
        
        instructions = """
        Mouse Controls:
        1. Click and drag in any servo box to control that servo
        2. Horizontal movement controls primary axis
        3. Release mouse to stop movement
        """
        
        ttk.Label(instr_frame, text=instructions, 
                font=("Arial", 10)).pack()

    def _setup_settings_tab(self, parent):
        """Set up the settings tab for tuning movement parameters"""
        # Configure grid layout
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=0)  # Title
        parent.rowconfigure(1, weight=1)  # Settings
        
        # Title
        title_label = ttk.Label(parent, text="MOVEMENT SETTINGS", 
                             font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, pady=10)
        
        # Create frame for settings
        settings_frame = ttk.Frame(parent, padding=10)
        settings_frame.grid(row=1, column=0, sticky="nsew")
        
        # Setting sliders
        settings = [
            ("Step Size", "STEP_SIZE", 1, 50, 
             "Controls how much the servo moves with each button press"),
            
            ("Speed Interval (ms)", "SPEED_INTERVAL", 10, 100, 
             "Controls the update interval for movements (lower = faster)"),
            
            ("Acceleration", "ACCELERATION", 0.0, 1.0, 
             "How quickly servos accelerate (higher = faster acceleration)"),
            
            ("Deceleration", "DECELERATION", 0.0, 1.0, 
             "How quickly servos stop (higher = faster stop)"),
            
            ("Smoothing", "SMOOTHING", 0.0, 0.99, 
             "Movement smoothing (higher = smoother but more latency)"),
            
            ("Mouse Sensitivity", "MOUSE_SENSITIVITY", 1, 20, 
             "Controls how sensitive the mouse control is")
        ]
        
        self.setting_vars = {}
        
        for i, (label, setting_key, min_val, max_val, description) in enumerate(settings):
            frame = ttk.Frame(settings_frame)
            frame.pack(fill=tk.X, pady=5)
            
            ttk.Label(frame, text=label, width=20).pack(side=tk.LEFT)
            
            # Show current value
            value_label = ttk.Label(frame, text=str(self.settings[setting_key]))
            value_label.pack(side=tk.RIGHT, padx=10)
            
            # Create variable and slider
            if isinstance(self.settings[setting_key], float):
                var = tk.DoubleVar(value=self.settings[setting_key])
                resolution = 0.01
            else:
                var = tk.IntVar(value=self.settings[setting_key])
                resolution = 1
                
            self.setting_vars[setting_key] = var
            
            slider = ttk.Scale(
                frame, 
                from_=min_val, 
                to=max_val,
                orient=tk.HORIZONTAL,
                length=300,
                variable=var
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
            
            # Update label and save value when slider changes
            slider.config(command=lambda val, key=setting_key, lbl=value_label: 
                        self.update_setting(key, val, lbl))
            
            # Description
            ttk.Label(frame, text=description, font=("Arial", 8), 
                    foreground="gray").pack(anchor=tk.W, padx=20)
        
        # Reset to defaults button
        reset_btn = ttk.Button(settings_frame, text="Reset to Defaults", 
                             command=self.reset_settings)
        reset_btn.pack(pady=10)

    def update_setting(self, key, value, label):
        """Update a setting when its slider changes"""
        # Convert string to appropriate type
        if isinstance(self.settings[key], float):
            self.settings[key] = float(value)
            label.config(text=f"{float(value):.2f}")
        else:
            self.settings[key] = int(float(value))
            label.config(text=str(int(float(value))))

    def reset_settings(self):
        """Reset all settings to defaults"""
        self.settings = self.DEFAULT_SETTINGS.copy()
        
        # Update all sliders
        for key, var in self.setting_vars.items():
            var.set(self.settings[key])

    def mouse_control_start(self, event, servo_name):
        """Start mouse control for a servo"""
        # Activate the servo
        self.mouse_servo = servo_name
        pin = self.SERVO_PINS[servo_name]
        
        # Initialize tracking
        canvas = event.widget
        self.mouse_start_x = event.x
        self.mouse_start_y = event.y
        self.mouse_last_pos = (event.x, event.y)
        self.mouse_last_time = time.time()
        
        # Turn on servo
        self.active_servos.add(servo_name)
        self.pi.set_servo_pulsewidth(pin, self.current_pw[servo_name])
        
        # Update status
        self.servo_status.config(text=f"Moving: {servo_name} (Mouse Control)")

    def mouse_control_move(self, event, servo_name):
        """Move servo based on mouse drag"""
        if self.mouse_servo != servo_name:
            return
            
        canvas = event.widget
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        
        # Calculate delta and direction
        delta_x = event.x - self.mouse_last_pos[0]
        delta_y = event.y - self.mouse_last_pos[1]
        
        # Update indicator position (constrain to canvas)
        oval = self.visual_controls[servo_name]["indicator"]
        canvas.move(oval, delta_x, delta_y)
        
        # Make sure indicator stays in bounds
        oval_coords = canvas.coords(oval)
        if oval_coords[0] < 0:
            canvas.move(oval, -oval_coords[0], 0)
        if oval_coords[1] < 0:
            canvas.move(oval, 0, -oval_coords[1])
        if oval_coords[2] > width:
            canvas.move(oval, width - oval_coords[2], 0)
        if oval_coords[3] > height:
            canvas.move(oval, 0, height - oval_coords[3])
        
        # Get distance from center
        center_x = width / 2
        center_y = height / 2
        current_x = (oval_coords[0] + oval_coords[2]) / 2
        current_y = (oval_coords[1] + oval_coords[3]) / 2
        
        # Calculate velocity based on distance from center
        x_ratio = (current_x - center_x) / (width / 2)
        y_ratio = (current_y - center_y) / (height / 2)
        
        # Use horizontal position for primary control
        normalized_ratio = x_ratio * self.settings["MOUSE_SENSITIVITY"]
        
        # Calculate target position offset and update
        min_pw, max_pw = self.SERVO_LIMITS[servo_name]
        range_pw = max_pw - min_pw
        middle_pw = (min_pw + max_pw) / 2
        
        # Set target position
        target_offset = normalized_ratio * range_pw / 2
        new_target = middle_pw + target_offset
        
        # Ensure within limits
        self.target_pw[servo_name] = max(min_pw, min(new_target, max_pw))
        
        # Update slider to reflect new position
        self.servo_sliders[servo_name].set(self.target_pw[servo_name])
        
        # Update last position
        self.mouse_last_pos = (event.x, event.y)
        self.mouse_last_time = time.time()

    def mouse_control_end(self, event, servo_name):
        """End mouse control for a servo"""
        if self.mouse_servo != servo_name:
            return
            
        # Reset mouse tracking
        self.mouse_servo = None
        
        # Reset the indicator to center position
        canvas = self.visual_controls[servo_name]["canvas"]
        oval = self.visual_controls[servo_name]["indicator"]
        
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        
        # Calculate center position
        center_x = width / 2
        center_y = height / 2
        size = 25  # Half the indicator size
        
        # Move indicator back to center
        current_x = (canvas.coords(oval)[0] + canvas.coords(oval)[2]) / 2
        current_y = (canvas.coords(oval)[1] + canvas.coords(oval)[3]) / 2
        
        canvas.move(oval, center_x - current_x, center_y - current_y)
        
        # Turn off servo after a short delay to allow for smooth stop
        self.root.after(500, lambda s=servo_name: self.delayed_servo_off(s))
        
        # Update status
        self.servo_status.config(text="Ready - All servos off")

    def delayed_servo_off(self, servo_name):
        """Turn off servo after delay (to allow for smooth stop)"""
        # Remove from active servos
        if servo_name in self.active_servos:
            self.active_servos.remove(servo_name)
            pin = self.SERVO_PINS[servo_name]
            self.pi.set_servo_pulsewidth(pin, 0)

    def on_slider_change(self, servo_name, value):
        """Handle slider position change"""
        # Update target position
        value = float(value)
        self.target_pw[servo_name] = value
        
        # Update position label
        self.position_labels[servo_name].config(text=f"{int(value)}μs")
        
        # If slider is being actively dragged, enable the servo
        if hasattr(self.root, 'focus_get') and self.root.focus_get() == self.servo_sliders[servo_name]:
            # Turn on servo
            if servo_name not in self.active_servos:
                self.active_servos.add(servo_name)
                pin = self.SERVO_PINS[servo_name]
                self.pi.set_servo_pulsewidth(pin, self.current_pw[servo_name])

    def create_servo_section(self, parent, group_name, servo_names, row):
        """Create a section of servo control buttons"""
        frame = ttk.LabelFrame(parent, text=group_name, padding=10)
        frame.grid(row=row, column=0, sticky="ew", padx=10, pady=5)
        
        button_container = ttk.Frame(frame)
        button_container.pack(fill=tk.X)
        
        # Ensure consistent width for button columns
        for i in range(len(servo_names)):
            button_container.columnconfigure(i, weight=1)
        
        for i, name in enumerate(servo_names):
            button_frame = ttk.Frame(button_container, padding=5)
            button_frame.grid(row=0, column=i, sticky="ew")
            
            # Show servo name
            ttk.Label(button_frame, text=name, font=("Arial", 10, "bold")).pack(pady=(0, 5))
            
            # Create button frame
            btn_frame = ttk.Frame(button_frame)
            btn_frame.pack()
            
            # CCW button
            ccw_btn = ttk.Button(btn_frame, text="←", width=3)
            ccw_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind events for CCW button
            button_id = f"{name} CCW"
            ccw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            ccw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))
            
            # CW button
            cw_btn = ttk.Button(btn_frame, text="→", width=3)
            cw_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind events for CW button
            button_id = f"{name} CW"
            cw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            cw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))

    def smooth_movement_loop(self):
        """Background thread that handles smooth movement of all servos"""
        while True:
            # Process each servo that has a target different from current position
            active_movement = False
            for name in self.SERVO_PINS.keys():
                # Skip inactive servos to save power
                if name not in self.active_servos and abs(self.target_pw[name] - self.current_pw[name]) < 1:
                    continue
                    
                # Calculate ideal movement with acceleration/deceleration
                target = self.target_pw[name]
                current = self.current_pw[name]
                diff = target - current
                
                # Apply acceleration/deceleration logic
                if abs(diff) > 1:  # Only move if difference is significant
                    active_movement = True
                    
                    # Calculate velocity target based on distance to target
                    distance_factor = min(1.0, abs(diff) / 100)  # Normalize distance effect
                    max_velocity = self.settings['STEP_SIZE'] * self.speed_factor.get() * distance_factor
                    
                    # Apply acceleration if moving toward target
                    if abs(self.velocities[name]) < max_velocity:
                        # Accelerate
                        self.velocities[name] += math.copysign(
                            self.settings['ACCELERATION'] * max_velocity,
                            diff
                        )
                        # Cap velocity
                        self.velocities[name] = math.copysign(
                            min(abs(self.velocities[name]), max_velocity),
                            self.velocities[name]
                        )
                    elif abs(diff) < abs(self.velocities[name]) * 2:
                        # Decelerate as we approach target
                        self.velocities[name] *= (1.0 - self.settings['DECELERATION'])
                    
                    # Ensure velocity direction matches target direction
                    if (self.velocities[name] * diff) < 0:
                        # If velocity direction opposes target direction, decelerate faster
                        self.velocities[name] *= (1.0 - self.settings['DECELERATION'] * 2)
                    
                    # Apply smoothing to current position
                    new_position = current + self.velocities[name]
                    
                    # Ensure within limits
                    min_pw, max_pw = self.SERVO_LIMITS[name]
                    new_position = max(min_pw, min(new_position, max_pw))
                    
                    # Apply movement with smoothing
                    self.current_pw[name] = (self.current_pw[name] * self.settings['SMOOTHING'] + 
                                            new_position * (1 - self.settings['SMOOTHING']))
                    
                    # Update servo position
                    pin = self.SERVO_PINS[name]
                    self.pi.set_servo_pulsewidth(pin, int(self.current_pw[name]))
                    
                    # Update position label and slider if available
                    if hasattr(self, 'position_labels') and name in self.position_labels:
                        self.root.after_idle(lambda n=name, p=int(self.current_pw[name]): 
                                            self.position_labels[n].config(text=f"{p}μs"))
                else:
                    # Reached target, stop velocity
                    self.velocities[name] = 0
                    
                    # If servo is active but no movement needed, update final position
                    if name in self.active_servos:
                        self.current_pw[name] = target
                        pin = self.SERVO_PINS[name]
                        self.pi.set_servo_pulsewidth(pin, int(target))
                    
            # Pause for movement interval
            time.sleep(self.settings['SPEED_INTERVAL'] / 1000.0)

    def on_closing(self):
        """Clean shutdown when window is closed"""
        # Make sure all servos are powered off
        self.all_servos_off()
        # Stop pigpio connection
        self.pi.stop()
        # Destroy window
        self.root.destroy()

    # Create main application
    if __name__ == "__main__":
        root = tk.Tk()
        app = RoboticArmController(root)
        # Set closing handler
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        # Start the event loop
        root.mainloop()