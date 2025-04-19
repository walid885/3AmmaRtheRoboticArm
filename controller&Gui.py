import tkinter as tk
import cv2
from PIL import Image, ImageTk
import pigpio
import threading
import time
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import io
import numpy as np

class RoboticArmController:
    # Servo pin configuration
    SERVO_PINS = {
        'BASE': 17,       # Base rotation
        'SHOULDER': 18,   # Shoulder joint
        'ELBOW': 19,      # Elbow joint
        'WRIST_PITCH': 20,  # Wrist up/down
        'WRIST_ROLL': 21,   # Wrist rotation
        'GRIPPER': 22     # End effector/gripper
    }
    
    # Min/Max pulse widths (in μs) for each servo
    SERVO_LIMITS = {
        'BASE': (500, 2500),
        'SHOULDER': (500, 2500),
        'ELBOW': (500, 2500),
        'WRIST_PITCH': (500, 2500),
        'WRIST_ROLL': (500, 2500),
        'GRIPPER': (1000, 2000)  # Closed to Open
    }
    
    # Default step size and interval for servo movements
    DEFAULT_STEP = 20      # Step size in μs
    DEFAULT_INTERVAL = 50  # Interval between steps in ms

    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Robotic Arm Control")
        self.root.geometry("1200x700")
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon!")
            tk.messagebox.showerror("Connection Error", "Failed to connect to pigpio daemon!")
            return
            
        # Current pulse width for each servo
        self.current_pw = {pin_name: 1500 for pin_name in self.SERVO_PINS}
        
        # Initialize all servos to middle position
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            
        # Button states tracking
        self.button_states = {}
        
        # Camera status
        self.camera_running = True
        self.camera_failed = False
        
        self._setup_ui()
        self.init_picamera()

    def _setup_ui(self):
        """Set up the user interface"""
        # Configure grid layout
        self.root.grid_columnconfigure(0, weight=3)  # Camera display
        self.root.grid_columnconfigure(1, weight=1)  # Controls
        
        # Camera display frame (left side)
        self.camera_frame = tk.Frame(self.root, bg="black", width=700, height=600)
        self.camera_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        # Camera label
        self.camera_label = tk.Label(self.camera_frame, bg="black")
        self.camera_label.pack(fill=tk.BOTH, expand=True)
        
        # Controls frame (right side)
        self.controls_frame = tk.Frame(self.root, bg="#f0f0f0", width=450, height=600)
        self.controls_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        
        # Status display
        self.status_label = tk.Label(self.controls_frame, text="Robotic Arm Status:", 
                                     font=("Arial", 14), bg="#f0f0f0")
        self.status_label.pack(pady=(20, 10))
        
        self.servo_status = tk.Label(self.controls_frame, text="No servo activated", 
                                     font=("Arial", 12), bg="white", width=30, height=2, 
                                     relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 20))
        
        # Position feedback
        self.feedback_frame = tk.Frame(self.controls_frame, bg="#f0f0f0")
        self.feedback_frame.pack(fill=tk.X, pady=10)
        
        self.position_labels = {}
        for i, name in enumerate(self.SERVO_PINS.keys()):
            row = i // 3
            col = i % 3
            
            label_frame = tk.Frame(self.feedback_frame, bg="#f0f0f0")
            label_frame.grid(row=row, column=col, padx=5, pady=5)
            
            tk.Label(label_frame, text=f"{name}:", font=("Arial", 9), 
                     bg="#f0f0f0").pack(side=tk.LEFT)
            
            self.position_labels[name] = tk.Label(label_frame, text="1500μs", 
                                                  font=("Arial", 9), width=7, 
                                                  bg="white", relief=tk.SUNKEN)
            self.position_labels[name].pack(side=tk.LEFT)
        
        # Create servo control sections
        self.create_servo_section("BASE & SHOULDER", ["BASE", "SHOULDER"], "#335577", "white")
        self.create_servo_section("ELBOW & WRISTS", ["ELBOW", "WRIST_PITCH", "WRIST_ROLL"], "#773355", "white")
        self.create_servo_section("GRIPPER", ["GRIPPER"], "#557733", "white")
        
        # Create preset positions buttons
        self.create_preset_buttons()

    def create_servo_section(self, group_name, servo_names, bg_color, fg_color):
        """Create a section of servo control buttons"""
        frame = tk.Frame(self.controls_frame, bg="#f0f0f0")
        frame.pack(pady=10, fill=tk.X)
        
        group_label = tk.Label(frame, text=group_name, font=("Arial", 12, "bold"), bg="#f0f0f0")
        group_label.pack(pady=(0, 5))
        
        button_container = tk.Frame(frame, bg="#f0f0f0")
        button_container.pack()
        
        for i, name in enumerate(servo_names):
            button_frame = tk.Frame(button_container, bg="#f0f0f0")
            button_frame.grid(row=0, column=i, padx=10)
            
            label = tk.Label(button_frame, text=name, font=("Arial", 10, "bold"), bg="#f0f0f0")
            label.pack(pady=(0, 5))
            
            # CCW button (usually increases pulse width)
            ccw_btn = tk.Button(button_frame, text="←", bg=bg_color, fg=fg_color,
                               font=("Arial", 10, "bold"), width=3, height=2)
            ccw_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind events with proper lambda to avoid late binding issues
            button_id = f"{name} CCW"
            ccw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            ccw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))
            
            # CW button (usually decreases pulse width)
            cw_btn = tk.Button(button_frame, text="→", bg=bg_color, fg=fg_color,
                              font=("Arial", 10, "bold"), width=3, height=2)
            cw_btn.pack(side=tk.LEFT, padx=2)
            
            button_id = f"{name} CW"
            cw_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            cw_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))

    def create_preset_buttons(self):
        """Create buttons for preset positions"""
        preset_frame = tk.Frame(self.controls_frame, bg="#f0f0f0")
        preset_frame.pack(pady=15, fill=tk.X)
        
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
        stop_btn = tk.Button(preset_frame, text="EMERGENCY STOP", command=self.emergency_stop,
                           bg="#FF3333", fg="white", font=("Arial", 12, "bold"),
                           width=20, height=2)
        stop_btn.pack(pady=15)

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
                
                # Ensure within limits
                min_pw, max_pw = self.SERVO_LIMITS[name]
                new_pw = max(min_pw, min(new_pw, max_pw))
                
                # Update position
                self.current_pw[name] = new_pw
                self.pi.set_servo_pulsewidth(pin, new_pw)
                
                # Update UI in main thread
                self.root.after(0, lambda n=name, p=new_pw: 
                               self.position_labels[n].config(text=f"{p}μs"))
            
            time.sleep(0.02)  # 20ms between steps
        
        # Update status when done
        self.root.after(0, lambda: self.servo_status.config(text="Preset position reached"))

    def emergency_stop(self):
        """Stop all servo movements immediately"""
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
        for name, pin in self.SERVO_PINS.items():
            self.pi.set_servo_pulsewidth(pin, self.current_pw[name])
            self.position_labels[name].config(text=f"{self.current_pw[name]}μs")
        
        self.servo_status.config(text="Ready")

    def button_press(self, button_id):
        """Handle button press events"""
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
        # Only proceed if button is still pressed
        if button_id in self.button_states and self.button_states[button_id]:
            # Parse button ID
            parts = button_id.split()
            servo_name = parts[0]
            direction = parts[1]  # CCW or CW
            
            # Get servo pin
            pin = self.SERVO_PINS[servo_name]
            
            # Get current pulse width
            current = self.current_pw[servo_name]
            
            # Get limits for this servo
            min_pw, max_pw = self.SERVO_LIMITS[servo_name]
            
            # Calculate new pulse width based on direction
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

    def init_picamera(self):
        """Initialize Pi Camera v2"""
        try:
            # Start camera capture in a separate thread
            self.camera_thread = threading.Thread(target=self.camera_loop)
            self.camera_thread.daemon = True
            self.camera_thread.start()
        except Exception as e:
            print(f"Pi Camera initialization error: {e}")
            self.handle_camera_failure()

    def camera_loop(self):
        """Continuous camera capture loop in a separate thread"""
        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (640, 480)
                camera.framerate = 30
                # Allow camera to warm up
                time.sleep(2)
                
                # Create in-memory stream
                stream = io.BytesIO()
                
                # Continuous capture
                while self.camera_running:
                    # Clear the stream and prepare for next frame
                    stream.seek(0)
                    stream.truncate()
                    
                    # Capture frame directly as JPEG
                    camera.capture(stream, format='jpeg', use_video_port=True)
                    
                    # Convert to a numpy array
                    stream.seek(0)
                    data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
                    
                    # Decode the image using OpenCV
                    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    
                    # Update the GUI with the new frame
                    self.update_frame(image)
                    
                    # Sleep briefly to reduce CPU usage
                    time.sleep(0.01)
        except Exception as e:
            print(f"Camera thread error: {e}")
            self.root.after(0, self.handle_camera_failure)

    def update_frame(self, frame):
        """Update the GUI with a new camera frame"""
        if not self.camera_running:
            return
            
        try:
            # Convert BGR to RGB for PIL
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image
            image = Image.fromarray(frame_rgb)
            
            # Resize to fit our display
            image = image.resize((700, 525), Image.LANCZOS)
            
            # Convert to PhotoImage for Tkinter
            photo = ImageTk.PhotoImage(image)
            
            # Update the label in the main thread
            self.root.after(0, lambda: self.update_label(photo))
        except Exception as e:
            print(f"Frame update error: {e}")

    def update_label(self, photo):
        """Update the label with a new photo (called in main thread)"""
        try:
            self.photo = photo  # Store reference to prevent garbage collection
            self.camera_label.config(image=self.photo)
        except Exception as e:
            print(f"Label update error: {e}")

    def handle_camera_failure(self):
        """Handle camera initialization failure"""
        self.camera_failed = True
        # Show a placeholder
        placeholder = Image.new('RGB', (700, 525), color='darkgray')
        self.photo = ImageTk.PhotoImage(placeholder)
        self.camera_label.config(image=self.photo)
        
        # Add text overlay
        if hasattr(self, 'text_label'):
            self.text_label.destroy()
        
        self.text_label = tk.Label(self.camera_label, text="Camera Not Available\nClick to retry",
                            font=("Arial", 18), bg="darkgray", fg="white")
        self.text_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        
        # Add click to retry
        self.camera_label.bind("<Button-1>", lambda e: self.retry_camera())

    def retry_camera(self):
        """Attempt to reconnect to camera"""
        print("Attempting to reconnect to camera...")
        self.camera_failed = False
        self.init_picamera()

    def cleanup(self):
        """Clean up resources when closing the application"""
        print("Cleaning up resources...")
        
        # Stop camera thread
        self.camera_running = False
        if hasattr(self, 'camera_thread'):
            time.sleep(0.5)  # Give camera thread time to stop
        
        # Clear any button states
        self.button_states.clear()
        
        # Turn off all servos by stopping pulses
        if hasattr(self, 'pi') and self.pi.connected:
            print("Turning off servo motors...")
            for name, pin in self.SERVO_PINS.items():
                self.pi.set_servo_pulsewidth(pin, 0)  # Stop the pulse
                time.sleep(0.1)  # Brief pause between commands
            
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
    
    # Close camera and GPIO properly when window is closed
    root.protocol("WM_DELETE_WINDOW", lambda: [app.cleanup(), root.destroy()])
    
    root.mainloop()

if __name__ == "__main__":
    main()