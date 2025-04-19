import tkinter as tk
import cv2
from PIL import Image, ImageTk
import pigpio
import threading
import time

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm Control GUI")
        self.root.geometry("1000x600")

        # Servo configuration
        self.GRIPPER_PIN = 17
        self.OPEN_PW = 2000   # Pulse width for open position (μs)
        self.CLOSED_PW = 1000  # Pulse width for closed position (μs)
        self.SERVO_STEP = 20  # Step size for gradual movement (μs)
        self.SERVO_INTERVAL = 50  # Interval between steps (ms)
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon!")
            # Show error in GUI
            tk.messagebox.showerror("Connection Error", "Failed to connect to pigpio daemon!")

        # Set gripper to middle position initially
        self.current_pw = 1500
        self.pi.set_servo_pulsewidth(self.GRIPPER_PIN, self.current_pw)

        # Configure grid layout
        self.root.grid_columnconfigure(0, weight=3) # Camera display
        self.root.grid_columnconfigure(1, weight=1) # Controls

        # Camera display frame (left side)
        self.camera_frame = tk.Frame(root, bg="black", width=600, height=600)
        self.camera_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Camera label
        self.camera_label = tk.Label(self.camera_frame, bg="black")
        self.camera_label.pack(fill=tk.BOTH, expand=True)

        # Controls frame (right side)
        self.controls_frame = tk.Frame(root, bg="#f0f0f0", width=400, height=600)
        self.controls_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Status display
        self.status_label = tk.Label(self.controls_frame, text="Robotic Arm Status:", font=("Arial", 14), bg="#f0f0f0")
        self.status_label.pack(pady=(20, 10))

        self.servo_status = tk.Label(self.controls_frame, text="No servo activated", font=("Arial", 12), bg="white",
                                  width=25, height=2, relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 20))

        # Servo control buttons
        self.create_button_frame("BLACK", ["B1", "B2", "B3"], "#333333", "white")
        self.create_button_frame("BLUE", ["B1", "B2", "B3"], "#3366cc", "white")

        # Dictionary to track button states (pressed or released)
        self.button_states = {}
        
        # Camera failure flag
        self.camera_failed = False

        # Initialize camera
        self.init_camera()

    def create_button_frame(self, group_name, button_names, bg_color, fg_color):
        frame = tk.Frame(self.controls_frame, bg="#f0f0f0")
        frame.pack(pady=10, fill=tk.X)

        group_label = tk.Label(frame, text=f"{group_name} SERVOS", font=("Arial", 12, "bold"), bg="#f0f0f0")
        group_label.pack(pady=(0, 5))

        button_container = tk.Frame(frame, bg="#f0f0f0")
        button_container.pack()

        for i, name in enumerate(button_names):
            button_frame = tk.Frame(button_container, bg="#f0f0f0")
            button_frame.grid(row=0, column=i, padx=10)
            
            label = tk.Label(button_frame, text=name, font=("Arial", 10, "bold"), bg="#f0f0f0")
            label.pack(pady=(0, 5))
            
            # Left button (counterclockwise)
            left_btn = tk.Button(button_frame, text="←", bg=bg_color, fg=fg_color,
                               font=("Arial", 10, "bold"), width=3, height=2)
            left_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind mouse events to the left button
            button_id = f"{group_name} {name} CCW"
            # Use lambda with default arguments to avoid late binding issues
            left_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            left_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))
            
            # Right button (clockwise)
            right_btn = tk.Button(button_frame, text="→", bg=bg_color, fg=fg_color,
                               font=("Arial", 10, "bold"), width=3, height=2)
            right_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind mouse events to the right button
            button_id = f"{group_name} {name} CW"
            right_btn.bind("<ButtonPress-1>", lambda event, bid=button_id: self.button_press(bid))
            right_btn.bind("<ButtonRelease-1>", lambda event, bid=button_id: self.button_release(bid))

    def button_press(self, button_id):
        # Mark button as pressed
        self.button_states[button_id] = True
        
        # Update status display
        self.servo_status.config(text=f"Activated: {button_id}")
        
        # Print information
        print(f"{button_id}")
        
        # Start continuous movement for the gripper
        if "BLACK B3" in button_id or "BLUE B3" in button_id:
            self.move_servo_continuously(button_id)

    def move_servo_continuously(self, button_id):
        # Only proceed if button is still pressed
        if button_id in self.button_states and self.button_states[button_id]:
            # Determine direction
            if "CCW" in button_id:  # Open gripper
                new_pw = min(self.current_pw + self.SERVO_STEP, self.OPEN_PW)
            else:  # Close gripper
                new_pw = max(self.current_pw - self.SERVO_STEP, self.CLOSED_PW)
            
            # Update position if it changed
            if new_pw != self.current_pw:
                self.current_pw = new_pw
                self.pi.set_servo_pulsewidth(self.GRIPPER_PIN, self.current_pw)
                print(f"Gripper position: {self.current_pw}μs")
            
            # Schedule the next movement only if button is still pressed
            self.root.after(self.SERVO_INTERVAL, lambda: self.move_servo_continuously(button_id))

    def button_release(self, button_id):
        # Check if this button was active
        if button_id in self.button_states:
            # Mark button as released
            self.button_states[button_id] = False
            
            # Print release information
            print(f"RELEASED: {button_id}")
            
            # Remove from button states
            del self.button_states[button_id]
            
            # Update status to show no active servo
            if not self.button_states:
                self.servo_status.config(text="No servo activated")

    def get_current_pw(self, pin):
        """Get current pulse width or return middle position if servo is off"""
        current_pw = self.pi.get_servo_pulsewidth(pin)
        if current_pw == 0:  # If servo is off
            return 1500  # Return middle position
        return current_pw

    def init_camera(self):
        try:
            # Explicitly release any existing camera first
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                time.sleep(0.5)  # Give the camera time to close properly
            
            # Try to connect to camera (0 is usually the default webcam)
            self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                self.handle_camera_failure()
            else:
                # If camera is available, update frames
                self.camera_failed = False
                self.update_camera()
        except Exception as e:
            print(f"Camera initialization error: {e}")
            self.handle_camera_failure()

    def handle_camera_failure(self):
        self.camera_failed = True
        # If no camera is available, show a placeholder
        placeholder = Image.new('RGB', (640, 480), color='darkgray')
        self.photo = ImageTk.PhotoImage(placeholder)
        self.camera_label.config(image=self.photo)
        self.camera_label.image = self.photo
        
        # Add text overlay to camera_label
        if hasattr(self, 'text_label'):
            self.text_label.destroy()
        
        self.text_label = tk.Label(self.camera_label, text="Camera Not Available\nClick to retry",
                            font=("Arial", 18), bg="darkgray", fg="white")
        self.text_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        
        # Add click to retry
        self.camera_label.bind("<Button-1>", lambda e: self.retry_camera())

    def retry_camera(self):
        print("Attempting to reconnect to camera...")
        self.init_camera()

    def update_camera(self):
        if self.camera_failed:
            return
            
        try:
            ret, frame = self.cap.read()
            if ret:
                # Convert from BGR (OpenCV format) to RGB (PIL format)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Convert to PIL Image and then to Tkinter PhotoImage
                image = Image.fromarray(frame)
                # Resize to fit our display if needed
                image = image.resize((600, 450), Image.LANCZOS)
                self.photo = ImageTk.PhotoImage(image)
                
                # Update the label
                self.camera_label.config(image=self.photo)
                self.camera_label.image = self.photo # Keep a reference
            else:
                print("Failed to get frame from camera")
                self.handle_camera_failure()
                return
            
            # Update every 10ms (approximately 100 fps max)
            self.root.after(10, self.update_camera)
        except Exception as e:
            print(f"Camera update error: {e}")
            self.handle_camera_failure()

    def cleanup(self):
        # Clear any button states
        self.button_states.clear()
        
        # Turn off the servo
        if hasattr(self, 'pi') and self.pi.connected:
            self.pi.set_servo_pulsewidth(self.GRIPPER_PIN, 0)
            self.pi.stop()
            
        # Release camera
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        except Exception as e:
            print(f"Error releasing camera: {e}")

if __name__ == "__main__":
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
    app = RoboticArmGUI(root)
    
    # Close camera and GPIO properly when window is closed
    root.protocol("WM_DELETE_WINDOW", lambda: [app.cleanup(), root.destroy()])
    
    root.mainloop()